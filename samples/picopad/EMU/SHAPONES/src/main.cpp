#include "../include.h"

#include "mono8x16.hpp"
#include "shapones/shapones.hpp"

static constexpr int FRAME_BUFF_STRIDE = nes::SCREEN_WIDTH * 3 / 2;
static constexpr int SOUND_FREQ = 22050;
static constexpr int SOUND_BUFF_SIZE = 256;

static constexpr uint32_t ROM_OFFSET = 0x100F0000;

static uint8_t line_buff[nes::SCREEN_WIDTH];
static uint8_t frame_buff[FRAME_BUFF_STRIDE * nes::SCREEN_HEIGHT];

static uint8_t sound_buff[SOUND_BUFF_SIZE * 2];
static int sound_buff_index = 0;

static uint8_t *ines_rom = nullptr;

static int fps_frame_count = 0;
static uint64_t fps_last_meas_ms = 0;
static char fps_str[16];

static volatile bool disp_update_req = false;

// NES color table
static const uint16_t COLOR_TABLE[] = {
    0x555, 0x027, 0x019, 0x308, 0x406, 0x603, 0x500, 0x410, 0x230, 0x040, 0x040,
    0x040, 0x034, 0x000, 0x000, 0x000, 0x999, 0x05C, 0x33F, 0x62E, 0x81B, 0xA16,
    0x922, 0x740, 0x560, 0x270, 0x080, 0x072, 0x067, 0x000, 0x000, 0x000, 0xFFF,
    0x5AF, 0x78F, 0xB6F, 0xE5F, 0xF5B, 0xF76, 0xD82, 0xAB0, 0x7C0, 0x5D2, 0x3D7,
    0x3BD, 0x444, 0x000, 0x000, 0xFFF, 0xADF, 0xCCF, 0xDBF, 0xFBF, 0xFBD, 0xFBB,
    0xEC9, 0xDD7, 0xBE7, 0xAE9, 0x9EB, 0xADE, 0xAAA, 0x000, 0x000,
};

static int disp_dma_ch;
static spi_hw_t *disp_spi_hw = nullptr;
static dma_channel_hw_t *disp_dma_hw = nullptr;

uint64_t disp_next_vsync_us = 0;

static const uint8_t *sound_refill();

static constexpr int LOCK_ID_BASE = 0;

static void boot_menu();
static void boot_error(const char *line1, const char *line2 = "");

static void core0_main();
static void core1_main();

static void disp_clear(uint16_t color = COL_BLACK);
static void disp_fill_rect(int x, int y, int w, int h, uint16_t color);
static void disp_enable_rgb444();
static void disp_flip();
static void disp_dma_start();
static void disp_dma_complete();
static bool disp_wait_vsync();

int main() {
#if USE_PICOPAD10 || USE_PICOPAD20
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    WaitMs(100);
    set_sys_clock_khz(250000, true);
#endif

    boot_menu();

    core0_main();

    return 0;
}

static void boot_menu() {
    constexpr int LINE_H = 16;
    constexpr int MAX_FILES = HEIGHT / LINE_H;
    const char *INES_DIR = "/EMU/SHAPONES/";
    char ines_path[256];
    int ines_size = 0;

    int num_files = 0;
    char **names = new char *[MAX_FILES];
    int *sizes = new int[MAX_FILES];

    do {
        disp_clear(COL_WHITE);

        // Mount disk
        if (!DiskMount()) {
            DispDrawText("INSERT DISK.", 64, (HEIGHT - LINE_H) / 2, 0, 0,
                         COL_BLACK, COL_WHITE);
        }
        while (!DiskMount()) {
            if (KeyGet() == KEY_Y) {
                boot_error("Boot cancelled.");
            }
        }

        // List up NES files
        sFile find;
        if (!FindOpen(&find, INES_DIR)) {
            boot_error("DIRECTORY NOT FOUND:", INES_DIR);
        }
        sFileInfo fi;
        while (num_files < MAX_FILES &&
               FindNext(&find, &fi, ATTR_ARCH, "*.NES")) {
            if (fi.namelen < 4) continue;
            names[num_files] = new char[fi.namelen + 1];
            memcpy(names[num_files], fi.name, fi.namelen + 1);
            sizes[num_files] = fi.size;
            num_files++;
        }
        FindClose(&find);

        if (num_files == 0) {
            boot_error("NO NES FILE FOUND IN:", INES_DIR);
        }

        // Render file list
        for (int i = 0; i < num_files; i++) {
            DispDrawText(names[i], 32, i * LINE_H, 0, 0, COL_BLACK, COL_WHITE);
        }
        DispDrawText("=>", 8, 0, 0, 0, COL_BLUE, COL_WHITE);

        // Wait file to be selected
        int sel_index = 0;
        while (true) {
            uint8_t key = KeyGet();
            if (key == KEY_UP || key == KEY_DOWN) {
                disp_fill_rect(0, sel_index * LINE_H, 32, LINE_H, COL_WHITE);
                if (key == KEY_UP) {
                    sel_index = (sel_index + num_files - 1) % num_files;
                } else {
                    sel_index = (sel_index + 1) % num_files;
                }
                DispDrawText("=>", 8, sel_index * LINE_H, 0, 0, COL_BLUE,
                             COL_WHITE);
            } else if (key == KEY_A) {
                snprintf(ines_path, sizeof(ines_path), "%s%s", INES_DIR,
                         names[sel_index]);
                ines_size = sizes[sel_index];
                break;
            } else if (key == KEY_Y) {
                break;
            }
        };
    } while (false);

    // Free file list
    for (int i = 0; i < num_files; i++) {
        delete[] names[i];
    }
    delete[] names;
    delete[] sizes;

    // Load NES file
    sFile ines_file;
    if (!FileOpen(&ines_file, ines_path)) {
        boot_error("FAILED TO OPEN FILE.");
    }
    if (ines_size < (128 + 1) * 1024) {
        // Load to RAM
        ines_rom = new uint8_t[ines_size];
        int ret = FileRead(&ines_file, ines_rom, ines_size);
        if (ret < ines_size) {
            char ret_str[32];
            snprintf(ret_str, sizeof(ret_str), "Code: %d", ret);
            boot_error("FAILED TO LOAD FILE.", ret_str);
        }
    } else if (ines_size < (1024 + 1) * 1024) {
        // Load to Flash
        constexpr int CHUNK_SIZE = 4096;
        FlashErase(ROM_OFFSET, ines_size);
        uint8_t *buff = new uint8_t[CHUNK_SIZE];
        int remaining = ines_size;
        while (remaining > 0) {
            int to_read = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
            int ret = FileRead(&ines_file, buff, to_read);
            if (ret <= 0) {
                char ret_str[32];
                snprintf(ret_str, sizeof(ret_str), "Code: %d", ret);
                boot_error("FAILED TO LOAD FILE.", ret_str);
            }
            FlashProgram(ROM_OFFSET + (ines_size - remaining), buff, ret);
            remaining -= ret;
        }
        delete[] buff;
        ines_rom = (uint8_t *)ROM_OFFSET;
    } else {
        char ret_str[32];
        snprintf(ret_str, sizeof(ret_str), "NES FILE TOO LARGE (%d KB)",
                 ines_size / 1024);
        boot_error(ret_str);
    }
    FileClose(&ines_file);

    // Map NES file to memory
    nes::memory::map_ines(ines_rom);
}

static void boot_error(const char *line1, const char *line2) {
    constexpr int CHAR_W = 8;
    constexpr int LINE_H = 16;
    disp_clear(COL_WHITE);
    DispDrawText(line1, 64, HEIGHT / 2 - LINE_H, 0, 0, COL_RED, COL_WHITE);
    DispDrawText(line2, 64, HEIGHT / 2, 0, 0, COL_RED, COL_WHITE);
    DispDrawText("Puress Y to exit.", (WIDTH - (CHAR_W * 16)) / 2,
                 HEIGHT - LINE_H * 2, 0, 0, COL_BLACK, COL_WHITE);
    while (KeyGet() != KEY_Y) {
    }
    ResetToBootLoader();
}

static void core0_main() {
    disp_clear(COL_BLACK);

    // Change display mode to RGB444
    disp_spi_hw = SPI_GetHw(DISP_SPI);
    disp_enable_rgb444();

    // Setup NES core
    auto cfg = nes::get_default_config();
    cfg.apu_sampling_rate = SOUND_FREQ;
    nes::init(cfg);

    // Run PPU
    Core1Exec(core1_main);

    // Start Sound
    {
        const uint8_t *sound = sound_refill();
        PlaySoundChan(0, sound, SOUND_BUFF_SIZE, SNDREPEAT_STREAM,
                      SOUND_FREQ / SOUNDRATE, 1.0f, SNDFORM_PCM, 0);
    }

    // Run CPU/APU
    while (true) {
        uint64_t now_ms = Time64() / 1000;

        nes::input::InputStatus is;
        is.raw = 0;
        if (KeyPressedFast(KEY_LEFT)) is.left = 1;
        if (KeyPressedFast(KEY_RIGHT)) is.right = 1;
        if (KeyPressedFast(KEY_UP)) is.up = 1;
        if (KeyPressedFast(KEY_DOWN)) is.down = 1;
        if (KeyPressedFast(KEY_A)) is.B = 1;
        if (KeyPressedFast(KEY_B)) is.A = 1;
        if (KeyPressedFast(KEY_X)) is.select = 1;
        if (KeyPressedFast(KEY_Y)) is.start = 1;
        nes::input::set_raw(0, is);

        nes::cpu::service();

        // stream sound
        if (SoundStreamIsEmpty(0)) {
            const uint8_t *sound = sound_refill();
            SoundStreamSetNext(0, sound, SOUND_BUFF_SIZE);
        }
    }
}

static void core1_main() {
    bool skip_frame = false;

    while (true) {
        int y;
        uint32_t flags = nes::ppu::service(line_buff, skip_frame, &y);

        // end of visible line
        if ((flags & nes::ppu::END_OF_VISIBLE_LINE) && !skip_frame) {
            // Convert palette index to RGB444
            uint8_t *rd_ptr = line_buff;
            uint8_t *wr_ptr = frame_buff + (y * FRAME_BUFF_STRIDE);
            for (int x = 0; x < nes::SCREEN_WIDTH; x += 2) {
                uint16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                uint16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                *(wr_ptr++) = (c0 >> 4) & 0xff;
                *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
                *(wr_ptr++) = c1 & 0xff;
            }
        }
        if (flags & nes::ppu::END_OF_VISIBLE_AREA) {
            if (!skip_frame) {
                disp_flip();
            }
            skip_frame = disp_wait_vsync();
        }
    }
}

static void disp_clear(uint16_t color) {
    disp_fill_rect(0, 0, WIDTH, HEIGHT, color);
}

static void disp_fill_rect(int x, int y, int w, int h, uint16_t color) {
    DispStartImg(x, x + w, y, y + w);
    for (int i = 0; i < w * h; i++) {
        DispSendImg2(color);
    }
    DispStopImg();
}

static void disp_enable_rgb444() {
    uint8_t cmd = 0x3A;
    uint8_t data = 0x03;
    GPIO_Out0(DISP_CS_PIN);
    GPIO_Out0(DISP_DC_PIN);
    SPI_Send8(DISP_SPI, &cmd, 1);
    GPIO_Out1(DISP_DC_PIN);
    SPI_Send8(DISP_SPI, &data, 1);
    GPIO_Out1(DISP_CS_PIN);
}

static void disp_flip() {
    uint64_t now_ms = Time64() / 1000;

    // measure FPS
    fps_frame_count += 1;
    uint32_t t_elapsed = now_ms - fps_last_meas_ms;
    if (t_elapsed >= 1000) {
        float fps = (float)(fps_frame_count * 1000) / t_elapsed;
        snprintf(fps_str, sizeof(fps_str), "%5.2f FPS", fps);
        fps_last_meas_ms = now_ms;
        fps_frame_count = 0;
    }

    mono8x16::draw_string_rgb444(frame_buff, FRAME_BUFF_STRIDE,
                                 nes::SCREEN_WIDTH, nes::SCREEN_HEIGHT, 0, 0,
                                 fps_str, 0xFFF);

    // complete previous DMA
    disp_dma_complete();
    GPIO_Out1(DISP_CS_PIN);

    // select window
    constexpr int x_offset = (WIDTH - nes::SCREEN_WIDTH) / 2;
    DispWindow(x_offset, x_offset + nes::SCREEN_WIDTH, 0, nes::SCREEN_HEIGHT);

    // kick new DMA
    GPIO_Out0(DISP_CS_PIN);
    GPIO_Out1(DISP_DC_PIN);
    disp_dma_start();
}

static void disp_dma_start() {
    disp_dma_ch = DMA_TEMP_CHAN();
    disp_dma_hw = DMA_GetHw(disp_dma_ch);

    DMA_Abort(disp_dma_ch);
    DMA_ClearError_hw(disp_dma_hw);
    DMA_SetRead_hw(disp_dma_hw, frame_buff);
    DMA_SetWrite_hw(disp_dma_hw, &disp_spi_hw->dr);
    DMA_SetCount_hw(disp_dma_hw, FRAME_BUFF_STRIDE * nes::SCREEN_HEIGHT);
    cb();
    DMA_SetCtrlTrig_hw(disp_dma_hw,
                       DMA_CTRL_TREQ(SPI_GetDreq_hw(disp_spi_hw, True)) |
                           DMA_CTRL_CHAIN(disp_dma_ch) | DMA_CTRL_INC_READ |
                           DMA_CTRL_SIZE(DMA_SIZE_8) | DMA_CTRL_EN);
}

static void disp_dma_complete() {
    if (!disp_dma_hw) return;

    DMA_Wait_hw(disp_dma_hw);
    DMA_Disable_hw(disp_dma_hw);

    while (SPI_IsBusy_hw(disp_spi_hw)) {
        SPI_RxFlush_hw(disp_spi_hw);
    }

    SPI_RxFlush_hw(disp_spi_hw);
    SPI_RxOverClear_hw(disp_spi_hw);
}

static bool disp_wait_vsync() {
    constexpr uint32_t FRAME_INTERVAL_US = 1000000 / 60;
    uint64_t now_us = Time64();
    int64_t wait_us = disp_next_vsync_us - now_us;
    bool delaying = (wait_us <= 0);
    if (!delaying) {
        WaitUs(wait_us);
    }
    disp_next_vsync_us += FRAME_INTERVAL_US;
    if (now_us > disp_next_vsync_us) {
        disp_next_vsync_us = now_us;
    }
    return delaying;
}

static const uint8_t *sound_refill() {
    uint8_t *ptr = sound_buff + sound_buff_index * SOUND_BUFF_SIZE;
    nes::apu::service(ptr, SOUND_BUFF_SIZE);
    sound_buff_index = (sound_buff_index + 1) & 1;
    return ptr;
}

void nes::lock_init(int id) { SpinClaim(LOCK_ID_BASE + id); }
void nes::lock_deinit(int id) { SpinUnclaim(LOCK_ID_BASE + id); }
void nes::lock_get(int id) { SpinLock(LOCK_ID_BASE + id); }
void nes::lock_release(int id) { SpinUnlock(LOCK_ID_BASE + id); }
