#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/time.h"

#include <string.h>

#include "ws19804.hpp"

#include "pwm_audio.hpp"

#include "shapones/shapones.hpp"
#include "nes_rom.hpp"

#include "mono8x16.hpp"

#include "ff.h"
#include "diskio.h"

static constexpr uint32_t SYS_CLK_FREQ = 250 * MHZ;

static constexpr int PIN_MONITOR = 1;

static constexpr int PIN_PAD_A      = 2;
static constexpr int PIN_PAD_B      = 3;
static constexpr int PIN_PAD_START  = 4;
static constexpr int PIN_PAD_SELECT = 6;
static constexpr int PIN_PAD_RIGHT  = 7;
static constexpr int PIN_PAD_DOWN   = 14;
static constexpr int PIN_PAD_LEFT   = 26;
static constexpr int PIN_PAD_UP     = 27;

static constexpr int PIN_SPEAKER = 28;
static constexpr int SPK_LATENCY = 256;
static constexpr int SPK_PWM_FREQ = 22050;

static constexpr int LINE_FIFO_DEPTH = 8;
static constexpr int LINE_FIFO_STRIDE = nes::SCREEN_WIDTH + 1;

static bool boot_menu();
static void boot_nes();
static void cpu_loop();
static void ppu_loop();
static void apu_dma_handler();
static void apu_fill_buffer(PwmAudio::sample_t *buff);

PwmAudio speaker(PIN_SPEAKER, SPK_LATENCY, 8, (float)SYS_CLK_FREQ / SPK_PWM_FREQ, apu_dma_handler);

static const uint16_t COLOR_TABLE[] = {
    0x555, 0x027, 0x019, 0x308, 0x406, 0x603, 0x500, 0x410, 
    0x230, 0x040, 0x040, 0x040, 0x034, 0x000, 0x000, 0x000, 
    0x999, 0x05C, 0x33F, 0x62E, 0x81B, 0xA16, 0x922, 0x740, 
    0x560, 0x270, 0x080, 0x072, 0x067, 0x000, 0x000, 0x000, 
    0xFFF, 0x5AF, 0x78F, 0xB6F, 0xE5F, 0xF5B, 0xF76, 0xD82, 
    0xAB0, 0x7C0, 0x5D2, 0x3D7, 0x3BD, 0x444, 0x000, 0x000, 
    0xFFF, 0xADF, 0xCCF, 0xDBF, 0xFBF, 0xFBD, 0xFBB, 0xEC9, 
    0xDD7, 0xBE7, 0xAE9, 0x9EB, 0xADE, 0xAAA, 0x000, 0x000, 
};

static const int input_pins[] = {
    PIN_PAD_A, PIN_PAD_B, PIN_PAD_SELECT, PIN_PAD_START,
    PIN_PAD_UP, PIN_PAD_DOWN, PIN_PAD_LEFT, PIN_PAD_RIGHT
};

static uint8_t line_buff[LINE_FIFO_DEPTH * LINE_FIFO_STRIDE];
static volatile int line_fifo_wptr = 0;
static volatile int line_fifo_rptr = 0;
static uint8_t frame_buff[240*240*3/2];
static uint8_t spk_buff[SPK_LATENCY];

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(100);
    stdio_init_all();
    set_sys_clock_khz(SYS_CLK_FREQ / 1000, true);
    setup_default_uart();
    
    gpio_init(PIN_MONITOR);
    gpio_set_dir(PIN_MONITOR, GPIO_OUT);
    gpio_put(PIN_MONITOR, 0);
    
    for(int i = 0; i < 8; i++) {
        int pin = input_pins[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
    
    ws19804::init(SYS_CLK_FREQ);

    if ( ! boot_menu()) {
        for(;;) sleep_ms(100);
    }
    boot_nes();

    return 0;
}

static void draw_string(int x, int y, const char *str) {
    mono8x16::draw_string_rgb444(
        frame_buff, 240 * 3 / 2, 240, 240,
        x, y, str, 0xfff);
}

extern uint8_t sd_resp[20];

static void clear_frame_buff() {
    for (int i = 0; i < 240 * 240 * 3 / 2; i++) {
        frame_buff[i] = 0;
    }
}

static void update_lcd() {
    ws19804::start_write_data(25, 0, 240, 240, frame_buff);
    ws19804::finish_write_data();
}

constexpr int MAX_FILES = 64;

static int enum_files(FATFS *fs, char **fname_list, int *fsize_list) {
    char tmp[16];
    int y = 20;
    constexpr int x_result = 120;

    DSTATUS dsts;
    FRESULT fres;

    clear_frame_buff();

    // Init FatFS
    draw_string(0, y, "Init FatFS"); update_lcd();
    dsts = disk_initialize(0);
    if (dsts & STA_NOINIT) {
        sprintf(tmp, "[NG] code=0x%x", (int)dsts);
        draw_string(x_result, y, tmp); update_lcd();
        return -1;
    }
    draw_string(x_result, y, "[OK]"); update_lcd();
    y += 20;

    // Mount
    draw_string(0, y, "Disk Mount"); update_lcd();
    fres = f_mount(fs, "", 0);
    if (fres != FR_OK) {
        sprintf(tmp, "[NG] code=0x%x", (int)fres);
        draw_string(x_result, y, tmp); update_lcd();
        return -1;
    }
    draw_string(x_result, y, "[OK]"); update_lcd();
    y += 20;

    // Enumerate File
    draw_string(0, y, "File List"); update_lcd();

    DIR dobj;
    FILINFO finfo;    /* ファイル情報 */
    fres = f_findfirst(&dobj, &finfo, "", "*.nes");
    int num_files = 0;
    while (fres == FR_OK && finfo.fname[0]) {
        fname_list[num_files] = (char*)malloc(strlen(finfo.fname) + 1);
        strcpy(fname_list[num_files], finfo.fname);
        fsize_list[num_files] = finfo.fsize;
        fres = f_findnext(&dobj, &finfo);
        num_files++;
    }
    if (fres != FR_OK) {
        sprintf(tmp, "[NG] code=0x%x", (int)fres);
        draw_string(x_result, y, tmp); update_lcd();
        return -1;
    }
    f_closedir(&dobj);
    draw_string(x_result, y, "[OK]"); update_lcd();
    y += 20;

    return num_files;
}

static int wait_key() {
    // wait all button released
    int num_pushed;
    do {
        sleep_ms(10);
        num_pushed = 0;
        for (int i = 0; i < 8; i++) {
            if ( ! gpio_get(input_pins[i])) {
                num_pushed++;
            }
        }
    } while(num_pushed != 0);

    // wait any button pushed
    for(;;) {
        sleep_ms(10);
        for (int i = 0; i < 8; i++) {
            if ( ! gpio_get(input_pins[i])) {
                return i;
            }
        }
    }
}

static int rom_select(int num_files, char **file_list) {
    int sel_index = 0;

    for(;;) {
        clear_frame_buff();
        for(int i = 0; i < num_files; i++) {
            draw_string(20, i * 20, file_list[i]);
        }
        draw_string(0, sel_index * 20, "=>");
        update_lcd();

        switch(wait_key()) {
        case nes::input::BTN_UP:
            sel_index = (sel_index + num_files - 1) % num_files;
            break;
        case nes::input::BTN_DOWN:
            sel_index = (sel_index + 1) % num_files;
            break;
        case nes::input::BTN_A:
        case nes::input::BTN_START:
            return sel_index;
        }
    }
}

static bool load_nes(const char *fname, int size) {
    FIL fil;
    FRESULT fr;

    clear_frame_buff();
    draw_string(0, 0, "Loading...");
    update_lcd();

    fr = f_open(&fil, fname, FA_READ);
    if (fr) {
        draw_string(0, 20, "File open failed.");
        update_lcd();
        return false;
    }

    UINT sz;
    uint8_t *ines = (uint8_t*)malloc(size);
    fr = f_read(&fil, ines, size, &sz);
    if (fr) {
        draw_string(0, 20, "File read failed.");
        update_lcd();
        return false;
    }

    f_close(&fil);

    nes::memory::map_ines(ines);
    ws19804::set_speed(SYS_CLK_FREQ / 4);

    return true;
}

static bool boot_menu() {
    FATFS fs;
    char *fname_list[MAX_FILES];
    int fsize_list[MAX_FILES];
    int num_files = enum_files(&fs, fname_list, fsize_list);
    if (num_files < 0) {
        return false;
    }

    int index = rom_select(num_files, fname_list);

    if ( ! load_nes(fname_list[index], fsize_list[index])) {
        return false;
    }

    for (int i = 0; i < num_files; i++) {
        free(fname_list[i]);
    }
    
    return true;
}

static void boot_nes() {
    //nes::memory::map_ines(nes_rom);
    nes::reset();
    nes::apu::set_sampling_rate(SPK_PWM_FREQ);

    apu_fill_buffer(speaker.get_buffer(0));
    apu_fill_buffer(speaker.get_buffer(1));
    speaker.play();
    
    multicore_launch_core1(ppu_loop);

    cpu_loop();
}

static void cpu_loop() {
    auto t_last_frame = get_absolute_time();
    int frame_count = 0;
    char fps_str[16];
    for(;;) {
        nes::cpu::service();
        
        nes::input::InputStatus input_status;
        input_status.raw = 0;
        for(int i = 0; i < 8; i++) {
            if ( ! gpio_get(input_pins[i])) {
                input_status.raw |= (1 << i);
            }
        }
        nes::input::set_raw(0, input_status);

        int fifo_rptr = line_fifo_rptr;
        if (line_fifo_wptr != fifo_rptr) {
            // convert color number to RGB444
            int y = line_buff[fifo_rptr * LINE_FIFO_STRIDE];
            uint8_t *rd_ptr = &line_buff[fifo_rptr * LINE_FIFO_STRIDE + 9];
            uint8_t *wr_ptr = frame_buff + (y * (240 * 3 / 2));
            for (int x = 0; x < 240; x+=2) {
                uint16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                uint16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                *(wr_ptr++) = (c0 >> 4) & 0xff;
                *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
                *(wr_ptr++) = c1 & 0xff;
            }
            line_fifo_rptr = (fifo_rptr + 1) % LINE_FIFO_DEPTH;
            
            if (y == nes::SCREEN_HEIGHT-1) {
                // fps measurement
                auto t_now = get_absolute_time();
                if (frame_count < 60-1) {
                    frame_count++;
                }
                else {
                    auto t_diff = absolute_time_diff_us(t_last_frame, t_now);
                    float fps = (60.0f * 1000000) / t_diff;
                    sprintf(fps_str, "%5.2ffps", fps);
                    t_last_frame = t_now;
                    frame_count = 0;
                }

                // DMA transfer
                ws19804::finish_write_data();
                draw_string(0, 0, fps_str);
                ws19804::start_write_data(25, 0, 240, 240, frame_buff);
                static int tmp = 0;
                gpio_put(PIN_MONITOR, tmp);
                tmp ^= 1;
            }
        }
    }
}

static void ppu_loop() {
    for(;;) {
        int wptr = line_fifo_wptr;
        bool eol = nes::ppu::service(&line_buff[wptr * LINE_FIFO_STRIDE + 1]);
        int y = nes::ppu::current_focus_y();
        if (eol && y < nes::SCREEN_HEIGHT) {
            line_buff[wptr * LINE_FIFO_STRIDE] = y;
            line_fifo_wptr = (wptr + 1) % LINE_FIFO_DEPTH;
        }
    }
}

static void apu_dma_handler() {
    speaker.flip_buffer();
    apu_fill_buffer(speaker.get_next_buffer());
}

static void apu_fill_buffer(PwmAudio::sample_t *buff) {
    nes::apu::service(spk_buff, speaker.LATENCY);
    for (int i = 0; i < speaker.LATENCY; i++) {
        buff[i] = spk_buff[i];
    }
}