#include "../include.h"

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
static bool ines_allocated = false;

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

static constexpr int SPINLOCK_ID_BASE = 0;
static constexpr int SEMAPHORE_ID_BASE = SPINLOCK_ID_BASE + nes::NUM_SPINLOCKS;

nes::input::status_t input_status;

sFile file_handle;

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
#if USE_PICOPAD
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  WaitMs(100);
  set_sys_clock_khz(250000, true);
#endif

  core0_main();

  return 0;
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
  nes::menu::show();

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

    bool menu_pressed_last = input_status.select && input_status.down;

    input_status.raw = 0;
    if (KeyPressedFast(KEY_LEFT)) input_status.left = 1;
    if (KeyPressedFast(KEY_RIGHT)) input_status.right = 1;
    if (KeyPressedFast(KEY_UP)) input_status.up = 1;
    if (KeyPressedFast(KEY_DOWN)) input_status.down = 1;
    if (KeyPressedFast(KEY_A)) input_status.A = 1;
    if (KeyPressedFast(KEY_B)) input_status.start = 1;
    if (KeyPressedFast(KEY_X)) input_status.B = 1;
    if (KeyPressedFast(KEY_Y)) input_status.select = 1;

    bool menu_pressed_now = input_status.select && input_status.down;

    if (menu_pressed_now && !menu_pressed_last) {
      if (nes::menu::is_shown()) {
        nes::menu::hide();
      } else {
        nes::menu::show();
      }
    }

    nes::input::set_status(0, input_status);

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

  LockoutInit(1);

  while (true) {
    nes::ppu::status_t s;
    nes::ppu::service(line_buff, skip_frame, &s);

    // end of visible line
    if (!!((s.timing & nes::ppu::timing_t::END_OF_VISIBLE_LINE)) &&
        !skip_frame) {
      // Convert palette index to RGB444
      uint8_t *rd_ptr = line_buff;
      uint8_t *wr_ptr = frame_buff + (s.focus_y * FRAME_BUFF_STRIDE);
      for (int x = 0; x < nes::SCREEN_WIDTH; x += 2) {
        uint16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
        uint16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
        *(wr_ptr++) = (c0 >> 4) & 0xff;
        *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
        *(wr_ptr++) = c1 & 0xff;
      }
    }
    if (!!(s.timing & nes::ppu::timing_t::END_OF_VISIBLE_AREA)) {
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
  if (wait_us > 0) {
    WaitUs(wait_us);
  }
  disp_next_vsync_us += FRAME_INTERVAL_US;
  if (now_us > disp_next_vsync_us) {
    disp_next_vsync_us = now_us;
  }
  return wait_us <= -5000;
}

static const uint8_t *sound_refill() {
  uint8_t *ptr = sound_buff + sound_buff_index * SOUND_BUFF_SIZE;
  nes::apu::service(ptr, SOUND_BUFF_SIZE);
  sound_buff_index = (sound_buff_index + 1) & 1;
  return ptr;
}

nes::result_t nes::ram_alloc(size_t size, void **out_ptr) {
  void *ptr = malloc(size);
  if (!ptr) {
    return nes::result_t::ERR_RAM_ALLOC_FAILED;
  }
  *out_ptr = ptr;
  return nes::result_t::SUCCESS;
}

void nes::ram_free(void *ptr) { free(ptr); }

nes::result_t nes::spinlock_init(int id) {
  SpinClaim(SPINLOCK_ID_BASE + id);
  return nes::result_t::SUCCESS;
}
void nes::spinlock_deinit(int id) { SpinUnclaim(SPINLOCK_ID_BASE + id); }
void nes::spinlock_get(int id) { SpinLock(SPINLOCK_ID_BASE + id); }
void nes::spinlock_release(int id) { SpinUnlock(SPINLOCK_ID_BASE + id); }

nes::result_t nes::semaphore_init(int id) {
  SpinClaim(SEMAPHORE_ID_BASE + id);
  return nes::result_t::SUCCESS;
}
void nes::semaphore_deinit(int id) { SpinUnclaim(SEMAPHORE_ID_BASE + id); }
void nes::semaphore_take(int id) { SpinLock(SEMAPHORE_ID_BASE + id); }
bool nes::semaphore_try_take(int id) {
  return SpinTryLock(SEMAPHORE_ID_BASE + id);
}
void nes::semaphore_give(int id) { SpinUnlock(SEMAPHORE_ID_BASE + id); }

nes::result_t nes::fs_mount() {
  if (DiskMount()) {
    return nes::result_t::SUCCESS;
  } else {
    WaitMs(500);
    return nes::result_t::ERR_FS_NO_DISK;
  }
}

void nes::fs_unmount() { DiskUnmount(); }

nes::result_t nes::fs_get_current_dir(char *out_path) {
  strncpy(out_path, "/EMU/SHAPONES/", nes::MAX_PATH_LENGTH);
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_enum_files(const char *path,
                                 nes::fs_enum_files_cb_t callback) {
  // List up NES files
  sFile find;
  if (!FindOpen(&find, path)) {
    return nes::result_t::ERR_FS_DIR_NOT_FOUND;
  }
  sFileInfo fi;
  while (FindNext(&find, &fi, ATTR_ARCH | ATTR_DIR, "*")) {
    if (strncmp(fi.name, ".", 1) == 0 || strncmp(fi.name, "..", 2) == 0) {
      continue;
    }
    nes::file_info_t fi2;
    fi2.name = fi.name;
    fi2.is_dir = !!(fi.attr & ATTR_DIR);
    if (!callback(fi2)) break;
  }
  FindClose(&find);
  return nes::result_t::SUCCESS;
}

bool nes::fs_exists(char const *path) { return FileExist(path); }

nes::result_t nes::fs_open(const char *path, bool write, void **handle) {
  if (!FileExist(path)) {
    if (!FileCreate(&file_handle, path)) {
      return nes::result_t::ERR_FS_OPEN_FAILED;
    }
  } else {
    if (!FileOpen(&file_handle, path)) {
      return nes::result_t::ERR_FS_OPEN_FAILED;
    }
  }
  *handle = &file_handle;
  return nes::result_t::SUCCESS;
}

void nes::fs_close(void *handle) {
  sFile *f = (sFile *)handle;
  FileClose(f);
}

nes::result_t nes::fs_seek(void *handle, size_t offset) {
  sFile *f = (sFile *)handle;
  if (!FileSeek(f, offset)) {
    return nes::result_t::ERR_FS_SEEK_FAILED;
  }
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_size(void *handle, size_t *out_size) {
  sFile *f = (sFile *)handle;
  *out_size = FileSize(f);
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_read(void *handle, uint8_t *buff, size_t size) {
  sFile *f = (sFile *)handle;
  int s = FileRead(f, buff, size);
  if (s == (int)size) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FS_READ_FAILED;
  }
}

nes::result_t nes::fs_write(void *handle, const uint8_t *buff, size_t size) {
  sFile *f = (sFile *)handle;
  int s = FileWrite(f, buff, size);
  if (s == (int)size) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FS_WRITE_FAILED;
  }
}

nes::result_t nes::fs_delete(const char *path) {
  if (FileDelete(path)) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FS_DELETE_FAILED;
  }
}

nes::result_t nes::load_ines(const char *path, const uint8_t **out_ines,
                             size_t *out_size) {
  nes::result_t res = nes::result_t::SUCCESS;

  nes::unload_ines();

  // Load NES file
  sFile ines_file;
  if (!FileOpen(&ines_file, path)) {
    return nes::result_t::ERR_FS_OPEN_FAILED;
  }
  int ines_size = ines_file.size;
  if (ines_size < (128 + 1) * 1024) {
    // Load to RAM
    ines_rom = new uint8_t[ines_size];
    ines_allocated = true;
    int ret = FileRead(&ines_file, ines_rom, ines_size);
    if (ret != ines_size) {
      res = nes::result_t::ERR_FS_READ_FAILED;
    }
  } else if (ines_size < (1024 + 1) * 1024) {
    // Load to Flash
    constexpr int CHUNK_SIZE = 4096;

    uint8_t *buff = new uint8_t[CHUNK_SIZE];

    LockoutStart();
    FlashErase(ROM_OFFSET, ines_size);
    int remaining = ines_size;
    while (remaining > 0) {
      int to_read = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
      int ret = FileRead(&ines_file, buff, to_read);
      if (ret <= 0) {
        res = nes::result_t::ERR_FS_READ_FAILED;
        break;
      }
      FlashProgram(ROM_OFFSET + (ines_size - remaining), buff, ret);
      remaining -= ret;
    }
    LockoutStop();
    delete[] buff;
    if (res != nes::result_t::SUCCESS) {
      return nes::result_t::ERR_FS_READ_FAILED;
    }
    ines_rom = (uint8_t *)ROM_OFFSET;
    ines_allocated = false;
  } else {
    res = nes::result_t::ERR_INES_TOO_LARGE;
  }
  FileClose(&ines_file);

  if (res != nes::result_t::SUCCESS) {
    return res;
  }

  *out_ines = ines_rom;
  *out_size = ines_size;

  return nes::result_t::SUCCESS;
}

void nes::unload_ines() {
  if (ines_allocated) {
    delete[] ines_rom;
  }
  ines_rom = nullptr;
  ines_allocated = false;
}