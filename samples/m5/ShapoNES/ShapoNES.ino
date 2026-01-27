#include <M5Unified.h>
#include <driver/i2s_pdm.h>
#include "FS.h"
#include "SD.h"

#include "shapones_core.h"

#include "AdcButton.hpp"

#pragma GCC optimize("O2")

#define SHAPONES_USE_CANVAS (0)
#define SHAPONES_ENABLE_AUDIO (1)

#ifdef ARDUINO_M5STACK_ATOMS3
static constexpr int TF_CS_PIN = -1;
static constexpr int TF_SCK_PIN = 7;
static constexpr int TF_MISO_PIN = 8;
static constexpr int TF_MOSI_PIN = 6;
#else
static constexpr int TF_CS_PIN = 4;
#endif

#ifdef ARDUINO_M5STACK_ATOMS3
static constexpr int BUFF_W = nes::SCREEN_WIDTH / 2;
static constexpr int BUFF_H = nes::SCREEN_HEIGHT / 2;
constexpr int DMA_HEIGHT = 120;
uint32_t resize_buff[BUFF_W];
#else
static constexpr int BUFF_W = nes::SCREEN_WIDTH;
static constexpr int BUFF_H = nes::SCREEN_HEIGHT;
constexpr int DMA_HEIGHT = 60;
#endif

static const uint8_t BUTTON_ADC_PINS[] = {
  5,
  2,
  1,
};
static constexpr int BUTTON_NUM_PINS =
  sizeof(BUTTON_ADC_PINS) / sizeof(BUTTON_ADC_PINS[0]);

static constexpr int BUTTON_A = 0;
static constexpr int BUTTON_B = 1;
static constexpr int BUTTON_SELECT = 2;
static constexpr int BUTTON_START = 3;
static constexpr int BUTTON_UP = 4;
static constexpr int BUTTON_DOWN = 5;
static constexpr int BUTTON_LEFT = 6;
static constexpr int BUTTON_RIGHT = 7;
static constexpr int BUTTON_DUMMY = 8;

static constexpr int DISPLAY_BUTTON_PIN = 41;

static constexpr uint32_t AUDIO_PDM_FREQ_HZ = 96000;
static constexpr uint32_t AUDIO_SAMPLE_FREQ_HZ = AUDIO_PDM_FREQ_HZ / 6;
static constexpr int AUDIO_BUFF_SIZE = 256;
static constexpr int AUDIO_DOUT_PIN = 38;
static constexpr int AUDIO_CLK_PIN = 39;

static uint16_t frame_buff[BUFF_W * BUFF_H];
static int dma_next_y = nes::SCREEN_HEIGHT;

static uint8_t *ines = nullptr;
static uint8_t line_buff[nes::SCREEN_WIDTH];
static uint64_t next_vsync_us = 0;
static bool skip_frame = false;

static spinlock_t locks[nes::NUM_LOCKS];
static SemaphoreHandle_t sems[nes::NUM_SEMAPHORES];

static adc_button::Pin button_pins[BUTTON_NUM_PINS];

static nes::input::status_t input_state = { 0 };

#ifdef ARDUINO_M5STACK_ATOMS3
// clang-format off
static const uint32_t COLOR_TABLE[] = {
  0x0e1d0e, 0x040611, 0x000015, 0x080013, 0x11000e, 0x150002, 0x140000, 0x0f0200,
  0x080b00, 0x001100, 0x001400, 0x000f02, 0x030f0b, 0x000000, 0x000000, 0x000000,
  0x172f17, 0x001c1d, 0x040e1d, 0x10001e, 0x170017, 0x1c000b, 0x1b0a00, 0x191301,
  0x111c00, 0x002500, 0x002a00, 0x002407, 0x002011, 0x000000, 0x000000, 0x000000,
  0x1f3f1f, 0x072f1f, 0x0b1c1f, 0x14221f, 0x1e1e1f, 0x1f1d16, 0x1f1d0c, 0x1f2607,
  0x1e2f07, 0x103402, 0x093709, 0x0b3e13, 0x003a1b, 0x0e1d0e, 0x000000, 0x000000,
  0x1f3f1f, 0x15391f, 0x18351f, 0x1a321f, 0x1f311f, 0x1f311b, 0x1f2f16, 0x1f3615,
  0x1f3914, 0x1c3f14, 0x153c17, 0x163f19, 0x133f1e, 0x172f17, 0x000000, 0x000000,
};
// clang-format on
#else
// clang-format off
static const uint16_t COLOR_TABLE[] = {
  0xae73, 0xd120, 0x1500, 0x1340, 0x0e88, 0x02a8, 0x00a0, 0x4078, 0x6041, 0x2002, 0x8002, 0xe201, 0xeb19, 0x0000, 0x0000, 0x0000,
  0xf7bd, 0x9d03, 0xdd21, 0x1e80, 0x17b8, 0x0be0, 0x40d9, 0x61ca, 0x808b, 0xa004, 0x4005, 0x8704, 0x1104, 0x0000, 0x0000, 0x0000,
  0xffff, 0xff3d, 0x9f5b, 0x5fa4, 0xdff3, 0xb6fb, 0xacfb, 0xc7fc, 0xe7f5, 0x8286, 0xe94e, 0xd35f, 0x5b07, 0xae73, 0x0000, 0x0000,
  0xffff, 0x3faf, 0xbfc6, 0x5fd6, 0x3ffe, 0x3bfe, 0xf6fd, 0xd5fe, 0x34ff, 0xf4e7, 0x97af, 0xf9b7, 0xfe9f, 0xf7bd, 0x0000, 0x0000,
};
// clang-format on
#endif

static uint32_t fps_frame_count = 0;
static uint64_t fps_last_meas_ms = 0;
static float fps = 0.0f;

struct stat_t {
  uint64_t start_us = 0;
  uint32_t time_us = 0;
  uint32_t count = 0;
};

static stat_t stat_cpu_service;
static stat_t stat_ppu_service;
static stat_t stat_apu_service;
static uint32_t stat_ppu_delay_clock = 0;
static uint32_t stat_ppu_delay_count = 0;
static uint32_t stat_pdm_sent_bytes = 0;
static uint32_t stat_pdm_sent_count = 0;

static uint64_t disp_button_down_ms = 0;
static bool disp_button_pressed = false;

static SHAPONES_INLINE void stat_start(stat_t &s) {
  s.start_us = micros();
}

static SHAPONES_INLINE void stat_end(stat_t &s) {
  uint32_t elapsed_us = micros() - s.start_us;
  s.time_us += elapsed_us;
  s.count++;
}

static SHAPONES_INLINE uint32_t stat_get_average(stat_t &s) {
  uint32_t ave = 0;
  if (s.count > 0) {
    ave = s.time_us / s.count;
  }
  s.time_us = 0;
  s.count = 0;
  return ave;
}

static i2s_chan_handle_t audio_i2s_ch;
static uint8_t apu_out_buff[AUDIO_BUFF_SIZE];
static int16_t audio_buff[AUDIO_BUFF_SIZE];
static constexpr uint32_t SAMPLE_SIZE = sizeof(audio_buff[0]);
static volatile uint32_t audio_wr_ptr = 0;
static volatile uint32_t audio_rd_ptr = 0;

static File file_handle;

static void read_input();
static void ppu_loop(void *arg);
static bool wait_vsync();
static bool dma_busy();
static void dma_start();
static void dma_maintain();
static void meas();
static void audio_init();
static void speaker_fill_buff(bool preload);
static bool disp_button_down();

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  delay(500);

  Serial.printf("ESP-IDF Version: %s\n", esp_get_idf_version());

  pinMode(DISPLAY_BUTTON_PIN, INPUT_PULLUP);

#ifdef ARDUINO_M5STACK_ATOMS3
  SPI.begin(TF_SCK_PIN, TF_MISO_PIN, TF_MOSI_PIN, TF_CS_PIN);
#endif

  auto nes_cfg = nes::get_default_config();
  nes_cfg.apu_sampling_rate = AUDIO_SAMPLE_FREQ_HZ;
  nes::init(nes_cfg);

  xTaskCreatePinnedToCore(ppu_loop, "PPULoop", 8192, NULL, 10, NULL,
                          PRO_CPU_NUM);

  for (int i = 0; i < BUTTON_NUM_PINS; i++) {
    pinMode(BUTTON_ADC_PINS[i], INPUT);
  }
  analogContinuousSetWidth(12);
  if (!analogContinuous(BUTTON_ADC_PINS, BUTTON_NUM_PINS, 3, 1000, nullptr)) {
    Serial.println("adc init failed");
  }
  analogContinuousStart();

#if SHAPONES_ENABLE_AUDIO
  audio_init();
#endif

  nes::menu::show();
}

void loop() {
  read_input();

  if (disp_button_down()) {
    if (nes::menu::is_shown()) {
      nes::menu::hide();
    } else {
      nes::menu::show();
    }
  }

  stat_start(stat_cpu_service);
  for (int i = 0; i < 100; i++) {
    nes::cpu::service();
  }
  stat_end(stat_cpu_service);

#if SHAPONES_ENABLE_AUDIO
  speaker_fill_buff(false);
#endif
}

static void read_input() {
  adc_continuous_data_t *data = nullptr;
  if (!analogContinuousRead(&data, 0)) return;
  for (int i = 0; i < BUTTON_NUM_PINS; i++) {
    adc_button::Pin &pin = button_pins[i];
    pin.update_value(data[i].avg_read_raw);
    uint32_t code = pin.read_current();
    for (int j = 0; j < adc_button::NUM_BUTTONS; j++) {
      int button_index = i * adc_button::NUM_BUTTONS + j;
      uint8_t pressed = code & 1;
      switch (button_index) {
        case BUTTON_LEFT: input_state.left = pressed; break;
        case BUTTON_RIGHT: input_state.right = pressed; break;
        case BUTTON_DOWN: input_state.down = pressed; break;
        case BUTTON_UP: input_state.up = pressed; break;
        case BUTTON_START: input_state.start = pressed; break;
        case BUTTON_SELECT: input_state.select = pressed; break;
        case BUTTON_A: input_state.A = pressed; break;
        case BUTTON_B: input_state.B = pressed; break;
      }
      code >>= 1;
    }
  }
  nes::input::set_status(0, input_state);
}

static void ppu_loop(void *arg) {
  uint64_t next_wdt_reset_ms = 0;
  while (true) {
    stat_ppu_delay_clock +=
      nes::cpu::ppu_cycle_leading() - nes::ppu::cycle_following();
    stat_ppu_delay_count += 1;
    nes::ppu::status_t status;
    stat_start(stat_ppu_service);
    nes::ppu::service(line_buff, skip_frame, &status);
    if (!skip_frame) {
      stat_end(stat_ppu_service);
    }
    if ((!!(status.timing & nes::ppu::timing_t::END_OF_VISIBLE_LINE)) && !skip_frame) {
#ifdef ARDUINO_M5STACK_ATOMS3
      if (status.focus_y % 2 == 0) {
        for (int x = 0; x < BUFF_W; x++) {
          uint32_t c0 = COLOR_TABLE[line_buff[x * 2 + 0] & 0x3f];
          uint32_t c1 = COLOR_TABLE[line_buff[x * 2 + 1] & 0x3f];
          resize_buff[x] = c0 + c1;
        }
      } else {
        uint16_t *wptr = frame_buff + status.focus_y / 2 * BUFF_W;
        for (int x = 0; x < BUFF_W; x++) {
          uint32_t c01 = resize_buff[x];
          uint32_t c2 = COLOR_TABLE[line_buff[x * 2 + 0] & 0x3f];
          uint32_t c3 = COLOR_TABLE[line_buff[x * 2 + 1] & 0x3f];
          uint32_t c = c01 + c2 + c3 + 0x020202;
          c = ((c >> 7) & 0xF100) | ((c >> 5) & 0x07E0) | ((c >> 2) & 0x001F);
          wptr[x] = ((c << 8) & 0xFF00) | ((c >> 8) & 0x00FF);
        }
      }
#else
      uint16_t *wptr = frame_buff + y * BUFF_W;
      for (int x = 0; x < BUFF_W; x++) {
        wptr[x] = COLOR_TABLE[line_buff[x] & 0x3f];
      }
#endif
    }

    if (!!(status.timing & nes::ppu::timing_t::END_OF_VISIBLE_AREA)) {
      if (!skip_frame) {
        dma_start();
        meas();
      }
      skip_frame = wait_vsync();
    } else {
      dma_maintain();
    }

    uint64_t now_ms = millis();
    if (now_ms > next_wdt_reset_ms) {
      next_wdt_reset_ms = now_ms + 1000;
      vTaskDelay(1);
    }
  }
}

static bool wait_vsync() {
  constexpr int FRAME_DELAY_US = 1000000 / 60;
  uint64_t now_us = micros();
  int64_t wait_us = next_vsync_us - now_us;
  if (wait_us > 0) {
    delayMicroseconds(wait_us);
  }
  next_vsync_us += FRAME_DELAY_US;
  if (now_us > next_vsync_us) {
    next_vsync_us = now_us;
  }
  return wait_us <= -5000;
}

static bool dma_busy() {
  return (dma_next_y < BUFF_H) || M5.Display.dmaBusy();
}

static void dma_start() {
  if (dma_busy()) return;

  dma_next_y = 0;
  dma_maintain();
}

static void dma_maintain() {
  if (dma_next_y >= BUFF_H) return;
  if (M5.Display.dmaBusy()) return;

  int dx = (M5.Display.width() - BUFF_W) / 2;
  int dy = (M5.Display.height() - BUFF_H) / 2 + dma_next_y;

  int h = DMA_HEIGHT;
  if (dma_next_y + h > BUFF_H) {
    h = BUFF_H - dma_next_y;
  }
  uint16_t *sptr = frame_buff + dma_next_y * BUFF_W;
  M5.Display.endWrite();
  M5.Display.startWrite();
  M5.Display.pushImageDMA(dx, dy, BUFF_W, h, sptr);
  dma_next_y += h;
}

static void meas() {
  uint64_t now_ms = millis();
  uint32_t elapsed_ms = now_ms - fps_last_meas_ms;

  uint32_t cpu_time_us = stat_get_average(stat_cpu_service);
  uint32_t ppu_time_us = stat_get_average(stat_ppu_service);
  uint32_t apu_time_us = stat_get_average(stat_apu_service);
  float ppu_delay_clocks = (float)stat_ppu_delay_clock / stat_ppu_delay_count;
  stat_ppu_delay_clock = 0;
  stat_ppu_delay_count = 0;
  float pdm_sent_bytes = (float)stat_pdm_sent_bytes / stat_pdm_sent_count;
  stat_pdm_sent_bytes = 0;
  stat_pdm_sent_count = 0;

  fps_frame_count++;
  if (elapsed_ms >= 1000) {
    fps = (float)(fps_frame_count * 1000) / elapsed_ms;
#if 1
    Serial.printf(
      "%5.2f FPS (CPU:%u us, PPU:%u us, APU:%u us, PPU delay:%6.2f cyc, PDM "
      "sent:%7.1f)\n",
      fps, cpu_time_us, ppu_time_us, apu_time_us, ppu_delay_clocks,
      pdm_sent_bytes);
#endif
    fps_last_meas_ms = now_ms;
    fps_frame_count = 0;
  }
}

static void audio_init() {
  i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_i2s_ch, nullptr));

  i2s_pdm_tx_clk_config_t clk_cfg =
    I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG(AUDIO_SAMPLE_FREQ_HZ);

  i2s_pdm_tx_slot_config_t slot_cfg = I2S_PDM_TX_SLOT_DAC_DEFAULT_CONFIG(
    I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);

  i2s_pdm_tx_config_t tx_cfg{
    .clk_cfg = clk_cfg,
    .slot_cfg = slot_cfg,
    .gpio_cfg = {
      .clk = I2S_GPIO_UNUSED,  //(gpio_num_t)AUDIO_CLK_PIN,
      .dout = (gpio_num_t)AUDIO_DOUT_PIN,
      .dout2 = I2S_GPIO_UNUSED,
      .invert_flags = {
        .clk_inv = false,
      },
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(audio_i2s_ch, &tx_cfg));

  speaker_fill_buff(true);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_i2s_ch));
}

static void speaker_fill_buff(bool preload) {
  uint32_t wr_ptr = audio_wr_ptr;
  uint32_t rd_ptr = audio_rd_ptr;

  uint32_t buff_free = (rd_ptr - wr_ptr) & (AUDIO_BUFF_SIZE - 1);
  if (buff_free == 0) {
    buff_free = AUDIO_BUFF_SIZE;
  }
  if (buff_free > 1) {
    buff_free--;
    stat_start(stat_apu_service);
    nes::apu::service(apu_out_buff, buff_free);
    for (int i = 0; i < buff_free; i++) {
      int16_t val = ((int16_t)apu_out_buff[i] - 128) * 256;
      audio_buff[wr_ptr] = val /* - audio_bias*/;
      wr_ptr = (wr_ptr + 1) & (AUDIO_BUFF_SIZE - 1);
    }
    stat_end(stat_apu_service);
  }

  if (wr_ptr < rd_ptr) {
    size_t to_write = (AUDIO_BUFF_SIZE - rd_ptr) * SAMPLE_SIZE;
    size_t written = 0;
    if (preload) {
      i2s_channel_preload_data(audio_i2s_ch, &audio_buff[rd_ptr], to_write,
                               &written);
    } else {
      i2s_channel_write(audio_i2s_ch, &audio_buff[rd_ptr], to_write, &written,
                        0);
    }
    rd_ptr = (rd_ptr + written / SAMPLE_SIZE) & (AUDIO_BUFF_SIZE - 1);
    stat_pdm_sent_bytes += written;
    stat_pdm_sent_count += 1;
  }
  if (rd_ptr < wr_ptr) {
    size_t to_write = (wr_ptr - rd_ptr) * SAMPLE_SIZE;
    size_t written = 0;
    if (preload) {
      i2s_channel_preload_data(audio_i2s_ch, &audio_buff[rd_ptr], to_write,
                               &written);
    } else {
      i2s_channel_write(audio_i2s_ch, &audio_buff[rd_ptr], to_write, &written,
                        0);
    }
    rd_ptr = (rd_ptr + written / SAMPLE_SIZE) & (AUDIO_BUFF_SIZE - 1);
    stat_pdm_sent_bytes += written;
    stat_pdm_sent_count += 1;
  }

  audio_wr_ptr = wr_ptr;
  audio_rd_ptr = rd_ptr;
}

static bool disp_button_down() {
  uint64_t now_ms = millis();
  bool ret = false;
  if (digitalRead(DISPLAY_BUTTON_PIN) == LOW) {
    if (!disp_button_pressed && now_ms > disp_button_down_ms) {
      disp_button_pressed = true;
      ret = true;
    }
  } else {
    disp_button_down_ms = now_ms + 100;
    disp_button_pressed = false;
  }
  return ret;
}

nes::result_t nes::lock_init(int id) {
  spinlock_initialize(&locks[id]);
  return nes::result_t::SUCCESS;
}
void nes::lock_deinit(int id) {}
void nes::lock_get(int id) {
  taskENTER_CRITICAL(&locks[id]);
}
void nes::lock_release(int id) {
  taskEXIT_CRITICAL(&locks[id]);
}

nes::result_t nes::sem_init(int id) {
  sems[id] = xSemaphoreCreateBinary();
  xSemaphoreGive(sems[id]);
  return nes::result_t::SUCCESS;
}
void nes::sem_deinit(int id) {}
bool nes::sem_try_take(int id) {
  return (xSemaphoreTake(sems[id], 0) == pdTRUE);
}
void nes::sem_take(int id) {
  while (!nes::sem_try_take(id)) {
    vTaskDelay(1);
  }
}
void nes::sem_give(int id) {
  xSemaphoreGive(sems[id]);
}

nes::result_t nes::fs_mount() {
  if (SD.begin(TF_CS_PIN, SPI, 10000000)) {
    return nes::result_t::SUCCESS;
  } else {
    Serial.printf("No Disk.\n");
    vTaskDelay(1000);
    return nes::result_t::ERR_NO_DISK;
  }
}

void nes::fs_unmount() {
  SD.end();
}

nes::result_t nes::fs_get_current_dir(char *out_path) {
  strncpy(out_path, "/", nes::MAX_PATH_LENGTH);
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_enum_files(const char *path,
                                 nes::fs_enum_files_cb_t callback) {
  bool is_dir;
  File root = SD.open(path);
  while (1) {
    nes::file_info_t fi;
    String filename = root.getNextFileName(&fi.is_dir);
    int sep = filename.lastIndexOf('/');
    if (sep >= 0) {
      filename = filename.substring(sep + 1);
    }
    if (filename == "") break;
    fi.name = (char *)filename.c_str();
    if (!callback(fi)) break;
  }
  root.close();
  return nes::result_t::SUCCESS;
}

bool nes::fs_exists(const char *path) {
  return SD.exists(path);
}

nes::result_t nes::fs_open(const char *path, bool write, void **handle) {
  file_handle = SD.open(path, write ? FILE_WRITE : FILE_READ);
  if (!file_handle) {
    Serial.printf("Open failed: %s\n", path);
    return nes::result_t::ERR_FAILED_TO_OPEN_FILE;
  }
  *handle = &file_handle;
  return nes::result_t::SUCCESS;
}

void nes::fs_close(void *handle) {
  File *f = (File *)handle;
  f->close();
}

nes::result_t nes::fs_seek(void *handle, size_t offset) {
  File *f = (File *)handle;
  if (f->seek(offset)) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FAILED_TO_SEEK_FILE;
  }
}

nes::result_t nes::fs_size(void *handle, size_t *out_size) {
  File *f = (File *)handle;
  *out_size = f->size();
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_read(void *handle, uint8_t *buff, size_t size) {
  File *f = (File *)handle;
  size_t s = f->read(buff, size);
  if (s == size) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FAILED_TO_READ_FILE;
  }
}

nes::result_t nes::fs_write(void *handle, const uint8_t *buff, size_t size) {
  File *f = (File *)handle;
  size_t s = f->write(buff, size);
  if (s == size) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FAILED_TO_WRITE_FILE;
  }
}

nes::result_t nes::fs_delete(const char *path) {
  if (SD.remove(path)) {
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FAILED_TO_DELETE_FILE;
  }
}

nes::result_t nes::load_ines(const char *path, const uint8_t **out_ines,
                             size_t *out_size) {
  nes::unload_ines();
  File f = SD.open(path, FILE_READ);
  if (!f) {
    return nes::result_t::ERR_FAILED_TO_OPEN_FILE;
  }
  size_t file_size = f.size();
  ines = new uint8_t[file_size];
  size_t s = f.read(ines, file_size);
  f.close();
  if (s == file_size) {
    *out_ines = ines;
    *out_size = file_size;
    return nes::result_t::SUCCESS;
  } else {
    return nes::result_t::ERR_FAILED_TO_READ_FILE;
  }
}

void nes::unload_ines() {
  if (ines != nullptr) {
    delete[] ines;
    ines = nullptr;
  }
}
