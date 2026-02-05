#pragma GCC optimize("O2")

#include <M5Unified.h>
#include <driver/i2s.h>

#include "shapones_config.hpp"
#include "shapones_core.h"
#include "shapones_host_impl.hpp"

#include "AdcButton.hpp"

#define EXPERIMENTAL_ENABLE_REMOTE (0)

#if EXPERIMENTAL_ENABLE_REMOTE
#include <AsyncUDP.h>
#include <WiFi.h>
#endif

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

#if SHAPONES_ENABLE_PDM_AUDIO || SHAPONES_ENABLE_I2S_AUDIO
static constexpr uint32_t AUDIO_I2S_FREQ_HZ = 96000;
static constexpr uint32_t AUDIO_SAMPLE_FREQ_HZ = AUDIO_I2S_FREQ_HZ / 6;
static constexpr int AUDIO_BUFF_LEN = 256;
#endif

#if SHAPONES_ENABLE_HALF_SCREEN
uint32_t resize_buff[BUFF_W];
#endif

static uint16_t frame_buff[BUFF_W * BUFF_H];
static int dma_next_y = nes::SCREEN_HEIGHT;

static uint8_t line_buff[nes::SCREEN_WIDTH];
static uint64_t next_vsync_us = 0;
static bool skip_frame = false;

static adc_button::Pin button_pins[BUTTON_NUM_PINS];

static nes::input::status_t input_state = { 0 };

#if SHAPONES_ENABLE_HALF_SCREEN
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

static uint64_t disp_button_down_ms = 0;
static bool disp_button_pressed = false;

static i2s_chan_handle_t audio_i2s_ch;
static uint8_t apu_out_buff[AUDIO_BUFF_LEN] = { 0 };
static int16_t audio_buff[AUDIO_BUFF_LEN] = { 0 };
static constexpr uint32_t SAMPLE_SIZE = sizeof(audio_buff[0]);
static uint32_t audio_wr_ptr = 0;
static uint32_t audio_rd_ptr = 0;

#if EXPERIMENTAL_ENABLE_REMOTE
static const IPAddress subnet(255, 255, 255, 0);
static const IPAddress local_ip(192, 168, 1, 100);
static const uint16_t udp_port = 12345;
static const char *ssid = "ShapoNES_AP";
static const char *password = "hogepiyo";
static AsyncUDP udp;
static constexpr uint32_t COMP_QUEUE_STRIDE = nes::SCREEN_WIDTH + 1;
static constexpr uint32_t COMP_QUEUE_DEPTH = 16;
static uint8_t comp_queue[COMP_QUEUE_STRIDE * COMP_QUEUE_DEPTH];
static volatile uint32_t comp_wr_ptr = 0;
static volatile uint32_t comp_rd_ptr = 0;
static uint8_t udp_buff[2048];
static uint32_t udp_len = 0;

static void push_comp_queue(int y) {
  uint32_t wp = comp_wr_ptr;
  uint32_t wp_next = (wp + 1) % COMP_QUEUE_DEPTH;
  while (wp_next == comp_rd_ptr) {
  }
  uint32_t offset = wp * COMP_QUEUE_STRIDE;
  comp_queue[offset] = y;
  memcpy(&comp_queue[offset + 1], line_buff, nes::SCREEN_WIDTH);
  SHAPONES_THREAD_FENCE_SEQ_CST();
  comp_wr_ptr = wp_next;
}

static void pop_comp_queue() {
  uint32_t rp = comp_rd_ptr;
  if (comp_wr_ptr == rp) return;
  uint32_t offset = rp * COMP_QUEUE_STRIDE;
  uint32_t y = comp_queue[offset];
  udp_buff[udp_len++] = 0xFF;
  udp_buff[udp_len++] = y;
  uint8_t *line = comp_queue + offset + 1;
  uint8_t b_last = line[0];
  uint32_t run_length = 1;
  for (uint32_t x = 1; x < nes::SCREEN_WIDTH; x++) {
    uint8_t b = line[x];
    if (b == 0x3F) b = 0x3E;
    if (b == b_last && run_length < 0x3F) {
      run_length++;
    } else {
      if (run_length < 4) {
        udp_buff[udp_len++] = b_last | ((run_length - 1) << 6);
      } else {
        udp_buff[udp_len++] = b_last | 0xC0;
        udp_buff[udp_len++] = run_length - 1;
      }
      b_last = b;
      run_length = 1;
    }
  }
  SHAPONES_THREAD_FENCE_SEQ_CST();
  comp_rd_ptr = (rp + 1) % COMP_QUEUE_DEPTH;
  SHAPONES_THREAD_FENCE_SEQ_CST();
  if (run_length < 4) {
    udp_buff[udp_len++] = b_last | ((run_length - 1) << 6);
  } else {
    udp_buff[udp_len++] = b_last | 0xC0;
    udp_buff[udp_len++] = run_length - 1;
  }
  constexpr uint32_t PAYLOAD_SIZE = 1436;
  if (udp_len >= PAYLOAD_SIZE || y == nes::SCREEN_HEIGHT - 1) {
    uint32_t to_send = (udp_len < PAYLOAD_SIZE) ? udp_len : PAYLOAD_SIZE;
    if (millis() > 5000) {
      udp.broadcastTo(udp_buff, to_send, udp_port);
    }
    memcpy(udp_buff, udp_buff + to_send, udp_len - to_send);
    udp_len -= to_send;
  }
}
#endif

static void input_init();
static void read_input();
static void ppu_loop(void *arg);
static bool wait_vsync();
static bool dma_busy();
static void dma_start();
static void dma_maintain();
static void audio_init();
static void audio_stream(bool preload);
static int audio_fill_buffer(int16_t *buff, int offset, int size);
static bool disp_button_down();

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  delay(500);

#if defined(ARDUINO_M5STACK_STICKS3)
  // to supress power noise
  M5.Power.setExtOutput(false);
#endif

  SHAPONES_PRINTF("ESP-IDF Version: %s\n", esp_get_idf_version());

  init_host_impl();

  pinMode(DISPLAY_BUTTON_PIN, INPUT_PULLUP);

#if EXPERIMENTAL_ENABLE_REMOTE
  WiFi.softAPConfig(local_ip, local_ip, subnet);
  WiFi.softAP(ssid, password);
#endif

#if SHAPONES_ENABLE_HALF_SCREEN
  SPI.begin(TF_SCK_PIN, TF_MISO_PIN, TF_MOSI_PIN, TF_CS_PIN);
#endif

  auto nes_cfg = nes::get_default_config();
  nes_cfg.apu_sampling_rate = AUDIO_SAMPLE_FREQ_HZ;
  nes::init(nes_cfg);

  xTaskCreatePinnedToCore(ppu_loop, "PPULoop", 8192, NULL, 10, NULL,
                          PRO_CPU_NUM);

  input_init();
  audio_init();

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

  for (int i = 0; i < 10; i++) {
    nes::cpu::service();
  }
#if EXPERIMENTAL_ENABLE_REMOTE
  pop_comp_queue();
#endif

  audio_stream(false);
}

static void input_init() {
  analogContinuousStop();
  for (int i = 0; i < BUTTON_NUM_PINS; i++) {
    pinMode(BUTTON_ADC_PINS[i], INPUT);
  }
  analogSetAttenuation(ADC_11db);
  analogContinuousSetWidth(12);
  if (!analogContinuous(BUTTON_ADC_PINS, BUTTON_NUM_PINS, 3, 1000, nullptr)) {
    Serial.println("adc init failed");
  }
  analogContinuousStart();
}

static void read_input() {
  adc_continuous_data_t *data = nullptr;
  if (!analogContinuousRead(&data, 0)) return;
  for (int i = 0; i < BUTTON_NUM_PINS; i++) {
    adc_button::Pin &pin = button_pins[i];
    pin.update_value(data[i].avg_read_mvolts);
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
    nes::ppu::status_t status;
    nes::ppu::service(line_buff, skip_frame, &status);
    if ((!!(status.timing & nes::ppu::timing_t::END_OF_VISIBLE_LINE)) && !skip_frame) {
#if EXPERIMENTAL_ENABLE_REMOTE
      push_comp_queue(status.focus_y);
#endif
#if SHAPONES_ENABLE_HALF_SCREEN
      if (status.focus_y % 2 == 0) {
        for (int x = 0; x < BUFF_W; x++) {
          uint32_t c0 = COLOR_TABLE[line_buff[x * 2 + 0]];
          uint32_t c1 = COLOR_TABLE[line_buff[x * 2 + 1]];
          resize_buff[x] = c0 + c1;
        }
      } else {
        uint16_t *wptr = frame_buff + status.focus_y / 2 * BUFF_W;
        for (int x = 0; x < BUFF_W; x++) {
          uint32_t c01 = resize_buff[x];
          uint32_t c2 = COLOR_TABLE[line_buff[x * 2 + 0]];
          uint32_t c3 = COLOR_TABLE[line_buff[x * 2 + 1]];
          uint32_t c = c01 + c2 + c3 + 0x020202;
          c = ((c >> 7) & 0xF100) | ((c >> 5) & 0x07E0) | ((c >> 2) & 0x001F);
          wptr[x] = ((c << 8) & 0xFF00) | ((c >> 8) & 0x00FF);
        }
      }
#else
      uint16_t *wptr = frame_buff + y * BUFF_W;
      for (int x = 0; x < BUFF_W; x++) {
        wptr[x] = COLOR_TABLE[line_buff[x]];
      }
#endif
    }

    if (!!(status.timing & nes::ppu::timing_t::END_OF_VISIBLE_AREA)) {
      if (!skip_frame) {
        dma_start();
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

static void audio_init() {
#if SHAPONES_ENABLE_I2S_AUDIO
  M5.Speaker.begin();
#endif

#if SHAPONES_ENABLE_PDM_AUDIO || SHAPONES_ENABLE_I2S_AUDIO
  i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_i2s_ch, nullptr));
#endif

#if SHAPONES_ENABLE_I2S_AUDIO
  i2s_std_clk_config_t clk_cfg =
    I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_FREQ_HZ);
  i2s_std_slot_config_t slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG(
    I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  i2s_std_config_t std_cfg{
    .clk_cfg = clk_cfg,
    .slot_cfg = slot_cfg,
    .gpio_cfg = {
      .mclk = (gpio_num_t)AUDIO_MCLK_PIN,
      .bclk = (gpio_num_t)AUDIO_BCLK_PIN,
      .ws = (gpio_num_t)AUDIO_LRCK_PIN,
      .dout = (gpio_num_t)AUDIO_DOUT_PIN,
      .din = (gpio_num_t)AUDIO_DIN_PIN,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_i2s_ch, &std_cfg));
#endif

#if SHAPONES_ENABLE_PDM_AUDIO
  i2s_pdm_tx_clk_config_t clk_cfg =
    I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG(AUDIO_SAMPLE_FREQ_HZ);
  i2s_pdm_tx_slot_config_t slot_cfg = I2S_PDM_TX_SLOT_DAC_DEFAULT_CONFIG(
    I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  i2s_pdm_tx_config_t tx_cfg{
    .clk_cfg = clk_cfg,
    .slot_cfg = slot_cfg,
    .gpio_cfg = {
      .clk = I2S_GPIO_UNUSED,
      .dout = (gpio_num_t)AUDIO_DOUT_PIN,
      .dout2 = I2S_GPIO_UNUSED,
      .invert_flags = {
        .clk_inv = false,
      },
    },
  };
  ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(audio_i2s_ch, &tx_cfg));
#endif

#if SHAPONES_ENABLE_PDM_AUDIO || SHAPONES_ENABLE_I2S_AUDIO
  audio_stream(true);
  ESP_ERROR_CHECK(i2s_channel_enable(audio_i2s_ch));
#endif
}

static void audio_stream(bool preload) {
#if SHAPONES_ENABLE_PDM_AUDIO || SHAPONES_ENABLE_I2S_AUDIO
  uint32_t buff_free = (audio_rd_ptr - audio_wr_ptr) & (AUDIO_BUFF_LEN - 1);
  if (buff_free == 0) {
    buff_free = AUDIO_BUFF_LEN;
  }
  if (buff_free > 1) {
    buff_free--;
    audio_wr_ptr = audio_fill_buffer(audio_buff, audio_wr_ptr, buff_free);
  }

  if (audio_wr_ptr < audio_rd_ptr) {
    size_t to_write = (AUDIO_BUFF_LEN - audio_rd_ptr) * SAMPLE_SIZE;
    size_t written = 0;
    if (preload) {
      i2s_channel_preload_data(audio_i2s_ch, &audio_buff[audio_rd_ptr],
                               to_write, &written);
    } else {
      i2s_channel_write(audio_i2s_ch, &audio_buff[audio_rd_ptr], to_write,
                        &written, 0);
    }
    audio_rd_ptr =
      (audio_rd_ptr + written / SAMPLE_SIZE) & (AUDIO_BUFF_LEN - 1);
  }
  if (audio_rd_ptr < audio_wr_ptr) {
    size_t to_write = (audio_wr_ptr - audio_rd_ptr) * SAMPLE_SIZE;
    size_t written = 0;
    if (preload) {
      i2s_channel_preload_data(audio_i2s_ch, &audio_buff[audio_rd_ptr],
                               to_write, &written);
    } else {
      i2s_channel_write(audio_i2s_ch, &audio_buff[audio_rd_ptr], to_write,
                        &written, 0);
    }
    audio_rd_ptr =
      (audio_rd_ptr + written / SAMPLE_SIZE) & (AUDIO_BUFF_LEN - 1);
  }
#endif
}

static int audio_fill_buffer(int16_t *buff, int offset, int size) {
  nes::apu::service(apu_out_buff, size);
  for (int i = 0; i < size; i++) {
    int16_t val = (int16_t)apu_out_buff[i] - 128;
#if SHAPONES_ENABLE_PDM_AUDIO
    audio_buff[offset] = val * 256;
#else
    audio_buff[offset] = val * 16;
#endif
    offset = (offset + 1) & (AUDIO_BUFF_LEN - 1);
  }
  return offset;
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
