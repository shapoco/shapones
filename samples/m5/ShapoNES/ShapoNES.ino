#include "SD.h"
#include <M5Unified.h>

#define SHAPONES_IMPLEMENTATION
#include "shapones_core.h"

#include "AdcButton.hpp"

#define SHAPONES_USE_CANVAS (0)

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
static constexpr int BUTTON_NUM_PINS = sizeof(BUTTON_ADC_PINS) / sizeof(BUTTON_ADC_PINS[0]);

static constexpr int BUTTON_A = 0;
static constexpr int BUTTON_B = 1;
static constexpr int BUTTON_SELECT = 2;
static constexpr int BUTTON_START = 3;
static constexpr int BUTTON_UP = 4;
static constexpr int BUTTON_DOWN = 5;
static constexpr int BUTTON_LEFT = 6;
static constexpr int BUTTON_RIGHT = 7;
static constexpr int BUTTON_DUMMY = 8;

static uint16_t frame_buff[BUFF_W * BUFF_H];
static int dma_next_y = nes::SCREEN_HEIGHT;

static uint8_t *ines = nullptr;
static uint8_t line_buff[nes::SCREEN_WIDTH];
static uint64_t next_vsync_us = 0;
static bool skip_frame = false;

static spinlock_t locks[nes::NUM_LOCKS];

static adc_button::Pin button_pins[BUTTON_NUM_PINS];

static nes::input::InputStatus input_state = {0};

#ifdef ARDUINO_M5STACK_ATOMS3
// clang-format off
const uint32_t COLOR_TABLE[] = {
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
const uint16_t COLOR_TABLE[] = {
  0xae73, 0xd120, 0x1500, 0x1340, 0x0e88, 0x02a8, 0x00a0, 0x4078, 0x6041, 0x2002, 0x8002, 0xe201, 0xeb19, 0x0000, 0x0000, 0x0000,
  0xf7bd, 0x9d03, 0xdd21, 0x1e80, 0x17b8, 0x0be0, 0x40d9, 0x61ca, 0x808b, 0xa004, 0x4005, 0x8704, 0x1104, 0x0000, 0x0000, 0x0000,
  0xffff, 0xff3d, 0x9f5b, 0x5fa4, 0xdff3, 0xb6fb, 0xacfb, 0xc7fc, 0xe7f5, 0x8286, 0xe94e, 0xd35f, 0x5b07, 0xae73, 0x0000, 0x0000,
  0xffff, 0x3faf, 0xbfc6, 0x5fd6, 0x3ffe, 0x3bfe, 0xf6fd, 0xd5fe, 0x34ff, 0xf4e7, 0x97af, 0xf9b7, 0xfe9f, 0xf7bd, 0x0000, 0x0000,
};
// clang-format on
#endif

static SHAPONES_INLINE void unpack565(uint16_t c, uint32_t *r, uint32_t *g, uint32_t *b) {
  c = ((c << 8) & 0xff00) | ((c >> 8) & 0x00ff);
  *r = (c >> 11) & 0x1F;
  *g = (c >> 5) & 0x3F;
  *b = c & 0x1F;
}

static SHAPONES_INLINE uint16_t pack565(uint32_t r, uint32_t g, uint32_t b) {
  uint16_t c = ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);
  return ((c << 8) & 0xff00) | ((c >> 8) & 0x00ff);
}

static void ppu_loop(void *arg);
static bool wait_vsync();
static bool dma_busy();
static void dma_start();
static void dma_maintain();

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  delay(500);

#ifdef ARDUINO_M5STACK_ATOMS3
  SPI.begin(TF_SCK_PIN, TF_MISO_PIN, TF_MOSI_PIN, TF_CS_PIN);
#endif

  while (false == SD.begin(TF_CS_PIN, SPI, 10000000)) {
    Serial.println("SD Wait...");
    delay(500);
  }

  const char* ines_path = "/super_mario_bros.nes";
  File ines_file = SD.open(ines_path, FILE_READ);
  if (!ines_file) {
    Serial.printf("Failed to open INES file: %s\n", ines_path);
    while (1) delay(1000);
  }
  size_t ines_size = ines_file.size();
  Serial.printf("INES file size: %d bytes\n", (int)ines_size);
  ines = new uint8_t[ines_size];
  ines_file.read(ines, ines_size);
  ines_file.close();

  SD.end();

  nes::memory::map_ines(ines);
  auto nes_cfg = nes::get_default_config();
  nes_cfg.apu_sampling_rate = 22050;
  nes::init(nes_cfg);
  
  xTaskCreatePinnedToCore(ppu_loop, "PPULoop", 8192, NULL, 10, NULL, PRO_CPU_NUM);

  for (int i = 0; i < BUTTON_NUM_PINS; i++) {
    pinMode(BUTTON_ADC_PINS[i], INPUT);
  }
  analogContinuousSetWidth(12);
  if (!analogContinuous(BUTTON_ADC_PINS, BUTTON_NUM_PINS, 3, 1000, &adc_complete)) {
    Serial.println("adc init fail");  
  }
  analogContinuousStart();
}

void loop() {
  for (int i = 0; i < 100; i++) {
    nes::cpu::service();
  }
}

void ARDUINO_ISR_ATTR adc_complete() {
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
  nes::input::set_raw(0, input_state);
}

static void ppu_loop(void *arg) {
  uint64_t next_wdt_reset_ms = 0;
  while (true) {
    int y;
    uint32_t timing = nes::ppu::service(line_buff, skip_frame, &y);
    if ((timing & nes::ppu::END_OF_VISIBLE_LINE) && !skip_frame) {
#ifdef ARDUINO_M5STACK_ATOMS3
      if (y % 2 == 0) {
        for (int x = 0; x < BUFF_W; x++) {
          uint32_t c0 = COLOR_TABLE[line_buff[x * 2 + 0] & 0x3f];
          uint32_t c1 = COLOR_TABLE[line_buff[x * 2 + 1] & 0x3f];
          resize_buff[x] = c0 + c1;
        }
      }
      else {
        uint16_t *wptr = frame_buff + y / 2 * BUFF_W;
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

    if (timing & nes::ppu::END_OF_VISIBLE_AREA) {
      if (!skip_frame) {
        dma_start();
      }
      skip_frame = wait_vsync();
    }
    else {
      dma_maintain();
    }

    uint64_t now_ms = millis();
    if (now_ms > next_wdt_reset_ms) {
      next_wdt_reset_ms = now_ms + 4000;
      vTaskDelay(1);
    }
  }
}

static bool wait_vsync() {
    constexpr int FRAME_DELAY_US = 1000000 / 60;
    uint64_t now_us = micros();
    int64_t wait_us = next_vsync_us - now_us;
    bool delaying = (wait_us <= 0);
    if (!delaying) {
        delayMicroseconds(wait_us);
    }
    next_vsync_us += FRAME_DELAY_US;
    if (now_us > next_vsync_us) {
        next_vsync_us = now_us;
    }
    return delaying;
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

void nes::lock_init(int id){
  spinlock_initialize(&locks[id]);
}
void nes::lock_deinit(int id){}
void nes::lock_get(int id){
  taskENTER_CRITICAL(&locks[id]);
}
void nes::lock_release(int id){
  taskEXIT_CRITICAL(&locks[id]);
}

