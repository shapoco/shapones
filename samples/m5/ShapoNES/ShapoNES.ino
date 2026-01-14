#include "SD.h"
#include <M5Unified.h>

#define SHAPONES_IMPLEMENTATION
#include "shapones_core.h"

#define SHAPONES_USE_CANVAS (0)

#if SHAPONES_USE_CANVAS
M5Canvas *canvas;
#else
uint16_t frame_buff[nes::SCREEN_WIDTH * nes::SCREEN_HEIGHT];
#endif
int dma_next_y = nes::SCREEN_HEIGHT;
constexpr int DMA_HEIGHT = 60;

uint8_t *ines = nullptr;
uint8_t line_buff[nes::SCREEN_WIDTH];
uint64_t next_vsync_us = 0;
bool skip_frame = false;

spinlock_t locks[nes::NUM_LOCKS];

// clang-format off
const uint16_t COLOR_TABLE[] = {
  0xae73, 0xd120, 0x1500, 0x1340, 0x0e88, 0x02a8, 0x00a0, 0x4078, 0x6041, 0x2002, 0x8002, 0xe201, 0xeb19, 0x0000, 0x0000, 0x0000,
  0xf7bd, 0x9d03, 0xdd21, 0x1e80, 0x17b8, 0x0be0, 0x40d9, 0x61ca, 0x808b, 0xa004, 0x4005, 0x8704, 0x1104, 0x0000, 0x0000, 0x0000,
  0xffff, 0xff3d, 0x9f5b, 0x5fa4, 0xdff3, 0xb6fb, 0xacfb, 0xc7fc, 0xe7f5, 0x8286, 0xe94e, 0xd35f, 0x5b07, 0xae73, 0x0000, 0x0000,
  0xffff, 0x3faf, 0xbfc6, 0x5fd6, 0x3ffe, 0x3bfe, 0xf6fd, 0xd5fe, 0x34ff, 0xf4e7, 0x97af, 0xf9b7, 0xfe9f, 0xf7bd, 0x0000, 0x0000,
};
// clang-format on

static bool wait_vsync();
static bool dma_busy();
static void dma_start();
static void dma_maintain();

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  delay(500);

#if SHAPONES_USE_CANVAS
  canvas = new M5Canvas(&M5.Display);
  canvas->createSprite(nes::SCREEN_WIDTH, nes::SCREEN_HEIGHT);
#endif

  while (false == SD.begin(GPIO_NUM_4, SPI, 10000000)) {
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
}

void loop() {
  for (int i = 0; i < 100; i++) {
    nes::cpu::service();
  }
}

static void ppu_loop(void *arg) {
  uint64_t next_wdt_reset_ms = 0;
  while (true) {
    int y;
    uint32_t timing = nes::ppu::service(line_buff, skip_frame, &y);
    if ((timing & nes::ppu::END_OF_VISIBLE_LINE) && !skip_frame) {
#if SHAPONES_USE_CANVAS
      uint16_t *wptr = (uint16_t *)(canvas->getBuffer()) + y * nes::SCREEN_WIDTH;
#else
      uint16_t *wptr = frame_buff + y * nes::SCREEN_WIDTH;
#endif
      for (int x = 0; x < nes::SCREEN_WIDTH; x++) {
        wptr[x] = COLOR_TABLE[line_buff[x] & 0x3f];
      }
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
  return (dma_next_y < nes::SCREEN_HEIGHT) || M5.Display.dmaBusy();
}

static void dma_start() {
  if (dma_busy()) return;

  dma_next_y = 0;
  dma_maintain();
}

static void dma_maintain() {
  if (dma_next_y >= nes::SCREEN_HEIGHT) return;
  if (M5.Display.dmaBusy()) return;

  int dx = (M5.Display.width() - nes::SCREEN_WIDTH) / 2;
  int dy = (M5.Display.height() - nes::SCREEN_HEIGHT) / 2 + dma_next_y;

  int h = DMA_HEIGHT;
  if (dma_next_y + h > nes::SCREEN_HEIGHT) {
    h = nes::SCREEN_HEIGHT - dma_next_y;
  }
#if SHAPONES_USE_CANVAS
  uint16_t *sptr = (uint16_t *)canvas->getBuffer();
#else
  uint16_t *sptr = frame_buff;
#endif
  sptr += dma_next_y * nes::SCREEN_WIDTH;
  M5.Display.endWrite();
  M5.Display.startWrite();
  M5.Display.pushImageDMA(dx, dy, nes::SCREEN_WIDTH, h, sptr);
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

