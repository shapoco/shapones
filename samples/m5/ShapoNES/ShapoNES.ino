#include "SD.h"
#include <M5Unified.h>
#include "shapones_core.h"

M5Canvas *canvas;

uint8_t *ines = nullptr;
uint8_t line_buff[nes::SCREEN_WIDTH];
bool skip_frame = false;

uint16_t COLOR_TABLE[] = {
  0xae73, 0xd120, 0x1500, 0x1340, 0xe88, 0x2a8, 0xa0, 0x4078, 0x6041, 0x2002, 0x8002, 0xe201, 0xeb19, 0x0, 0x0, 0x0, 0xf7bd, 0x9d03, 0xdd21, 0x1e80, 0x17b8, 0xbe0, 0x40d9, 0x61ca, 0x808b, 0xa004, 0x4005, 0x8704, 0x1104, 0x0, 0x0, 0x0, 0xffff, 0xff3d, 0x9f5b, 0x5fa4, 0xdff3, 0xb6fb, 0xacfb, 0xc7fc, 0xe7f5, 0x8286, 0xe94e, 0xd35f, 0x5b07, 0xae73, 0x0, 0x0, 0xffff, 0x3faf, 0xbfc6, 0x5fd6, 0x3ffe, 0x3bfe, 0xf6fd, 0xd5fe, 0x34ff, 0xf4e7, 0x97af, 0xf9b7, 0xfe9f, 0xf7bd, 0x0, 0x0,
};

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  delay(500);

  canvas = new M5Canvas(&M5.Display);
  canvas->createSprite(nes::SCREEN_WIDTH, nes::SCREEN_HEIGHT);

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

  nes::memory::map_ines(ines);
  auto nes_cfg = nes::get_default_config();
  nes_cfg.apu_sampling_rate = 22050;
  nes::init(nes_cfg);
}

void loop() {
  nes::cpu::service();

  int y;
  uint32_t timing = nes::ppu::service(line_buff, skip_frame, &y);
  if ((timing & nes::ppu::END_OF_VISIBLE_LINE) && !skip_frame) {
    uint16_t *wptr = (uint16_t *)(canvas->getBuffer());
    wptr += y * nes::SCREEN_WIDTH;
    for (int x = 0; x < nes::SCREEN_WIDTH; x++) {
      wptr[x] = COLOR_TABLE[line_buff[x] & 0x3f];
    }
  }
  if (timing & nes::ppu::END_OF_VISIBLE_AREA) {
    if (!skip_frame) {
      // display frame
      int x = (M5.Display.width() - nes::SCREEN_WIDTH) / 2;
      int y = (M5.Display.height() - nes::SCREEN_HEIGHT) / 2;
      canvas->pushSprite(x, y);
    }
    skip_frame = false;
  }
}

void nes::lock_init(int id){}
void nes::lock_deinit(int id){}
void nes::lock_get(int id){}
void nes::lock_release(int id){}

