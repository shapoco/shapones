// PicoCalc 320x320 LCD (ST7365P, ILI9488-like) on SPI1, RGB565.
//
// The init sequence and colour format are ClockworkPi's (their NES patch,
// picocalc.cpp). Unlike theirs this keeps no frame buffer: each NES line is
// sent by DMA as soon as the PPU finishes it, into a 256x240 window in the
// middle of the screen, which leaves the RAM for larger ROMs.
#include "lcd.hpp"

#include <initializer_list>

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>

namespace picocalc::lcd {

static spi_inst_t *const lcd_spi = spi1;
static constexpr int PIN_SCK = 10, PIN_MOSI = 11, PIN_MISO = 12;
static constexpr int PIN_CS = 13, PIN_DC = 14, PIN_RST = 15;

static int dma_ch;
static dma_channel_config dma_cfg;
static bool streaming = false;

static void wait_idle() {
  dma_channel_wait_for_finish_blocking(dma_ch);
  while (spi_get_hw(lcd_spi)->sr & SPI_SSPSR_BSY_BITS) tight_loop_contents();
  // drop whatever the DMA writes left in the receive FIFO
  while (spi_is_readable(lcd_spi)) (void)spi_get_hw(lcd_spi)->dr;
  spi_get_hw(lcd_spi)->icr = SPI_SSPICR_RORIC_BITS;
}

static void end_stream() {
  if (!streaming) return;
  wait_idle();
  gpio_put(PIN_CS, 1);
  streaming = false;
}

static void command(uint8_t cmd, const uint8_t *data = nullptr, int len = 0) {
  end_stream();
  gpio_put(PIN_DC, 0);
  gpio_put(PIN_CS, 0);
  spi_write_blocking(lcd_spi, &cmd, 1);
  if (len) {
    gpio_put(PIN_DC, 1);
    spi_write_blocking(lcd_spi, data, len);
  }
  gpio_put(PIN_CS, 1);
}

static void cmd1(uint8_t cmd, uint8_t d) { command(cmd, &d, 1); }

static void window(int x, int y, int w, int h) {
  int x1 = x + w - 1, y1 = y + h - 1;
  uint8_t xs[] = {(uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(x1 >> 8), (uint8_t)x1};
  uint8_t ys[] = {(uint8_t)(y >> 8), (uint8_t)y, (uint8_t)(y1 >> 8), (uint8_t)y1};
  command(0x2A, xs, 4);
  command(0x2B, ys, 4);
}

static void begin_write() {
  gpio_put(PIN_DC, 0);
  gpio_put(PIN_CS, 0);
  uint8_t c = 0x2C;
  spi_write_blocking(lcd_spi, &c, 1);
  gpio_put(PIN_DC, 1);
  streaming = true;
}

void init() {
  spi_init(lcd_spi, 20'000'000);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  for (int p : {PIN_CS, PIN_DC, PIN_RST}) {
    gpio_init(p);
    gpio_put(p, 1);
    gpio_set_dir(p, GPIO_OUT);
  }

  dma_ch = dma_claim_unused_channel(true);
  dma_cfg = dma_channel_get_default_config(dma_ch);
  channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&dma_cfg, true);
  channel_config_set_write_increment(&dma_cfg, false);
  channel_config_set_dreq(&dma_cfg, spi_get_dreq(lcd_spi, true));

  gpio_put(PIN_RST, 0);
  sleep_ms(10);
  gpio_put(PIN_RST, 1);
  sleep_ms(10);

  cmd1(0xF0, 0xC3);
  cmd1(0xF0, 0x96);
  cmd1(0x36, 0x48);  // memory access control
  cmd1(0x3A, 0x65);  // 16-bit colour
  cmd1(0xB1, 0xA0);  // frame rate
  cmd1(0xB4, 0x00);
  cmd1(0xB7, 0xC6);
  { uint8_t d[] = {0x02, 0xE0}; command(0xB9, d, 2); }
  { uint8_t d[] = {0x80, 0x06}; command(0xC0, d, 2); }
  cmd1(0xC1, 0x15);
  cmd1(0xC2, 0xA7);
  cmd1(0xC5, 0x04);
  { uint8_t d[] = {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xAA, 0x33}; command(0xE8, d, 8); }
  { uint8_t d[] = {0xF0, 0x06, 0x0F, 0x05, 0x04, 0x20, 0x37, 0x33, 0x4C, 0x37, 0x13, 0x14, 0x2B, 0x31};
    command(0xE0, d, sizeof d); }
  { uint8_t d[] = {0xF0, 0x11, 0x1B, 0x11, 0x0F, 0x0A, 0x37, 0x43, 0x4C, 0x37, 0x13, 0x13, 0x2C, 0x32};
    command(0xE1, d, sizeof d); }
  cmd1(0xF0, 0x3C);
  cmd1(0xF0, 0x69);
  cmd1(0x35, 0x00);
  command(0x11);  // sleep out
  sleep_ms(120);
  command(0x21);  // inversion on
  fill(0, 0, WIDTH, HEIGHT, 0x0000);
  command(0x29);  // display on
  sleep_ms(20);

  spi_set_baudrate(lcd_spi, 62'500'000);
}

void fill(int x, int y, int w, int h, uint16_t color) {
  static uint8_t row[WIDTH * 2];
  for (int i = 0; i < w; i++) {
    row[i * 2] = color >> 8;
    row[i * 2 + 1] = color;
  }
  window(x, y, w, h);
  begin_write();
  for (int j = 0; j < h; j++) spi_write_blocking(lcd_spi, row, w * 2);
  end_stream();
}

void frame_begin() {
  window(NES_X, NES_Y, NES_W, NES_H);
  begin_write();
}

void send_line(const uint8_t *pixels) {
  wait_idle();
  dma_channel_configure(dma_ch, &dma_cfg, &spi_get_hw(lcd_spi)->dr, pixels,
                        NES_W * 2, true);
}

void frame_end() { end_stream(); }

}  // namespace picocalc::lcd
