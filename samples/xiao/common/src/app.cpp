#include "shapones/xiao/app.hpp"
#include "shapones/shapones.hpp"
#include "shapones/xiao/display.hpp"
#include "shapones/xiao/ioex.hpp"
#include "shapones/xiao/pins.h"
#include "shapones/xiao/spi.h"
#include "shapones/xiao/timer.h"

namespace shapones::xiao {

// NES color table
static const uint16_t COLOR_TABLE_0[] = {
    0x5055, 0x7002, 0x9001, 0x8030, 0x6040, 0x3060, 0x0050, 0x0041,
    0x0023, 0x0004, 0x0004, 0x0004, 0x4003, 0x0000, 0x0000, 0x0000,
    0x9099, 0xC005, 0xF033, 0xE062, 0xB081, 0x60A1, 0x2092, 0x0074,
    0x0056, 0x0027, 0x0008, 0x2007, 0x7006, 0x0000, 0x0000, 0x0000,
    0xF0FF, 0xF05A, 0xF078, 0xF0B6, 0xF0E5, 0xB0F5, 0x60F7, 0x20D8,
    0x00AB, 0x007C, 0x205D, 0x703D, 0xD03B, 0x4044, 0x0000, 0x0000,
    0xF0FF, 0xF0AD, 0xF0CC, 0xF0DB, 0xF0FB, 0xD0FB, 0xB0FB, 0x90EC,
    0x70DD, 0x70BE, 0x90AE, 0xB09E, 0xE0AD, 0xA0AA, 0x0000, 0x0000,
};
static const uint16_t COLOR_TABLE_1[] = {
    0x5505, 0x2700, 0x1900, 0x0803, 0x0604, 0x0306, 0x0005, 0x1004,
    0x3002, 0x4000, 0x4000, 0x4000, 0x3400, 0x0000, 0x0000, 0x0000,
    0x9909, 0x5C00, 0x3F03, 0x2E06, 0x1B08, 0x160A, 0x2209, 0x4007,
    0x6005, 0x7002, 0x8000, 0x7200, 0x6700, 0x0000, 0x0000, 0x0000,
    0xFF0F, 0xAF05, 0x8F07, 0x6F0B, 0x5F0E, 0x5B0F, 0x760F, 0x820D,
    0xB00A, 0xC007, 0xD205, 0xD703, 0xBD03, 0x4404, 0x0000, 0x0000,
    0xFF0F, 0xDF0A, 0xCF0C, 0xBF0D, 0xBF0F, 0xBD0F, 0xBB0F, 0xC90E,
    0xD70D, 0xE70B, 0xE90A, 0xEB09, 0xDE0A, 0xAA0A, 0x0000, 0x0000,
};

static uint64_t input_next_read_us = 0;

static uint8_t ppu_line_buff[shapones::SCREEN_WIDTH];
static uint64_t next_vsync_us = 0;
static bool ppu_frame_skip = false;
static volatile bool display_refresh_req = false;

static void update_input();
static void convert_color(int y);
static bool wait_vsync();

void app_init() {
  xiao_ioex_init();
  xiao_ioex_set_dir(0, 0xFF, 0xFE);
  xiao_ioex_set_dir(1, 0xFF, 0xFF);

  // Power on peripherals
  xiao_ioex_put(XIAO_IOEX_PERI_EN_PIN, true);
  xiao_sleep_ms(100);
  xiao_ioex_put(XIAO_IOEX_PERI_EN_PIN, false);

  xiao_spi_init();

  display::init();

  auto cfg = shapones::get_default_config();
  cfg.apu_sampling_rate = 22050;
  shapones::init(cfg);
  shapones::menu::show();
}

void cpu_service() {
  update_input();

  for (int i = 0; i < 10; i++) {
    cpu::service();
  }

  if (display_refresh_req && !xiao_spi_dma_is_busy()) {
    display_refresh_req = false;
    display::refresh();
  }
}

void ppu_service() {
  ppu::status_t ppu_status;
  ppu::service(ppu_line_buff, ppu_frame_skip, &ppu_status);

  if (!!(ppu_status.timing & ppu::timing_t::END_OF_VISIBLE_LINE) &&
      !ppu_frame_skip) {
    convert_color(ppu_status.focus_y);
  }
  if (!!(ppu_status.timing & ppu::timing_t::END_OF_VISIBLE_AREA)) {
    if (!ppu_frame_skip) {
      display_refresh_req = true;
    }
    ppu_frame_skip = wait_vsync();
  }
}

static void update_input() {
  uint64_t now_us = xiao_get_time_us();
  if (now_us >= input_next_read_us) {
    input_next_read_us = now_us + 1'000'000 / 120;

    uint16_t ioex_input = xiao_ioex_read_double();

    input::status_t input_status;
    input_status.raw = 0;
    for (int i = 0; i < 8; i++) {
      if ((ioex_input & (1 << XIAO_IOEX_GAME_BTN_PINS[i])) == 0) {
        input_status.raw |= (1 << i);
      }
    }
    input::set_status(0, input_status);
  }
}

static void convert_color(int y) {
  uint32_t *rd_ptr =
      (uint32_t *)(ppu_line_buff +
                   (shapones::SCREEN_WIDTH - display::WIDTH) / 2);
  uint32_t *wr_ptr = (uint32_t *)display::get_pixel_pointer(0, y);
  for (int i = 0; i < display::WIDTH / 8; i++) {
    uint32_t in0 = *(rd_ptr++);
    uint_fast16_t pix0 = COLOR_TABLE_0[(in0 >> 0) & 0xFF];
    uint_fast16_t pix1 = COLOR_TABLE_1[(in0 >> 8) & 0xFF];
    uint_fast16_t pix2 = COLOR_TABLE_0[(in0 >> 16) & 0xFF];
    uint_fast16_t pix3 = COLOR_TABLE_1[(in0 >> 24) & 0xFF];
    uint32_t in1 = *(rd_ptr++);
    uint_fast16_t pix4 = COLOR_TABLE_0[(in1 >> 0) & 0xFF];
    uint_fast16_t pix5 = COLOR_TABLE_1[(in1 >> 8) & 0xFF];
    uint_fast16_t pix6 = COLOR_TABLE_0[(in1 >> 16) & 0xFF];
    uint_fast16_t pix7 = COLOR_TABLE_1[(in1 >> 24) & 0xFF];
    uint32_t out0 = 0;
    out0 |= pix0;
    out0 |= pix1 << 8;
    out0 |= pix2 << 24;
    *(wr_ptr++) = out0;
    uint32_t out1 = 0;
    out1 |= pix2 >> 8;
    out1 |= pix3;
    out1 |= pix4 << 16;
    out1 |= pix5 << 24;
    *(wr_ptr++) = out1;
    uint32_t out2 = 0;
    out2 |= pix5 >> 8;
    out2 |= pix6 << 8;
    out2 |= pix7 << 16;
    *(wr_ptr++) = out2;
  }
}

static bool wait_vsync() {
  constexpr int FRAME_DELAY_US = 1'000'000 / 60;
  uint64_t now_us = xiao_get_time_us();
  int64_t wait_us = next_vsync_us - now_us;
  if (wait_us > 0) {
    xiao_sleep_us(wait_us);
  }
  next_vsync_us += FRAME_DELAY_US;
  if (now_us > next_vsync_us) {
    next_vsync_us = now_us;
  }
  return wait_us <= -5'000;
}

}  // namespace shapones::xiao
