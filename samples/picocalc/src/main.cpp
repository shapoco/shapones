// ShapoNES for the ClockworkPi PicoCalc (Pico 2 / Pico 2 W).
//
// Structure from the XIAO RP2350 port (samples/xiao): core 0 runs the CPU,
// keyboard and menu, core 1 the PPU; sound is refilled from a DMA
// interrupt. Screen set-up and colours from ClockworkPi's PicoCalc NES
// patch. Runs from the UF2 Loader boot menu; Ctrl+Esc returns to it.
#include <hardware/clocks.h>
#include <hardware/vreg.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include <initializer_list>

#include "shapones/shapones.hpp"
#include "shapones/xiao/spi.h"

#include "audio.hpp"
#include "kbd.hpp"
#include "lcd.hpp"
#include "picocalc_shell.h"

using namespace picocalc;

static constexpr uint32_t SYS_CLK_FREQ = 250'000'000;

// NES palette in RGB565 (ClockworkPi's table), byte-swapped so a uint16_t
// store puts the high byte first, as the LCD wants it
#define SW(c) (uint16_t)((((c) >> 8) & 0xFF) | (((c) & 0xFF) << 8))
static const uint16_t PALETTE[64] = {
    SW(0x52aa), SW(0x010e), SW(0x0093), SW(0x3011), SW(0x400c), SW(0x6006), SW(0x6000), SW(0x4080),
    SW(0x2180), SW(0x0220), SW(0x0220), SW(0x0220), SW(0x0188), SW(0x0000), SW(0x0000), SW(0x0000),
    SW(0x9cd3), SW(0x02b9), SW(0x319f), SW(0x611d), SW(0x8897), SW(0xa88c), SW(0xa904), SW(0x7220),
    SW(0x5320), SW(0x23a0), SW(0x0440), SW(0x03a4), SW(0x032e), SW(0x0000), SW(0x0000), SW(0x0000),
    SW(0xffff), SW(0x555f), SW(0x745f), SW(0xbb3f), SW(0xeabf), SW(0xfab7), SW(0xfbac), SW(0xdc44),
    SW(0xadc0), SW(0x7660), SW(0x56e4), SW(0x36ee), SW(0x35db), SW(0x4228), SW(0x0000), SW(0x0000),
    SW(0xffff), SW(0xaeff), SW(0xce7f), SW(0xdddf), SW(0xfddf), SW(0xfddb), SW(0xfdd7), SW(0xee73),
    SW(0xdeee), SW(0xbf6e), SW(0xaf73), SW(0x9f77), SW(0xaefd), SW(0xad55), SW(0x0000), SW(0x0000),
};

// ---------------------------------------------------------------- core 1: PPU

static uint8_t ppu_line[shapones::SCREEN_WIDTH];
static uint16_t lcd_lines[2][lcd::NES_W];
static int lcd_bank = 0;
static bool in_frame = false;
static uint64_t next_vsync_us = 0;
static int frame_skip_count = 0;

// sleep until the next 1/60 s; true if running more than 5 ms late
#ifndef NES_DEBUG_ROW
#define NES_DEBUG_ROW 0   // 1: show the input debug row
#endif

// --- debug row above the picture: what the emulator is receiving ---
// 8 squares: A B Select Start Up Down Left Right (green = pressed);
// then menu shown (blue), key events (white/grey flips on each event),
// I2C errors (red flips on each error), last key code in binary (yellow)
static volatile uint8_t dbg_pad = 0, dbg_key = 0;
static uint32_t dbg_shown_events = 0, dbg_shown_errors = 0;
static bool dbg_ev_flip = false, dbg_err_flip = false;

static void draw_debug_row() {
  constexpr int Y = 14, S = 10;
  int x = 8;
  for (int i = 0; i < 8; i++, x += S + 3)
    lcd::fill(x, Y, S, S, (dbg_pad >> i) & 1 ? 0x07E0 : 0x2104);
  x += 6;
  lcd::fill(x, Y, S, S, shapones::menu::is_shown() ? 0x001F : 0x2104);
  x += S + 9;
  uint32_t ev = kbd::events(), er = kbd::errors();
  if (ev != dbg_shown_events) { dbg_shown_events = ev; dbg_ev_flip = !dbg_ev_flip; }
  if (er != dbg_shown_errors) { dbg_shown_errors = er; dbg_err_flip = !dbg_err_flip; }
  lcd::fill(x, Y, S, S, dbg_ev_flip ? 0xFFFF : 0x8410);
  x += S + 3;
  lcd::fill(x, Y, S, S, dbg_err_flip ? 0xF800 : 0x2104);
  x += S + 9;
  for (int i = 7; i >= 0; i--, x += S + 1)
    lcd::fill(x, Y, S, S, (dbg_key >> i) & 1 ? 0xFFE0 : 0x2104);
}

static bool wait_vsync() {
  constexpr int FRAME_US = 1'000'000 / 60;
  uint64_t now = time_us_64();
  int64_t wait = (int64_t)(next_vsync_us - now);
  if (wait > 0) sleep_us(wait);
  next_vsync_us += FRAME_US;
  if (now > next_vsync_us) next_vsync_us = now;
  return wait <= -5000;
}

static void core1_main() {
  for (;;) {
    bool skip = frame_skip_count > 0;
    shapones::ppu::status_t st;
    shapones::ppu::service(ppu_line, skip, &st);

    if (st.is_end_of_visible_line() && !skip) {
      int y = st.focus_y;
      if (y == 0) {
        lcd::frame_begin();
        in_frame = true;
      }
      if (in_frame) {
        uint16_t *out = lcd_lines[lcd_bank];
        for (int x = 0; x < lcd::NES_W; x++) out[x] = PALETTE[ppu_line[x] & 0x3F];
        lcd::send_line((const uint8_t *)out);
        lcd_bank ^= 1;
        if (y == lcd::NES_H - 1) {
          lcd::frame_end();
          in_frame = false;
#if NES_DEBUG_ROW
          static int n;
          if ((++n & 3) == 0) draw_debug_row();
#endif
        }
      }
    }
    if (st.is_end_of_visible_area()) {
      if (wait_vsync())
        frame_skip_count = (frame_skip_count + 1) % 4;
      else
        frame_skip_count = 0;
    }
  }
}

// ---------------------------------------------------------- core 0: CPU, keys

static uint64_t next_input_us = 0;

static void return_to_menu() {
  multicore_reset_core1();
  audio::set_muted(true);
  lcd::fill(0, 0, lcd::WIDTH, lcd::HEIGHT, 0x0000);
  picocalc_shell_exit();   // only returns if there's no card
}

static void update_input() {
  uint64_t now = time_us_64();
  if (now < next_input_us) return;
  next_input_us = now + 1'000'000 / 120;

  for (int i = 0; i < 8; i++) {           // drain the key FIFO
    int k = kbd::poll();
    if (k < 0) break;
    if (k > 0) dbg_key = k;
    if (k == kbd::KEY_ESC) {
      if (kbd::is_down(kbd::KEY_CTRL)) return_to_menu();
      if (shapones::menu::is_shown())
        shapones::menu::hide();
      else
        shapones::menu::show();
    }
  }

  using namespace kbd;
  auto any = [](std::initializer_list<uint8_t> keys) {
    for (uint8_t k : keys)
      if (is_down(k)) return true;
    return false;
  };
  shapones::input::status_t s;
  s.raw = 0;
  // Enter picks in the menu (A there), and is Start in a game
  bool menu = shapones::menu::is_shown();
  s.A = any({']', '}', 'x', 'X', 'k', 'K'}) || (menu && is_down(KEY_ENTER));
  s.B = any({'[', '{', 'z', 'Z', 'j', 'J'});
  s.select = any({'-', '_', ' '});
  s.start = any({'=', '+'}) || (!menu && is_down(KEY_ENTER));
  s.up = is_down(KEY_UP);
  s.down = is_down(KEY_DOWN);
  s.left = is_down(KEY_LEFT);
  s.right = is_down(KEY_RIGHT);
  shapones::input::set_status(0, s);
  dbg_pad = s.raw;
}

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(100);
  set_sys_clock_khz(SYS_CLK_FREQ / 1000, true);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                  SYS_CLK_FREQ, SYS_CLK_FREQ);

  lcd::init();
  kbd::init();
  xiao_spi_init();

  auto cfg = shapones::get_default_config();
  cfg.apu_sampling_rate = audio::sampling_rate_hz();
  shapones::init(cfg);
  shapones::menu::show();

  audio::init();
  multicore_launch_core1(core1_main);

  for (;;) {
    update_input();
    for (int i = 0; i < 10; i++) shapones::cpu::service();
  }
}
