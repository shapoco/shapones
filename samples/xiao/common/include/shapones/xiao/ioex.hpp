#ifndef SHAPONES_XIAO_IOEX_HPP
#define SHAPONES_XIAO_IOEX_HPP

#include <stdint.h>

static const int XIAO_IOEX_PERI_EN_PIN = 4;
static const int XIAO_IOEX_LCD_RST_PIN = 5;
static const int XIAO_IOEX_INT_MUTE_PIN = 6;
static const int XIAO_IOEX_BTN_MENU_PIN = 7;
static const int XIAO_IOEX_BTN_A_PIN = 8;
static const int XIAO_IOEX_BTN_B_PIN = 9;
static const int XIAO_IOEX_BTN_SELECT_PIN = 10;
static const int XIAO_IOEX_BTN_START_PIN = 11;
static const int XIAO_IOEX_BTN_UP_PIN = 12;
static const int XIAO_IOEX_BTN_DOWN_PIN = 13;
static const int XIAO_IOEX_BTN_LEFT_PIN = 14;
static const int XIAO_IOEX_BTN_RIGHT_PIN = 15;

static const int XIAO_IOEX_GAME_BTN_PINS[] = {
    XIAO_IOEX_BTN_A_PIN,     XIAO_IOEX_BTN_B_PIN,     XIAO_IOEX_BTN_SELECT_PIN,
    XIAO_IOEX_BTN_START_PIN, XIAO_IOEX_BTN_UP_PIN,    XIAO_IOEX_BTN_DOWN_PIN,
    XIAO_IOEX_BTN_LEFT_PIN,  XIAO_IOEX_BTN_RIGHT_PIN,
};

namespace shapones::xiao {

void xiao_ioex_init();
void xiao_ioex_deinit();
void xiao_ioex_set_dir_masked(int port, uint8_t mask, uint8_t value);
void xiao_ioex_write_masked(int port, uint8_t mask, uint8_t value);
uint8_t xiao_ioex_read_masked(int port, uint8_t mask);

static inline void xiao_ioex_set_dir(int pin, bool output) {
  int port = pin / 8;
  int bit = pin % 8;
  xiao_ioex_set_dir_masked(port, 1 << bit, output ? 0 : (1 << bit));
}

static inline void xiao_ioex_write(int pin, bool value) {
  int port = pin / 8;
  int bit = pin % 8;
  xiao_ioex_write_masked(port, 1 << bit, value ? (1 << bit) : 0);
}

static inline bool xiao_ioex_read(int pin) {
  int port = pin / 8;
  int bit = pin % 8;
  return (xiao_ioex_read_masked(port, 1 << bit) != 0);
}

uint16_t xiao_ioex_read_double();

}  // namespace shapones::xiao

#endif
