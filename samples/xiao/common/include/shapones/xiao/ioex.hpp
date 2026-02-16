#ifndef SHAPONES_XIAO_IOEX_HPP
#define SHAPONES_XIAO_IOEX_HPP

#include <stdint.h>

static const int XIAO_IOEX_PERI_EN_PIN = 0;
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
void xiao_ioex_set_dir(int port, uint8_t mask, uint8_t value);
uint8_t xiao_ioex_read(int port);
uint16_t xiao_ioex_read_double();
void xiao_ioex_write(int port, uint8_t mask, uint8_t value);
void xiao_ioex_put(int pin, bool value);

}  // namespace shapones::xiao

#endif
