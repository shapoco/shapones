#ifndef COMMON_HPP
#define COMMON_HPP

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "shapones/shapones.hpp"
#include "ws19804.hpp"
#include "mono8x16.hpp"

// system clock frequency
static constexpr uint32_t SYS_CLK_FREQ = 250 * MHZ;

// frame buffer for DMA (RGB444)
constexpr int FRAME_BUFF_WIDTH = 240;
constexpr int FRAME_BUFF_STRIDE = FRAME_BUFF_WIDTH * 3 / 2;
constexpr int FRAME_BUFF_HEIGHT = nes::SCREEN_HEIGHT;
extern uint8_t frame_buff[FRAME_BUFF_STRIDE * nes::SCREEN_HEIGHT];

// pad pins
static constexpr int PIN_PAD_A      = 2;
static constexpr int PIN_PAD_B      = 3;
static constexpr int PIN_PAD_START  = 4;
static constexpr int PIN_PAD_SELECT = 6;
static constexpr int PIN_PAD_RIGHT  = 7;
static constexpr int PIN_PAD_DOWN   = 14;
static constexpr int PIN_PAD_LEFT   = 26;
static constexpr int PIN_PAD_UP     = 27;
extern const int input_pins[];

// wait until some key is pressed
int wait_key();

// clear frame buffer with black color
void clear_frame_buff();

// draw string to frame buffer
void draw_string(int x, int y, const char *str);

// transfer image from frame buffer to LCD
void update_lcd();

#endif
