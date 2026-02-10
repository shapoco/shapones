#ifndef SHAPONES_XIAO_RP2350_DISPLAY_HPP
#define SHAPONES_XIAO_RP2350_DISPLAY_HPP

#include <stdint.h>

#include "spibus.h"

namespace shapones::xiao::rp2350::display {

static constexpr int WIDTH = 240;
static constexpr int HEIGHT = 240;
static constexpr int STRIDE = WIDTH * 3 / 2;

void init();
uint8_t *get_pixel_pointer(int x, int y);
void set_pixel(int x, int y, uint16_t color);
void fill_rect(int x, int y, int w, int h, uint16_t color);
void draw_rect(int x, int y, int w, int h, uint16_t color);
void refresh();

}  // namespace shapones::xiao::rp2350::display

#endif
