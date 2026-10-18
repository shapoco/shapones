#pragma once
#include <stdint.h>

namespace picocalc::lcd {

static constexpr int WIDTH = 320, HEIGHT = 320;
static constexpr int NES_W = 256, NES_H = 240;
static constexpr int NES_X = (WIDTH - NES_W) / 2, NES_Y = (HEIGHT - NES_H) / 2;

void init();
void fill(int x, int y, int w, int h, uint16_t color);  // big-endian RGB565 handled inside

// one NES frame: frame_begin, then up to 240 send_line calls (512 bytes of
// big-endian RGB565 each; the buffer must stay untouched until the next
// send_line or frame_end), then frame_end
void frame_begin();
void send_line(const uint8_t *pixels);
void frame_end();

}  // namespace picocalc::lcd
