#include "mono8x16.hpp"

namespace mono8x16 {

static constexpr int W = 8;
static constexpr int H = 16;
static constexpr int CHAR_STRIDE = (W * H + 7) / 8;

void draw_char_rgb444(
    uint8_t *dest,
    int dest_stride,
    int dest_w,
    int dest_h,
    int x0,
    int y0,
    const char c,
    uint16_t color
) {
    if (c <= 0x20 || 0x80 <= c) return;
    const uint8_t *rd_ptr = bmp + (c - 0x20) * CHAR_STRIDE;
    for (int iy = 0; iy < H; iy++) {
        int y = y0 + iy;
        uint8_t pattern = *(rd_ptr++);
        if (0 <= y && y < dest_h) {
            for (int ix = 0; ix < W; ix++) {
                int x = x0 + ix;
                if ((pattern & 1) && 0 <= x && x < dest_w) {
                    int wr_index = dest_stride * y + x * 3 / 2;
                    if ((x & 1) == 0) {
                        dest[wr_index  ] = (color >> 4) & 0xff;
                        dest[wr_index+1] &= 0x0f;
                        dest[wr_index+1] |= (color << 4) & 0xf0;
                    }
                    else {
                        dest[wr_index  ] &= 0xf0;
                        dest[wr_index  ] |= (color >> 8) & 0xf;
                        dest[wr_index+1] = color & 0xff;
                    }
                }
                pattern >>= 1;
            }
        }
    }
}

void draw_string_rgb444(
    uint8_t *dest,
    int dest_stride,
    int dest_w,
    int dest_h,
    int x0,
    int y0,
    const char* str,
    uint16_t color
) {
    char c;
    while ((c = *(str++)) != '\0') {
        draw_char_rgb444(
            dest,
            dest_stride,
            dest_w,
            dest_h,
            x0,
            y0,
            c,
            color
        );
        x0 += W + 1;
    }
}

}
