#!/usr/bin/env python3

from PIL import Image

import optparse
import sys
import numpy as np

parser = optparse.OptionParser()
parser.add_option(
    "-o",
    "--output",
    dest="output",
    default="../include/shapones/font8x16.hpp",
    help="出力先のC++ヘッダファイルのパス",
    metavar="FILE",
)
(options, args) = parser.parse_args()

img = Image.open("font8x16.png").convert("RGB")
width, height = img.size
img_pixels = np.array(
    [[img.getpixel((x, y)) for x in range(width)] for y in range(height)]
)

CHAR_WIDTH = 8
CHAR_HEIGHT = 16

CODE_FIRST = 0x20
CODE_LAST = 0xBF

PALETTE = [
    0x000000,  # Black
    0x808080,  # Gray
    0xC0C0C0,  # Silver
    0xFFFFFF,  # White
]

def nearest_index(r, g, b):
    min_dist = None
    best_index = None
    for color in PALETTE:
        pr = (color >> 16) & 0xFF
        pg = (color >> 8) & 0xFF
        pb = color & 0xFF
        dr = r - pr
        dg = g - pg
        db = b - pb
        dist = dr * dr + dg * dg + db * db
        if min_dist is None or dist < min_dist:
            min_dist = dist
            best_index = PALETTE.index(color)
    return best_index


with open(options.output, "w") as f:
    f.write("#ifndef SHAPONES_FONT8X16_HPP\n")
    f.write("#define SHAPONES_FONT8X16_HPP\n\n")
    f.write("#include <stdint.h>\n\n")
    f.write("namespace nes::menu {\n\n")
    f.write("const uint16_t FONT8X16_CODE_FIRST = 0x%02X;\n" % CODE_FIRST)
    f.write("const uint16_t FONT8X16_CODE_LAST = 0x%02X;\n" % CODE_LAST)
    f.write("const uint16_t FONT8X16_DATA[] = {\n")
    for code in range(CODE_FIRST, CODE_LAST + 1):
        i = code - CODE_FIRST
        ix = i % 16
        iy = i // 16
        char_x = ix * CHAR_WIDTH
        char_y = iy * CHAR_HEIGHT
        f.write("  // 0x%02X '%c'\n" % (code, chr(code)))
        for y in range(CHAR_HEIGHT):
            if y % 8 == 0:
                f.write("  ")
            word = 0
            for x in range(CHAR_WIDTH):
                pixel = img_pixels[char_y + y][char_x + x]
                r, g, b = pixel
                index = nearest_index(r, g, b)
                word |= (index & 0x03) << (x * 2)
            f.write("0x%04X," % word)
            if ((y + 1) % 8) == 0:
                f.write("\n")
            else:
                f.write(" ")
    f.write("};\n\n")
    f.write("}  // namespace nes\n\n")
    f.write("#endif // SHAPONES_FONT8X16_HPP\n")
