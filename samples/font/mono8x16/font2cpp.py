#!/usr/bin/env python3

from PIL import Image

import sys
import numpy as np

W = 8
H = 16
PADDING = 1

COLS = 16
ROWS = 6
LENGTH = COLS * ROWS

INPUT_IMAGE = sys.argv[1]
NAME_SPACE = sys.argv[2]

img = Image.open(INPUT_IMAGE)
width, height = img.size
img_pixels = np.array([[img.getpixel((x,y)) for x in range(width)] for y in range(height)])

print('#include "stdint.h"')
print()
print('namespace %s {' % NAME_SPACE)
print()
print('const uint8_t bmp[] = {')

index = 0

for iy in range(ROWS):
    y_step = iy * (H + PADDING)
    for ix in range(COLS):
        x_step = ix * (W + PADDING)
        code = 0x20 + iy * 16 + ix
        
        for y_sub in range(H):
            byte = 0
            for x_sub in range(W):
                y = y_step + y_sub
                x = x_step + x_sub
                pixel = img_pixels[y, x]
                if pixel[0] == 255:
                    byte |= 1 << x_sub
            
            if (index % 16 == 0):
                print('    ', end='')
            print('0x%02x, ' % byte, end='')
            if (index % 16 == 15) or (index == LENGTH - 1):
                print()
            index += 1

print('};')
print()
print('}')
