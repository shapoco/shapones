#!/bin/env python3

import sys

with open(sys.argv[1], 'rb') as f: 
    data = f.read()

print('#include "stdint.h"')
print('#include "nes_rom.hpp"')
print('const uint8_t nes_rom[] = {')

n = len(data)
for i in range(n):
    if (i % 16 == 0):
        print('    ', end='')
    print('0x%02x, ' % data[i], end='')
    if (i % 16 == 15) or (i == n - 1):
        print()

print('}')
