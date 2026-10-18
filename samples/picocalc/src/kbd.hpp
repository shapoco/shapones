#pragma once
#include <stdint.h>

namespace picocalc::kbd {

// key codes from the keyboard controller
static constexpr uint8_t KEY_ALT = 0xA1, KEY_LSHIFT = 0xA2, KEY_RSHIFT = 0xA3,
                         KEY_CTRL = 0xA5, KEY_ESC = 0xB1, KEY_LEFT = 0xB4,
                         KEY_UP = 0xB5, KEY_DOWN = 0xB6, KEY_RIGHT = 0xB7,
                         KEY_ENTER = 0x0A, KEY_F1 = 0x81;

void init();
int poll();                // one FIFO event: -1 none, 0 not a press, else the key pressed
bool is_down(uint8_t key);
uint32_t errors();          // failed I2C reads so far (debug row)
uint32_t events();          // key events read so far

}  // namespace picocalc::kbd
