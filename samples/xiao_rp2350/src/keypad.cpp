#include <pico/stdlib.h>

#include "keypad.hpp"

namespace shapones::xiao::rp::keypad {

static const int IOEX_PINS[] = {8, 9, 10, 11, 12, 13, 14, 15, 7};

pca9555::Driver *ioex;
uint8_t ioex_values[2] = {0, 0};

void init(pca9555::Driver *ie) { ioex = ie; }

void update() {
  for (int i = 0; i < 2; i++) {
    ioex_values[i] = ioex->read_port(i);
  }
}

bool is_pressed(int key) {
  if (key < 0 || key >= 9) return false;
  int port = IOEX_PINS[key] / 8;
  int bit = IOEX_PINS[key] % 8;
  return (ioex_values[port] & (1 << bit)) == 0;
}

}  // namespace shapones::xiao::rp::keypad