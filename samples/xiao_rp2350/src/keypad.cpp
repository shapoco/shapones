#include <pico/stdlib.h>

#include "keypad.hpp"
#include "mcp23017.hpp"

namespace shapones::xiao::rp::keypad {

static const int IOEX_PINS[] = {0, 1, 2, 3, 4, 5, 8, 9, 6};

mcp23017::Driver *ioex;
uint8_t ioex_values[2] = {0, 0};

void init(mcp23017::Driver *ie) { ioex = ie; }

void update() {
  for (int i = 0; i < 2; i++) {
    ioex_values[i] = ioex->read_port(i);
  }
}

bool is_pressed(int key) {
  if (key < 0 || key >= 8) return false;
  int port = IOEX_PINS[key] / 8;
  int bit = IOEX_PINS[key] % 8;
  return (ioex_values[port] & (1 << bit)) == 0;
}

}  // namespace shapones::xiao::rp2350::keypad