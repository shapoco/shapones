#include "shapones/input.hpp"

namespace nes::input {

static InputControl reg;
static InputStatus raw[2];
static uint8_t shift_reg[2];

InputStatus get_raw(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  return raw[player];
}

void set_raw(int player, InputStatus s) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  raw[player] = s;
}

void update() {
  if (reg.strobe) {
    shift_reg[0] = raw[0].raw;
    shift_reg[1] = raw[1].raw;
  }
}

uint8_t read_latched(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  uint8_t retval = shift_reg[player] & 1;
  shift_reg[player] >>= 1;
  return retval;
}

void write_control(uint8_t data) {
  reg.raw = data;
  update();
}

}  // namespace nes::input
