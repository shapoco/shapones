#include "shapones/xiao/ioex.hpp"

#include <PCA95x5.h>

namespace shapones::xiao {

PCA9535 ioex;

static uint8_t dir[2] = {0xFF, 0xFF};
static uint8_t output[2] = {0xFF, 0xFF};

static void update_pin_mode(int port);

void xiao_ioex_init() {
  Wire.begin();
  Wire.setClock(1'000'000);
  ioex.attach(Wire);
}

void xiao_ioex_deinit() {}

void xiao_ioex_set_dir(int port, uint8_t mask, uint8_t value) {
  dir[port] = (dir[port] & ~mask) | (value & mask);
  update_pin_mode(port);
}

uint8_t xiao_ioex_read(int port) {
  if (port == 0) {
    return ioex.read();
  } else {
    return ioex.read() >> 8;
  }
}

uint16_t xiao_ioex_read_double() { return ioex.read(); }

void xiao_ioex_write(int port, uint8_t mask, uint8_t value) {
  output[port] = (output[port] & ~mask) | (value & mask);
  ioex.write(output[0] | (output[1] << 8));
}

static void update_pin_mode(int port) {
  ioex.direction(dir[0] | (dir[1] << 8));
}

void xiao_ioex_put(int pin, bool value) {
  int port = pin / 8;
  int bit = pin % 8;
  xiao_ioex_write(port, 1 << bit, value ? (1 << bit) : 0);
}

}  // namespace shapones::xiao
