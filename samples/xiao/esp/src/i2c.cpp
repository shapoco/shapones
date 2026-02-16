#include "shapones/xiao/i2c.h"

#include <Wire.h>

extern "C" {

uint32_t xiao_i2c_ioex_frequency() { return 1'000'000; }

void xiao_i2c_init() { Wire.begin(); }

void xiao_i2c_deinit() { Wire.end(); }

void xiao_i2c_set_baudrate(uint32_t baudrate) { Wire.setClock(baudrate); }

void xiao_i2c_write_blocking(uint8_t dev_addr, const uint8_t *data,
                             uint32_t size, bool stop) {
  Wire.beginTransmission(dev_addr);
  Wire.write(data, size);
  if (stop) {
    Wire.endTransmission();
  }
}

void xiao_i2c_read_blocking(uint8_t dev_addr, uint8_t *data, uint32_t size,
                            bool stop) {
  Wire.requestFrom(dev_addr, size, stop);
  for (uint32_t i = 0; i < size; i++) {
    data[i] = Wire.read();
  }
}
}
