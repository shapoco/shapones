#include "shapones/xiao/i2c.h"
#include "shapones/xiao/pins.h"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

static i2c_inst_t *const i2c_inst = i2c1;

extern "C" {

uint32_t xiao_i2c_ioex_frequency() { return 400'000; }

void xiao_i2c_init() {
  i2c_init(i2c_inst, xiao_i2c_ioex_frequency());
  gpio_set_function(XIAO_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(XIAO_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(XIAO_I2C_SDA_PIN);
  gpio_pull_up(XIAO_I2C_SCL_PIN);
}

void xiao_i2c_deinit() {
  i2c_deinit(i2c_inst);
  gpio_deinit(XIAO_I2C_SDA_PIN);
  gpio_deinit(XIAO_I2C_SCL_PIN);
}

void xiao_i2c_set_baudrate(uint32_t baudrate) {
  i2c_set_baudrate(i2c_inst, baudrate);
}

void xiao_i2c_write_blocking(uint8_t dev_addr, const uint8_t *data,
                             uint32_t size, bool nostop) {
  i2c_write_blocking(i2c_inst, dev_addr, data, size, nostop);
}

void xiao_i2c_read_blocking(uint8_t dev_addr, uint8_t *data, uint32_t size,
                            bool nostop) {
  i2c_read_blocking(i2c_inst, dev_addr, data, size, nostop);
}
}
