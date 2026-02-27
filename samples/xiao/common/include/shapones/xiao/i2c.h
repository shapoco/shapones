#ifndef SHAPONES_XIAO_I2C_H
#define SHAPONES_XIAO_I2C_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

uint32_t xiao_i2c_ioex_frequency();
void xiao_i2c_init();
void xiao_i2c_deinit();
void xiao_i2c_set_baudrate(uint32_t baudrate);
void xiao_i2c_write_blocking(uint8_t dev_addr, const uint8_t *data,
                             uint32_t size, bool nostop);
void xiao_i2c_read_blocking(uint8_t dev_addr, uint8_t *data, uint32_t size,
                            bool nostop);

#if defined(__cplusplus)
}
#endif

#endif
