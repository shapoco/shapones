#include "shapones/xiao/ioex.hpp"
#include "shapones/xiao/i2c.h"

namespace shapones::xiao {

static constexpr uint8_t DEV_ADDR = 0x20;

enum class reg_t : uint8_t {
  GPIOA = 0x00,
  GPIOB = 0x01,
  OLATA = 0x02,
  OLATB = 0x03,
  IPOLA = 0x04,
  IPOLB = 0x05,
  IODIRA = 0x06,
  IODIRB = 0x07,
};

static uint8_t dir[2] = {0xFF, 0xFF};
static uint8_t output[2] = {0xFF, 0xFF};
static uint8_t ipol[2] = {0, 0};

static void write_reg(reg_t reg, uint8_t value);
static uint8_t read_reg(reg_t reg);

void xiao_ioex_init() {}

void xiao_ioex_deinit() {}

void xiao_ioex_set_dir_masked(int port, uint8_t mask, uint8_t value) {
  dir[port] = (dir[port] & ~mask) | (value & mask);
  write_reg(port == 0 ? reg_t::IODIRA : reg_t::IODIRB, dir[port]);
}

void xiao_ioex_set_input_polarity(int port, uint8_t mask, uint8_t value) {
  ipol[port] = (ipol[port] & ~mask) | (value & mask);
  write_reg(port == 0 ? reg_t::IPOLA : reg_t::IPOLB, ipol[port]);
}

void xiao_ioex_write_masked(int port, uint8_t mask, uint8_t value) {
  output[port] = (output[port] & ~mask) | (value & mask);
  write_reg(port == 0 ? reg_t::OLATA : reg_t::OLATB, output[port]);
}

uint8_t xiao_ioex_read_masked(int port, uint8_t mask) {
  return read_reg(port == 0 ? reg_t::GPIOA : reg_t::GPIOB) & mask;
}

static void write_reg(reg_t reg, uint8_t value) {
  uint8_t buf[2] = {static_cast<uint8_t>(reg), value};
  xiao_i2c_write_blocking(DEV_ADDR, buf, 2, true);
}

static uint8_t read_reg(reg_t reg) {
  uint8_t reg_addr = static_cast<uint8_t>(reg);
  uint8_t value;
  xiao_i2c_write_blocking(DEV_ADDR, &reg_addr, 1, true);
  xiao_i2c_read_blocking(DEV_ADDR, &value, 1, true);
  return value;
}

}  // namespace shapones::xiao
