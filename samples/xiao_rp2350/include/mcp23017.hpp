#ifndef MCP23017_HPP
#define MCP23017_HPP

#include <hardware/i2c.h>

namespace mcp23017 {
enum class reg_t : uint8_t {
  IODIRA = 0x00,
  IODIRB = 0x01,
  IPOLA = 0x02,
  IPOLB = 0x03,
  GPIOA = 0x12,
  GPIOB = 0x13,
  OLATA = 0x14,
  OLATB = 0x15,
  GPPUA = 0x0C,
  GPPUB = 0x0D,
};

class Driver {
 private:
  i2c_inst_t* i2c;
  const uint8_t dev_addr;
  uint8_t port_dir[2] = {0xFF, 0xFF};
  uint8_t port_pullup[2] = {0, 0};
  uint8_t port_values[2] = {0xFF, 0xFF};

 public:
  Driver(i2c_inst_t* i2c_inst, uint8_t dev_addr = 0x20)
      : i2c(i2c_inst), dev_addr(dev_addr) {}

  void deinit() {
    write_reg(reg_t::IODIRA, 0xFF);
    write_reg(reg_t::IODIRB, 0xFF);
    write_reg(reg_t::GPPUA, 0x00);
    write_reg(reg_t::GPPUB, 0x00);
    write_reg(reg_t::OLATA, 0x00);
    write_reg(reg_t::OLATB, 0x00);
  }

  void write_reg(reg_t reg, uint8_t value) {
    uint8_t buf[2] = {static_cast<uint8_t>(reg), value};
    i2c_write_blocking(i2c, dev_addr, buf, 2, false);
  }

  uint8_t read_reg(reg_t reg) {
    uint8_t reg_addr = static_cast<uint8_t>(reg);
    uint8_t value;
    i2c_write_blocking(i2c, dev_addr, &reg_addr, 1, true);
    i2c_read_blocking(i2c, dev_addr, &value, 1, false);
    return value;
  }

  void set_dir(int port, uint8_t mask, uint8_t dir) {
    port_dir[port & 1] = (port_dir[port & 1] & ~mask) | (dir & mask);
    // GPA7 and GPB7 must be outputs
    if ((port & 1) == 0) {
      write_reg(reg_t::IODIRA, port_dir[0] & 0x7F);
    } else {
      write_reg(reg_t::IODIRB, port_dir[1] & 0x7F);
    }
  }

  void write_port(int port, uint8_t mask, uint8_t value) {
    port_values[port & 1] = (port_values[port & 1] & ~mask) | (value & mask);
    if ((port & 1) == 0) {
      write_reg(reg_t::OLATA, port_values[0]);
    } else {
      write_reg(reg_t::OLATB, port_values[1]);
    }
  }

  uint8_t read_port(int port) {
    if ((port & 1) == 0) {
      return read_reg(reg_t::GPIOA);
    } else {
      return read_reg(reg_t::GPIOB);
    }
  }

  void set_polarity(int port, uint8_t pol) {
    if ((port & 1) == 0) {
      write_reg(reg_t::IPOLA, pol);
    } else {
      write_reg(reg_t::IPOLB, pol);
    }
  }

  void set_pullup(int port, uint8_t mask, uint8_t pup) {
    port_pullup[port & 1] = (port_pullup[port & 1] & ~mask) | (pup & mask);
    if ((port & 1) == 0) {
      write_reg(reg_t::GPPUA, port_pullup[0]);
    } else {
      write_reg(reg_t::GPPUB, port_pullup[1]);
    }
  }
};

}  // namespace mcp23017

#endif
