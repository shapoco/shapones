// PicoCalc keyboard: the STM32 keyboard controller at I2C address 0x1F
// keeps a FIFO of key events, read two bytes at a time from register 9:
// state (1 pressed, 2 held, 3 released) and key code.
#include "kbd.hpp"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

namespace picocalc::kbd {

static i2c_inst_t *const kbd_i2c = i2c1;
static constexpr int PIN_SDA = 6, PIN_SCL = 7;
static constexpr uint8_t ADDR = 0x1F, REG_FIFO = 0x09;

static uint8_t down[256];
static volatile uint32_t n_errors = 0, n_events = 0;
uint32_t errors() { return n_errors; }
uint32_t events() { return n_events; }

void init() {
  i2c_init(kbd_i2c, 100'000);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_SDA);
  gpio_pull_up(PIN_SCL);
  for (int i = 0; i < 16 && poll() >= 0; i++) {}  // drop keys left from the menu
  for (auto &d : down) d = 0;
}

int poll() {
  uint8_t reg = REG_FIFO, ev[2];
  if (i2c_write_timeout_us(kbd_i2c, ADDR, &reg, 1, false, 5000) != 1) { n_errors++; return -1; }
  if (i2c_read_timeout_us(kbd_i2c, ADDR, ev, 2, false, 5000) != 2) { n_errors++; return -1; }
  if (ev[1] == 0) return -1;              // FIFO empty
  n_events++;
  if (ev[0] == 1) down[ev[1]] = 1;
  else if (ev[0] == 3) down[ev[1]] = 0;
  return ev[0] == 1 ? ev[1] : 0;          // newly pressed key, else 0
}

bool is_down(uint8_t key) { return down[key] != 0; }

}  // namespace picocalc::kbd
