#include <hardware/adc.h>
#include <pico/stdlib.h>

#include "keypad.hpp"

namespace shapones::xiao::rp2350::keypad {

static void adc_start();
static bool adc_get_result(uint16_t *out_value);

static uint16_t last_adc_values[NUM_ADC_PINS];
static ParallelSwitch switch_pins[NUM_ADC_PINS];
static int pin_index = 0;

void init() {
  adc_init();
  for (int i = 0; i < NUM_ADC_PINS; i++) {
    adc_gpio_init(26 + ADC_CHANNELS[i]);
  }
  adc_set_clkdiv(0);
  adc_select_input(ADC_CHANNELS[pin_index]);
  adc_start();
}

bool update() {
  uint16_t value;
  bool updated = adc_get_result(&value);
  if (updated) {
    last_adc_values[pin_index] = value;
    switch_pins[pin_index].update_value(value);
    pin_index = (pin_index + 1) % NUM_ADC_PINS;
    adc_select_input(ADC_CHANNELS[pin_index]);
    adc_start();
  }
  return updated;
}

bool is_pressed(int key) {
  int i = key / 3;
  int j = key % 3;
  return !!((switch_pins[i].read_current() >> j) & 0x1);
}

static void adc_start() { hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS); }

static bool adc_get_result(uint16_t *out_value) {
  bool ready = !!(adc_hw->cs & ADC_CS_READY_BITS);
  if (ready) {
    *out_value = (uint16_t)adc_hw->result;
  }
  return ready;
}

}  // namespace shapones::xiao::rp2350::keypad