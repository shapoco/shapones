
#include <Arduino.h>

#include "shapones/xiao/gpio.h"

#if defined(__cplusplus)
extern "C" {
#endif

int xiao_get_pin_number(xiao_pin_t pin) {
  switch (pin) {
    case XIAO_PIN_TFCARD_CS:
      return 6;
    default:
      return -1;
  }
}

void xiao_gpio_init(int pin, bool output) {
  pinMode(pin, output ? OUTPUT : INPUT);
}

void xiao_gpio_deinit(int pin) { pinMode(pin, INPUT); }

void xiao_gpio_put(int pin, bool value) {
  digitalWrite(pin, value ? HIGH : LOW);
}

#if defined(__cplusplus)
}
#endif
