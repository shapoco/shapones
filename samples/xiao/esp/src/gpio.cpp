
#include <Arduino.h>

#include "shapones/xiao/gpio.h"

#if defined(__cplusplus)
extern "C" {
#endif

void xiao_gpio_init(int pin, bool output) {
  pinMode(pin, output ? OUTPUT : INPUT);
}

void xiao_gpio_deinit(int pin) { pinMode(pin, INPUT); }

void xiao_gpio_put(int pin, bool value) {
  digitalWrite(pin, value ? HIGH : LOW);
}

bool xiao_gpio_get(int pin) { return digitalRead(pin) == HIGH; }

#if defined(__cplusplus)
}
#endif
