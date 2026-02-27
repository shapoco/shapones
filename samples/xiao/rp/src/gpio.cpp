#include "shapones/xiao/gpio.h"

#include <hardware/gpio.h>

#if defined(__cplusplus)
extern "C" {
#endif

void xiao_gpio_init(int pin, bool output) {
  gpio_init(pin);
  gpio_set_dir(pin, output ? GPIO_OUT : GPIO_IN);
}

void xiao_gpio_deinit(int pin) { gpio_set_dir(pin, GPIO_IN); }

void xiao_gpio_put(int pin, bool value) {
  gpio_put(pin, value ? 1 : 0);
}

bool xiao_gpio_get(int pin) { return gpio_get(pin) != 0; }

#if defined(__cplusplus)
}
#endif
