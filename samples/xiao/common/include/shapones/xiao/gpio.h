#ifndef SHAPONES_XIAO_GPIO_HPP
#define SHAPONES_XIAO_GPIO_HPP
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum {
  XIAO_PIN_TFCARD_CS,
} xiao_pin_t;

int xiao_get_pin_number(xiao_pin_t pin);
void xiao_gpio_init(int pin, bool output);
void xiao_gpio_deinit(int pin);
void xiao_gpio_put(int pin, bool value);

#if defined(__cplusplus)
}
#endif

#endif
