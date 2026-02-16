#ifndef SHAPONES_XIAO_GPIO_H
#define SHAPONES_XIAO_GPIO_H

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

void xiao_gpio_init(int pin, bool output);
void xiao_gpio_deinit(int pin);
void xiao_gpio_put(int pin, bool value);
bool xiao_gpio_get(int pin);

#if defined(__cplusplus)
}
#endif

#endif
