#ifndef SHAPONES_XIAO_TIMER_HPP
#define SHAPONES_XIAO_TIMER_HPP

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

uint64_t xiao_get_time_ms();
uint64_t xiao_get_time_us();
void xiao_sleep_ms(uint32_t ms);
void xiao_sleep_us(uint32_t us);

#if defined(__cplusplus)
}
#endif

#endif  // SHAPONES_XIAO_TIMER_HPP
