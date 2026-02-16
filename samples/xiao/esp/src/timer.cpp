#include "shapones/xiao/timer.h"

#include <Arduino.h>

extern "C" {

uint64_t xiao_get_time_ms() { return millis(); }
uint64_t xiao_get_time_us() { return micros(); }
void xiao_sleep_ms(uint32_t ms) { delay(ms); }
void xiao_sleep_us(uint32_t us) { delayMicroseconds(us); }
}
