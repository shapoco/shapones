#pragma once
#include <stdint.h>

namespace picocalc::audio {
uint32_t sampling_rate_hz();
void init();
void set_muted(bool muted);
}
