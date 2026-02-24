#ifndef SHAPONES_XIAO_AUDIO_HPP
#define SHAPONES_XIAO_AUDIO_HPP

#include <stdint.h>

#include "shapones/xiao/ioex.hpp"

namespace shapones::xiao::audio {

uint32_t get_sampling_rate_hz();

void init();
void deinit();
void stream();

static inline void set_muted(bool muted) {
  xiao_ioex_write(XIAO_IOEX_INT_MUTE_PIN, muted);
}

}  // namespace shapones::xiao::audio

#endif
