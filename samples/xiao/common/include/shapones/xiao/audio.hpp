#ifndef SHAPONES_XIAO_AUDIO_HPP
#define SHAPONES_XIAO_AUDIO_HPP

#include <stdint.h>

namespace shapones::xiao::audio {

uint32_t get_sampling_rate_hz();

void init();
void deinit();
void stream();

}  // namespace shapones::xiao::audio

#endif
