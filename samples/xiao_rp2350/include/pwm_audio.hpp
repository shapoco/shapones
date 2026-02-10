#ifndef PWM_AUDIO_HPP
#define PWM_AUDIO_HPP

#include "stdint.h"

namespace shapones::xiao::rp2350::pwm_audio {

static constexpr int LATENCY = 256;

using fill_buffer_cb_t = void (*)();

using sample_t = uint32_t;

void init(uint32_t sys_clk_freq, fill_buffer_cb_t cb);
void play();
void stop();
sample_t* get_buffer(int bank);
sample_t* get_next_buffer();
void flip_buffer();

}  // namespace shapones::xiao::rp2350::pwm_audio

#endif
