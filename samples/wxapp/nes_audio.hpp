#ifndef NES_AUDIO_HPP
#define NES_AUDIO_HPP

#include <pulse/error.h>
#include <pulse/simple.h>
#include <thread>
#include "stdint.h"

namespace nes_audio {

static constexpr int FREQ_HZ = 22050;
static constexpr int LATENCY_US = 100 * 1000;
static constexpr int CHUNK_SIZE = 256;

void play();
void stop();

}  // namespace nes_audio

#endif
