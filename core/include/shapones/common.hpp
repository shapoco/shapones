#ifndef SHAPONES_BASIC_HPP
#define SHAPONES_BASIC_HPP

#if !(SHAPONES_NO_STDLIB)
#include "stdint.h"
#endif

namespace nes {

using addr_t = uint16_t;
using cycle_t = unsigned int;

static constexpr int SCREEN_WIDTH = 256;
static constexpr int SCREEN_HEIGHT = 240;

static constexpr addr_t WRAM_SIZE = 2 * 1024;
static constexpr addr_t VRAM_SIZE = 4 * 1024;
static constexpr addr_t PRGROM_RANGE = 32 * 1024;
static constexpr addr_t PRGRAM_RANGE = 8 * 1024;
static constexpr addr_t CHRROM_RANGE = 8 * 1024;

}  // namespace nes

#endif
