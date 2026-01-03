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

}

#endif
