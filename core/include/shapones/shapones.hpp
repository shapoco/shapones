#ifndef SHAPONES_HPP
#define SHAPONES_HPP

#include "shapones/common.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/memory.hpp"
#include "shapones/cpu.hpp"
#include "shapones/ppu.hpp"
#include "shapones/dma.hpp"
#include "shapones/apu.hpp"

namespace nes {

Config get_default_config();
void init(const Config& cfg);
void deinit();
void reset();
void render_next_line(uint8_t *line_buff);
void vsync(uint8_t *line_buff);

}

#endif
