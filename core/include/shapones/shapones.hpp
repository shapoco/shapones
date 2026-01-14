#ifndef SHAPONES_HPP
#define SHAPONES_HPP

#include "shapones/common.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/memory.hpp"
#include "shapones/mapper.hpp"
#include "shapones/cpu.hpp"
#include "shapones/ppu.hpp"
#include "shapones/dma.hpp"
#include "shapones/apu.hpp"

namespace nes {

Config get_default_config();
void init(const Config& cfg);
void deinit();
void reset();
uint32_t render_next_line(uint8_t *line_buff, bool skip_render = false);
void vsync(uint8_t *line_buff, bool skip_render = false);

}

#endif
