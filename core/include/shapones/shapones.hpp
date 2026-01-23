#ifndef SHAPONES_HPP
#define SHAPONES_HPP

#include "shapones/apu.hpp"
#include "shapones/common.hpp"
#include "shapones/cpu.hpp"
#include "shapones/dma.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/menu.hpp"
#include "shapones/ppu.hpp"

namespace nes {

Config get_default_config();

result_t init(const Config &cfg);
void deinit();

result_t map_ines(const uint8_t *ines);
result_t reset();

result_t render_next_line(uint8_t *line_buff, bool skip_render = false,
                          ppu::status_t *status = nullptr);
result_t vsync(uint8_t *line_buff, bool skip_render = false);

}  // namespace nes

#endif
