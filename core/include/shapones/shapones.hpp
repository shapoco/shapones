#ifndef SHAPONES_HPP
#define SHAPONES_HPP

#include "shapones/apu.hpp"
#include "shapones/common.hpp"
#include "shapones/cpu.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/menu.hpp"
#include "shapones/ppu.hpp"

namespace nes {

config_t get_default_config();

result_t init(const config_t &cfg);
void deinit();

result_t reset();

result_t render_next_line(uint8_t *line_buff, bool skip_render = false,
                          ppu::status_t *status = nullptr);
result_t vsync(uint8_t *line_buff, bool skip_render = false);

}  // namespace nes

#endif
