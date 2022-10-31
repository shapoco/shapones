#include "shapones/shapones.hpp"

namespace nes {

void reset() {
    cpu::reset();
    ppu::reset();
}

void render_next_line(uint8_t *line_buff) {
    const int batch_size = 3;

    while (ppu::is_in_hblank()) {
        int ppu_cycle = cpu::exec_next_inst(batch_size) * 3;
        ppu::render_next_block(line_buff, ppu_cycle);
    }

    while ( ! ppu::is_in_hblank()) {
        int ppu_cycle = cpu::exec_next_inst(batch_size) * 3;
        ppu::render_next_block(line_buff, ppu_cycle);
    }

}

void vsync(uint8_t *line_buff) {
    while (ppu::current_focus_y() != 0) {
        render_next_line(line_buff);
    }
}

}
