#include "shapones/shapones.hpp"

namespace nes {

void reset() {
    cpu::reset();
    ppu::reset();
}

void render_next_line(uint8_t *line_buff) {
    cpu::render_next_line(line_buff);
}

void vsync(uint8_t *line_buff) {
    while (ppu::current_focus_y() != 0) {
        render_next_line(line_buff);
    }
}

}
