#include "shapones/cpu.hpp"
#include "shapones/ppu.hpp"
#include "shapones/apu.hpp"

namespace nes {

void reset() {
    cpu::reset();
    ppu::reset();
    apu::reset();
}

void render_next_line(uint8_t *line_buff) {
    bool eol;
    do {
        cpu::service();
        eol = ppu::service(line_buff);
    } while (!eol);
}

void vsync(uint8_t *line_buff) {
    while (ppu::current_focus_y() != ppu::SCAN_LINES - 1) {
        render_next_line(line_buff);
    }
}

}
