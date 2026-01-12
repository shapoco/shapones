#include "shapones/shapones.hpp"

namespace nes {

Config get_default_config() {
    Config cfg;
    cfg.apu_sampling_rate = 44100;
    return cfg;
}

void init(const Config& cfg) {
    for (int i = 0; i < NUM_LOCKS; i++) {
        nes::lock_init(i);
    }
    apu::set_sampling_rate(cfg.apu_sampling_rate);
    reset();
}

void deinit() {
    lock_deinit(LOCK_INTERRUPTS);
}

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
