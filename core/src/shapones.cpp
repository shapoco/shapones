#include "shapones/shapones.hpp"

namespace nes {

Config get_default_config() {
  Config cfg;
  cfg.apu_sampling_rate = 44100;
  return cfg;
}

void init(const Config &cfg) {
  for (int i = 0; i < NUM_LOCKS; i++) {
    nes::lock_init(i);
  }
  apu::set_sampling_rate(cfg.apu_sampling_rate);
  reset();
}

void deinit() { lock_deinit(LOCK_INTERRUPTS); }

void reset() {
  cpu::reset();
  ppu::reset();
  apu::reset();
}

uint32_t render_next_line(uint8_t *line_buff, bool skip_render) {
  uint32_t timing;
  do {
    cpu::service();
    timing = ppu::service(line_buff, skip_render);
  } while (!(timing & ppu::END_OF_VISIBLE_LINE));
  return timing;
}

void vsync(uint8_t *line_buff, bool skip_render) {
  uint32_t timing;
  do {
    cpu::service();
    timing = ppu::service(line_buff, skip_render);
  } while (!(timing & ppu::END_OF_FRAME));
}

}  // namespace nes
