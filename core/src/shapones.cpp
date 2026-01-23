#include "shapones/shapones.hpp"

namespace nes {

Config get_default_config() {
  Config cfg;
  cfg.apu_sampling_rate = 44100;
  return cfg;
}

result_t init(const Config &cfg) {
  for (int i = 0; i < NUM_LOCKS; i++) {
    SHAPONES_TRY(nes::lock_init(i));
  }
  SHAPONES_TRY(memory::init());
  SHAPONES_TRY(mapper::init());
  SHAPONES_TRY(cpu::init());
  SHAPONES_TRY(ppu::init());
  SHAPONES_TRY(apu::init());
  SHAPONES_TRY(menu::init());
  apu::set_sampling_rate(cfg.apu_sampling_rate);
  cpu::stop();
  return result_t::SUCCESS;
}

void deinit() {
  cpu::deinit();
  ppu::deinit();
  apu::deinit();
  mapper::deinit();
  memory::deinit();
  menu::deinit();
  for (int i = 0; i < NUM_LOCKS; i++) {
    nes::lock_deinit(i);
  }
}

result_t map_ines(const uint8_t *ines) {
  SHAPONES_TRY(memory::map_ines(ines));
  SHAPONES_TRY(reset());
  return result_t::SUCCESS;
}

result_t reset() {
  SHAPONES_TRY(cpu::reset());
  SHAPONES_TRY(ppu::reset());
  SHAPONES_TRY(apu::reset());
  return result_t::SUCCESS;
}

void stop() { cpu::stop(); }

result_t render_next_line(uint8_t *line_buff, bool skip_render,
                          ppu::status_t *status) {
  ppu::status_t s;
  if (!status) status = &s;
  do {
    SHAPONES_TRY(cpu::service());
    SHAPONES_TRY(ppu::service(line_buff, skip_render, status));
  } while (!(status->timing & ppu::timing_t::END_OF_VISIBLE_LINE));
  return result_t::SUCCESS;
}

result_t vsync(uint8_t *line_buff, bool skip_render) {
  ppu::status_t status;
  do {
    SHAPONES_TRY(cpu::service());
    SHAPONES_TRY(ppu::service(line_buff, skip_render, &status));
  } while (!(status.timing & ppu::timing_t::END_OF_FRAME));
  return result_t::SUCCESS;
}

}  // namespace nes
