#include "shapones/shapones.hpp"
#include "shapones/state.hpp"

namespace nes {

char ines_path[MAX_PATH_LENGTH + 1] = "";

static result_t write_state(void *file_handle);
static result_t read_state(void *file_handle);

config_t get_default_config() {
  config_t cfg;
  cfg.apu_sampling_rate = 44100;
  return cfg;
}

const char* get_ines_path() {
  return ines_path;
}

result_t init(const config_t &cfg) {
  for (int i = 0; i < NUM_LOCKS; i++) {
    SHAPONES_TRY(nes::lock_init(i));
  }
  SHAPONES_TRY(interrupt::init());
  SHAPONES_TRY(memory::init());
  SHAPONES_TRY(mapper::init());
  SHAPONES_TRY(cpu::init());
  SHAPONES_TRY(ppu::init());
  SHAPONES_TRY(apu::init());
  SHAPONES_TRY(menu::init());
  SHAPONES_TRY(input::init());
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
  input::deinit();
  interrupt::deinit();
  for (int i = 0; i < NUM_LOCKS; i++) {
    nes::lock_deinit(i);
  }
}

result_t map_ines(const uint8_t *ines, const char *path) {
  SHAPONES_TRY(memory::map_ines(ines));
  SHAPONES_TRY(reset());
  strncpy(ines_path, path, MAX_PATH_LENGTH);
  return result_t::SUCCESS;
}

result_t reset() {
  SHAPONES_TRY(interrupt::reset());
  SHAPONES_TRY(cpu::reset());
  SHAPONES_TRY(ppu::reset());
  SHAPONES_TRY(apu::reset());
  SHAPONES_TRY(mapper::instance->reset());
  SHAPONES_TRY(input::reset());
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
