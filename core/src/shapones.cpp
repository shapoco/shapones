#include "shapones/shapones.hpp"
#include "shapones/state.hpp"

namespace nes {

char ines_path[MAX_PATH_LENGTH + 1] = "";
bool ines_mapped = false;

static void build_blend_table();

config_t get_default_config() {
  config_t cfg;
  cfg.apu_sampling_rate = 44100;
  return cfg;
}

const char *get_ines_path() { return ines_path; }

result_t init(const config_t &cfg) {
  build_blend_table();

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
  unmap_ines();
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
  unmap_ines();

  result_t res = result_t::SUCCESS;
  do {
    LockBlock ppu_block(LOCK_STATE_PPU);
    LockBlock apu_block(LOCK_STATE_APU);

    if (path && strnlen(path, MAX_PATH_LENGTH + 1) == 0) {
      res = result_t::ERR_PATH_TOO_LONG;
      break;
    }

    res = memory::map_ines(ines);
    if (res != result_t::SUCCESS) break;

    if (path) {
      strncpy(ines_path, path, MAX_PATH_LENGTH);
      ines_path[MAX_PATH_LENGTH] = '\0';
    } else {
      ines_path[0] = '\0';
    }

    reset();
  } while (0);

  if (res == result_t::SUCCESS) {
    ines_mapped = true;
  } else {
    ines_mapped = false;
    ines_path[0] = '\0';
  }
  return res;
}

void unmap_ines() {
  if (!ines_mapped) return;

  {
    LockBlock ppu_block(LOCK_STATE_PPU);
    LockBlock apu_block(LOCK_STATE_APU);
    stop();
    memory::unmap_ines();
    ines_mapped = false;
  }

  if (ines_path[0] != '\0') {
    unload_ines();
  }

  ines_path[0] = '\0';
}

result_t reset() {
  nes::state::reset();
  SHAPONES_TRY(mapper::instance->reset());
  SHAPONES_TRY(ppu::reset());
  SHAPONES_TRY(apu::reset());
  SHAPONES_TRY(input::reset());
  SHAPONES_TRY(interrupt::reset());
  SHAPONES_TRY(cpu::reset());
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

static void build_blend_table() {
  for (int i = 0; i < 64; i++) {
    uint32_t ci = NES_PALETTE_24BPP[i];
    uint8_t ri = (ci >> 16) & 0xff;
    uint8_t gi = (ci >> 8) & 0xff;
    uint8_t bi = ci & 0xff;
    for (int j = 0; j < 64; j++) {
      uint32_t cj = NES_PALETTE_24BPP[j];
      uint8_t rj = (cj >> 16) & 0xff;
      uint8_t gj = (cj >> 8) & 0xff;
      uint8_t bj = cj & 0xff;
      uint8_t r = (ri + rj) / 2;
      uint8_t g = (gi + gj) / 2;
      uint8_t b = (bi + bj) / 2;
      blend_table[(i << 6) | j] = nearest_rgb888(r, g, b);
    }
  }
}

}  // namespace nes
