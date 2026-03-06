#include "shapones/shapones.hpp"
#include "shapones/state.hpp"

namespace shapones {

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

  for (int i = 0; i < NUM_SPINLOCKS; i++) {
    SHAPONES_RET_ERR(shapones::spinlock_init(i));
  }
  for (int i = 0; i < NUM_SEMAPHORES; i++) {
    SHAPONES_RET_ERR(shapones::semaphore_init(i));
  }
  SHAPONES_RET_ERR(interrupt::init());
  SHAPONES_RET_ERR(memory::init());
  SHAPONES_RET_ERR(mapper::init());
  SHAPONES_RET_ERR(cpu::init());
  SHAPONES_RET_ERR(ppu::init());
  SHAPONES_RET_ERR(apu::init());
  SHAPONES_RET_ERR(menu::init());
  SHAPONES_RET_ERR(input::init());
  apu::set_sampling_rate(cfg.apu_sampling_rate);
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
  for (int i = 0; i < NUM_SPINLOCKS; i++) {
    shapones::spinlock_deinit(i);
  }
  for (int i = 0; i < NUM_SEMAPHORES; i++) {
    shapones::semaphore_deinit(i);
  }
}

result_t map_ines(const uint8_t *ines, const char *path) {
  unmap_ines();

  result_t res = result_t::SUCCESS;
  do {
    SemaphoreBlock ppu_block(SEMAPHORE_PPU);
    SemaphoreBlock apu_block(SEMAPHORE_APU);

    if (path && strnlen(path, MAX_PATH_LENGTH + 1) == 0) {
      SHAPONES_BRK_ERR(res, result_t::ERR_FS_PATH_TOO_LONG);
    }

    SHAPONES_BRK_ERR(res, memory::map_ines(ines));

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
    SemaphoreBlock ppu_block(SEMAPHORE_PPU);
    SemaphoreBlock apu_block(SEMAPHORE_APU);
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
  shapones::state::reset();
  SHAPONES_RET_ERR(mapper::instance->reset());
  SHAPONES_RET_ERR(ppu::reset());
  SHAPONES_RET_ERR(apu::reset());
  SHAPONES_RET_ERR(input::reset());
  SHAPONES_RET_ERR(interrupt::reset());
  SHAPONES_RET_ERR(cpu::reset());
  return result_t::SUCCESS;
}

void stop() { cpu::stop(); }

result_t render_next_line(uint8_t *line_buff, bool skip_render,
                          ppu::status_t *status) {
  ppu::status_t s;
  if (!status) status = &s;
  do {
    SHAPONES_RET_ERR(cpu::service());
    SHAPONES_RET_ERR(ppu::service(line_buff, skip_render, status));
  } while (!status->is_end_of_visible_line());
  return result_t::SUCCESS;
}

result_t vsync(uint8_t *line_buff, bool skip_render) {
  ppu::status_t status;
  do {
    SHAPONES_RET_ERR(cpu::service());
    SHAPONES_RET_ERR(ppu::service(line_buff, skip_render, &status));
  } while (!status.is_end_of_frame());
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

}  // namespace shapones
