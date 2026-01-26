#include "shapones/state.hpp"
#include "shapones/apu.hpp"
#include "shapones/cpu.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/menu.hpp"
#include "shapones/ppu.hpp"

namespace nes::state {

static constexpr int SS_BUFF_DEPTH = 2;
static constexpr int SS_SCALING = nes::SCREEN_HEIGHT / state::SS_HEIGHT;
static constexpr int SS_CLIP_LEFT =
    (nes::SCREEN_WIDTH - state::SS_WIDTH * SS_SCALING) / 2;
static constexpr int SS_CLIP_TOP =
    (nes::SCREEN_HEIGHT - state::SS_HEIGHT * SS_SCALING) / 2;

uint8_t ss_buff[SS_SIZE_BYTES * SS_BUFF_DEPTH];
volatile int ss_wr_index = 0;
volatile int ss_num_stored = 0;
uint32_t ss_capture_counter = 0;

static uint32_t get_slot_offset(int slot, uint32_t slot_size) {
  return state_file_header_t::SIZE + state_slot_entry_t::SIZE * MAX_SLOTS +
         slot * slot_size;
}
static result_t write_screenshot(void *file_handle);
static result_t write_slot_data(void *file_handle);
static result_t read_slot_data(void *file_handle);

void reset() {
  Exclusive lock(LOCK_PPU);
  ss_wr_index = 0;
  ss_num_stored = 0;
  ss_capture_counter = 5 * 60;  // wait 5 seconds before first shot
  memset(ss_buff, 0, sizeof(ss_buff));
}

void auto_screenshot(int focus_y, const uint8_t *line_buff) {
  if (menu::is_shown()) return;
  if (ss_capture_counter > 0) {
    if (focus_y == SCREEN_HEIGHT - 1) {
      ss_capture_counter--;
    }
    return;
  }

  int sy = focus_y - SS_CLIP_TOP;
  int dy = sy / SS_SCALING;
  if (0 <= dy && dy < SS_HEIGHT && sy % SS_SCALING == 0) {
    Exclusive lock(LOCK_PPU);
    int wr_index = ss_wr_index;
    uint8_t *dst = &ss_buff[(wr_index * SS_SIZE_BYTES) + (dy * SS_WIDTH)];
    for (int dx = 0; dx < SS_WIDTH; dx++) {
      int sx = SS_CLIP_LEFT + dx * SS_SCALING;
      uint8_t c01 =
          blend_colors(line_buff[sx] & 0x3F, line_buff[sx + 1] & 0x3F);
      uint8_t c23 =
          blend_colors(line_buff[sx + 2] & 0x3F, line_buff[sx + 3] & 0x3F);
      dst[dx] = blend_colors(c01, c23);
    }
    if (dy == SS_HEIGHT - 1) {
      ss_wr_index = (wr_index + 1) % SS_BUFF_DEPTH;
      ss_num_stored++;
      if (ss_num_stored > SS_BUFF_DEPTH) {
        ss_num_stored = SS_BUFF_DEPTH;
      }
      ss_capture_counter = 5 * 60;  // wait 5 seconds before next shot
    }
  }
}

uint32_t get_slot_size() {
  uint32_t size = 0;
  size += SS_SIZE_BYTES;
  size += cpu::get_state_size();
  size += ppu::get_state_size();
  size += apu::get_state_size();
  size += interrupt::get_state_size();
  size += memory::get_state_size();
  size += mapper::instance->get_state_size();
  size += input::state_size();
  return size;
}

result_t save(const char *path, int slot) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, true, &f));

  size_t file_size = 0;
  SHAPONES_TRY(fs_size(f, &file_size));
  bool create = file_size < sizeof(state_file_header_t) +
                                sizeof(state_slot_entry_t) * MAX_SLOTS;

  do {
    if (create) {
      // write file header
      {
        uint8_t buff[state_file_header_t::SIZE];
        memset(buff, 0, sizeof(buff));
        state_file_header_t fh;
        fh.marker = MARKER;
        fh.version = VERSION;
        fh.slot_size = slot_size;
        fh.store(buff);

        res = fs_seek(f, 0);
        if (res != result_t::SUCCESS) break;
        res = fs_write(f, buff, sizeof(buff));
        if (res != result_t::SUCCESS) break;
      }

      // initialize index
      for (int i = 0; i < MAX_SLOTS; i++) {
        uint8_t buff[state_slot_entry_t::SIZE];
        memset(buff, 0, sizeof(buff));
        res = fs_write(f, buff, sizeof(buff));
        if (res != result_t::SUCCESS) break;
      }
    } else {
      // verify header
      uint8_t buff[state_file_header_t::SIZE];
      res = fs_seek(f, 0);
      if (res != result_t::SUCCESS) break;
      res = fs_read(f, buff, sizeof(buff));
      if (res != result_t::SUCCESS) break;
      state_file_header_t fh;
      fh.load(buff);
      if (fh.marker != MARKER) {
        res = result_t::ERR_INVALID_STATE_FORMAT;
        break;
      }
      if (fh.slot_size != slot_size) {
        res = result_t::ERR_STATE_SIZE_MISMATCH;
        break;
      }
    }

    // write slot data
    {
      uint32_t offset = get_slot_offset(slot, slot_size);
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = write_screenshot(f);
      if (res != result_t::SUCCESS) break;

      res = write_slot_data(f);
      if (res != result_t::SUCCESS) break;
    }

    // update index
    {
      uint8_t buff[state_slot_entry_t::SIZE];
      memset(buff, 0, sizeof(buff));
      state_slot_entry_t slot_header;
      slot_header.flags = SLOT_FLAG_USED;
      slot_header.play_time_sec =
          cpu::ppu_cycle_count / (cpu::CLOCK_FREQ_NTSC * 3);
      strncpy(slot_header.name, "No Name", state_slot_entry_t::NAME_LENGTH);
      slot_header.store(buff);

      int offset = state_file_header_t::SIZE + state_slot_entry_t::SIZE * slot;
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;
      res = fs_write(f, buff, sizeof(buff));
      if (res != result_t::SUCCESS) break;
    }

  } while (0);

  fs_close(f);
  return res;
}

result_t load(const char *path, int slot) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    // header check
    {
      uint8_t buff[state_file_header_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_file_header_t fh;
      fh.load(buff);
      if (fh.marker != MARKER) {
        res = result_t::ERR_INVALID_STATE_FORMAT;
        break;
      }
      if (fh.slot_size != slot_size) {
        res = result_t::ERR_STATE_SIZE_MISMATCH;
        break;
      }
    }

    // read slot data
    {
      uint32_t offset = get_slot_offset(slot, slot_size);
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = fs_read(f, ss_buff, SS_SIZE_BYTES);
      if (res != result_t::SUCCESS) break;
      ss_wr_index = 1;
      ss_num_stored = 1;
      ss_capture_counter = 5 * 60;  // wait 5 seconds before next shot

      res = read_slot_data(f);
      if (res != result_t::SUCCESS) break;
    }
  } while (0);

  fs_close(f);

  return res;
}

result_t read_screenshot(const char *path, int slot, uint8_t *out_buff) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    uint32_t offset = get_slot_offset(slot, slot_size);
    res = fs_seek(f, offset);
    if (res != result_t::SUCCESS) break;

    res = fs_read(f, out_buff, SS_SIZE_BYTES);
    if (res != result_t::SUCCESS) break;
  } while (0);

  fs_close(f);

  return res;
}

static result_t write_screenshot(void *f) {
  Exclusive lock(LOCK_PPU);
  int rd_index = (ss_wr_index + SS_BUFF_DEPTH - ss_num_stored) % SS_BUFF_DEPTH;
  return fs_write(f, &ss_buff[rd_index * SS_SIZE_BYTES], SS_SIZE_BYTES);
}

static result_t write_slot_data(void *f) {
  Exclusive lock_ppu(LOCK_PPU);
  Exclusive lock_apu(LOCK_APU);
  SHAPONES_TRY(cpu::save_state(f));
  SHAPONES_TRY(ppu::save_state(f));
  SHAPONES_TRY(apu::save_state(f));
  SHAPONES_TRY(input::save_state(f));
  SHAPONES_TRY(interrupt::save_state(f));
  SHAPONES_TRY(memory::save_state(f));
  SHAPONES_TRY(mapper::instance->save_state(f));
  return result_t::SUCCESS;
}

static result_t read_slot_data(void *f) {
  Exclusive lock_ppu(LOCK_PPU);
  Exclusive lock_apu(LOCK_APU);
  SHAPONES_TRY(cpu::load_state(f));
  SHAPONES_TRY(ppu::load_state(f));
  SHAPONES_TRY(apu::load_state(f));
  SHAPONES_TRY(input::load_state(f));
  SHAPONES_TRY(interrupt::load_state(f));
  SHAPONES_TRY(memory::load_state(f));
  SHAPONES_TRY(mapper::instance->load_state(f));
  return result_t::SUCCESS;
}

result_t enum_slots(const char *path, enum_slot_cb_t callback) {
  result_t res = result_t::SUCCESS;

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    // header check
    {
      uint8_t buff[state_file_header_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_file_header_t fh;
      fh.load(buff);
      if (fh.marker != MARKER) {
        res = result_t::ERR_INVALID_STATE_FORMAT;
        break;
      }
    }

    // read slot entries
    for (int i = 0; i < MAX_SLOTS; i++) {
      uint8_t buff[state_slot_entry_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_slot_entry_t slot_entry;
      slot_entry.index = i;
      slot_entry.load(buff);
      if (!callback(slot_entry)) {
        break;
      }
    }
  } while (0);
  fs_close(f);
  return res;
}

}  // namespace nes::state
