#include "shapones/state.hpp"
#include "shapones/apu.hpp"
#include "shapones/cpu.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/ppu.hpp"

namespace nes {

static result_t write_state(void *file_handle);
static result_t read_state(void *file_handle);

uint32_t get_state_slot_size() {
  uint32_t size = 0;
  size += cpu::get_state_size();
  size += ppu::get_state_size();
  size += apu::get_state_size();
  size += interrupt::get_state_size();
  size += memory::get_state_size();
  size += mapper::instance->get_state_size();
  size += input::state_size();
  return size;
}

result_t save_state(const char *path, int slot) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_state_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, true, &f));

  size_t file_size = 0;
  SHAPONES_TRY(fs_size(f, &file_size));
  bool create = file_size < sizeof(state_file_header_t) +
                                sizeof(state_slot_entry_t) * STATE_MAX_SLOTS;

  do {
    if (create) {
      // write file header
      {
        uint8_t buff[state_file_header_t::SIZE];
        memset(buff, 0, sizeof(buff));
        state_file_header_t fh;
        fh.marker = STATE_MARKER;
        fh.version = STATE_VERSION;
        fh.slot_size = slot_size;
        fh.store(buff);

        res = fs_seek(f, 0);
        if (res != result_t::SUCCESS) break;
        res = fs_write(f, buff, sizeof(buff));
        if (res != result_t::SUCCESS) break;
      }

      // initialize index
      for (int i = 0; i < STATE_MAX_SLOTS; i++) {
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
      if (fh.marker != STATE_MARKER) {
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
      int offset = state_file_header_t::SIZE +
                   state_slot_entry_t::SIZE * STATE_MAX_SLOTS +
                   slot * slot_size;
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = write_state(f);
      if (res != result_t::SUCCESS) break;
    }

    // update index
    {
      uint8_t buff[state_slot_entry_t::SIZE];
      memset(buff, 0, sizeof(buff));
      state_slot_entry_t slot_header;
      slot_header.flags = STATE_SLOT_FLAG_USED;
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

static result_t write_state(void *f) {
  Exclusive lock_cpu(LOCK_CPU);
  Exclusive lock_ppu(LOCK_PPU);
  Exclusive lock_apu(LOCK_APU);
  SHAPONES_TRY(cpu::save_state(f));
  SHAPONES_TRY(ppu::save_state(f));
  SHAPONES_TRY(apu::save_state(f));
  SHAPONES_TRY(interrupt::save_state(f));
  SHAPONES_TRY(memory::save_state(f));
  SHAPONES_TRY(mapper::instance->save_state(f));
  SHAPONES_TRY(input::save_state(f));
  return result_t::SUCCESS;
}

result_t load_state(const char *path, int slot) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_state_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    // header check
    {
      uint8_t buff[state_file_header_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_file_header_t fh;
      fh.load(buff);
      if (fh.marker != STATE_MARKER) {
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
      int offset = state_file_header_t::SIZE +
                   state_slot_entry_t::SIZE * STATE_MAX_SLOTS +
                   slot * slot_size;
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = read_state(f);
      if (res != result_t::SUCCESS) break;
    }
  } while (0);

  fs_close(f);

  return res;
}

static result_t read_state(void *f) {
  Exclusive lock_cpu(LOCK_CPU);
  Exclusive lock_ppu(LOCK_PPU);
  Exclusive lock_apu(LOCK_APU);
  SHAPONES_TRY(cpu::load_state(f));
  SHAPONES_TRY(ppu::load_state(f));
  SHAPONES_TRY(apu::load_state(f));
  SHAPONES_TRY(interrupt::load_state(f));
  SHAPONES_TRY(memory::load_state(f));
  SHAPONES_TRY(mapper::instance->load_state(f));
  SHAPONES_TRY(input::load_state(f));
  return result_t::SUCCESS;
}

result_t enum_state_slots(const char *path, enum_state_slot_cb_t callback) {
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
      if (fh.marker != STATE_MARKER) {
        res = result_t::ERR_INVALID_STATE_FORMAT;
        break;
      }
    }

    // read slot entries
    for (int i = 0; i < STATE_MAX_SLOTS; i++) {
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

}  // namespace nes