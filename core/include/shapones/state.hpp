#ifndef SHAPONES_STATE_HPP
#define SHAPONES_STATE_HPP

#include "shapones/common.hpp"

namespace nes {

static const char *STATE_FILE_EXT = "spn";

static constexpr uint32_t STATE_MAX_SLOTS = 16;
static constexpr uint64_t STATE_MARKER = 0x74617453736E7053ULL;  // "SpnsStat"
static constexpr uint64_t STATE_VERSION = 1;

static constexpr uint32_t STATE_SLOT_FLAG_USED = 0x00000001;

struct state_file_header_t {
  static constexpr uint32_t SIZE = 32;

  uint64_t marker;
  uint64_t version;
  uint32_t slot_size;

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u64(marker);
    writer.u64(version);
    writer.u32(slot_size);
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    marker = reader.u64();
    version = reader.u64();
    slot_size = reader.u32();
  }
};

struct state_slot_entry_t {
  static constexpr uint32_t SIZE = 32;
  static constexpr uint32_t NAME_LENGTH = 16;

  int index;
  uint32_t flags;
  uint32_t play_time_sec;
  char name[NAME_LENGTH];

  bool is_used() const { return (flags & STATE_SLOT_FLAG_USED) != 0; }

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u32(flags);
    writer.u32(play_time_sec);
    for (int i = 0; i < NAME_LENGTH; i++) {
      writer.u8(name[i]);
    }
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    flags = reader.u32();
    play_time_sec = reader.u32();
    for (int i = 0; i < NAME_LENGTH; i++) {
      name[i] = (char)reader.u8();
    }
  }
};

using enum_state_slot_cb_t = bool (*)(const state_slot_entry_t &entry);

uint32_t get_state_slot_size();
result_t enum_state_slots(const char *path, enum_state_slot_cb_t callback);

result_t save_state(const char *path, int slot);
result_t load_state(const char *path, int slot);

}  // namespace nes

#endif