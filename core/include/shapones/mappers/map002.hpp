#ifndef SHAPONES_MAP002_HPP
#define SHAPONES_MAP002_HPP

#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"

namespace nes::mapper {

using namespace nes::memory;

// see: https://www.nesdev.org/wiki/UxROM
class Map002 : public Mapper {
 private:
  uint8_t bank = 0;

 public:
  static constexpr uint32_t STATE_SIZE = 8;

  Map002() : Mapper(2, "UxROM") {}

  result_t init() override { return result_t::SUCCESS; }

  result_t reset() override {
    bank = 0;
    prgrom_remap(0xC000, prgrom_phys_size - 0x4000, 0x4000);
    return result_t::SUCCESS;
  }

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0xffff) {
      bank = value & 0x0F;
      perform_remap();
    }
  }

  void perform_remap() { prgrom_remap(0x8000, bank * 0x4000, 0x4000); }

  uint32_t get_state_size() override { return STATE_SIZE; }

  result_t save_state(void *file_handle) override {
    uint8_t buff[STATE_SIZE];
    memset(buff, 0, sizeof(buff));
    buff[0] = bank;
    return nes::fs_write(file_handle, buff, STATE_SIZE);
  }

  result_t load_state(void *file_handle) override {
    uint8_t buff[STATE_SIZE];
    SHAPONES_TRY(nes::fs_read(file_handle, buff, STATE_SIZE));
    bank = buff[0];
    perform_remap();
    return result_t::SUCCESS;
  }
};

}  // namespace nes::mapper

#endif
