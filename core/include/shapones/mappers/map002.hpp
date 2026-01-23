#ifndef SHAPONES_MAP002_HPP
#define SHAPONES_MAP002_HPP

#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"

namespace nes::mapper {

using namespace nes::memory;

// see: https://www.nesdev.org/wiki/UxROM
class Map002 : public Mapper {
 public:
  Map002() : Mapper(2, "UxROM") {}

  result_t init() override {
    prgrom_remap(0xC000, prgrom_phys_size - 0x4000, 0x4000);
    return result_t::SUCCESS;
  }

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0xffff) {
      uint32_t bank = value & 0x0F;
      prgrom_remap(0x8000, bank * 0x4000, 0x4000);
    }
  }
};

}  // namespace nes::mapper

#endif
