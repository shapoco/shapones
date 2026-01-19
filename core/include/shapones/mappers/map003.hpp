#ifndef SHAPONES_MAP003_HPP
#define SHAPONES_MAP003_HPP

#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"

namespace nes::mapper {

using namespace nes::memory;

class Map003 : public Mapper {
 public:
  Map003() : Mapper(3, "CNROM") {}

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0xffff) {
      chrrom_remap(0x0000, (value & 0x3) * 0x2000, 0x2000);
    }
  }
};

}  // namespace nes::mapper

#endif
