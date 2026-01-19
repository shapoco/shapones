#ifndef SHAPONES_MAP001_HPP
#define SHAPONES_MAP001_HPP

#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"

namespace nes::mapper {

using namespace nes::memory;

class Map001 : public Mapper {
 private:
  uint8_t shift_reg = 0b10000;
  uint8_t ctrl_reg = 0;
  uint8_t chr_bank0 = 0;
  uint8_t chr_bank1 = 0;
  uint8_t prg_bank = 0;

 public:
  Map001() : Mapper(1, "MMC1") {}

  void write(addr_t addr, uint8_t value) override {
    bool remap = false;
    if (value & 0x80) {
      shift_reg = 0b10000;
      ctrl_reg |= 0x0C;
      remap = true;
    } else {
      bool shift = !(shift_reg & 0x01);
      uint8_t val = (shift_reg >> 1) | ((value & 0x01) << 4);
      if (shift) {
        shift_reg = val;
      } else {
        shift_reg = 0b10000;
        switch (addr & 0x6000) {
          default:
          case 0x0000: ctrl_reg = val; break;
          case 0x2000: chr_bank0 = val; break;
          case 0x4000: chr_bank1 = val; break;
          case 0x6000: prg_bank = val; break;
        }
        remap = true;
      }
    }

    if (remap) {
      switch (ctrl_reg & 0x03) {
        default:
        case 0:
          set_nametable_arrangement(NametableArrangement::SINGLE_LOWER);
          break;
        case 1:
          set_nametable_arrangement(NametableArrangement::SINGLE_UPPER);
          break;
        case 2:
          set_nametable_arrangement(NametableArrangement::HORIZONTAL);
          break;
        case 3: set_nametable_arrangement(NametableArrangement::VERTICAL); break;
      }

      switch (ctrl_reg & 0x0C) {
        default:
        case 0x00:
        case 0x04: prgrom_remap(0x8000, (prg_bank & 0x0E) << 14, 0x8000); break;
        case 0x08:
          prgrom_remap(0x8000, 0, 0x4000);
          prgrom_remap(0xC000, (prg_bank & 0x0F) << 14, 0x4000);
          break;
        case 0x0C:
          prgrom_remap(0x8000, (prg_bank & 0x0F) << 14, 0x4000);
          prgrom_remap(0xC000, prgrom_phys_size - 0x4000, 0x4000);
          break;
      }

      if ((ctrl_reg & 0x10) == 0) {
        chrrom_remap(0x0000, (chr_bank0 & 0x1E) << 12, 0x2000);
      } else {
        chrrom_remap(0x0000, (chr_bank0 & 0x1F) << 12, 0x1000);
        chrrom_remap(0x1000, (chr_bank1 & 0x1F) << 12, 0x1000);
      }
    }
  }
};

}  // namespace nes::mapper

#endif
