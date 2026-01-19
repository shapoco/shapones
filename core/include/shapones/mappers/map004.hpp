#ifndef SHAPONES_MAP004_HPP
#define SHAPONES_MAP004_HPP

#include "shapones/interrupt.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"

namespace nes::mapper {

using namespace nes::memory;

class Map004 : public Mapper {
 private:
  uint8_t reg_sel;
  uint8_t map_reg[8];

  volatile bool irq_enable = false;
  volatile bool irq_reloading = false;
  volatile uint8_t irq_latch = 255;
  volatile uint8_t irq_counter = 0;

 public:
  Map004() : Mapper(4, "MMC3") {}

  void init() override { prgrom_remap(0xE000, prgrom_phys_size - 8192, 8192); }

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0x9FFF) {
      if ((addr & 0x0001) == 0) {
        reg_sel = value;
      } else {
        int ireg = reg_sel & 0x07;
        if (ireg <= 1) {
          value &= 0xfe;  // ignore LSB for 2KB bank
        }
        map_reg[ireg] = value;
      }

      constexpr int pbs = 8192;
      prgrom_remap(0xA000, map_reg[7] * pbs, pbs);
      if ((reg_sel & 0x40) == 0) {
        prgrom_remap(0x8000, map_reg[6] * pbs, pbs);
        prgrom_remap(0xC000, prgrom_phys_size - pbs * 2, pbs);
      } else {
        prgrom_remap(0x8000, prgrom_phys_size - pbs * 2, pbs);
        prgrom_remap(0xC000, map_reg[6] * pbs, pbs);
      }

      constexpr int cbs = 1024;
      if ((reg_sel & 0x80) == 0) {
        chrrom_remap(0x0000, map_reg[0] * cbs, cbs * 2);
        chrrom_remap(0x0800, map_reg[1] * cbs, cbs * 2);
        chrrom_remap(0x1000, map_reg[2] * cbs, cbs);
        chrrom_remap(0x1400, map_reg[3] * cbs, cbs);
        chrrom_remap(0x1800, map_reg[4] * cbs, cbs);
        chrrom_remap(0x1C00, map_reg[5] * cbs, cbs);
      } else {
        chrrom_remap(0x0000, map_reg[2] * cbs, cbs);
        chrrom_remap(0x0400, map_reg[3] * cbs, cbs);
        chrrom_remap(0x0800, map_reg[4] * cbs, cbs);
        chrrom_remap(0x0C00, map_reg[5] * cbs, cbs);
        chrrom_remap(0x1000, map_reg[0] * cbs, cbs * 2);
        chrrom_remap(0x1800, map_reg[1] * cbs, cbs * 2);
      }
    } else if (0xA000 <= addr && addr <= 0xBFFF) {
      if ((addr & 0x0001) == 0) {
        if ((value & 0x01) == 0) {
          set_nametable_arrangement(NametableArrangement::HORIZONTAL);
        } else {
          set_nametable_arrangement(NametableArrangement::VERTICAL);
        }
      } else {
        // PRG RAM protect
        // TODO
      }
    } else if (0xC000 <= addr && addr <= 0xDFFF) {
      if ((addr & 0x0001) == 0) {
        // IRQ latch
        irq_latch = value;
      } else {
        // IRQ reload
        irq_reloading = true;
      }
    } else if (0xE000 <= addr && addr <= 0xFFFF) {
      bool enable_old = irq_enable;
      bool enable_new = !!(addr & 0x0001);
      if (!enable_new && enable_old) {
        interrupt::deassert_irq(interrupt::Source::MMC3);
      }
      irq_enable = enable_new;
    }
  }

  bool hblank(const nes::ppu::Registers &reg, int y) override {
    bool irq = false;
    if (reg.mask.bg_enable && reg.mask.sprite_enable) {
      uint8_t irq_counter_before = irq_counter;
      if (irq_counter == 0 || irq_reloading) {
        irq_reloading = false;
        irq_counter = irq_latch;
      } else {
        irq_counter--;
      }
      if (irq_enable && irq_counter == 0) {
        interrupt::assert_irq(interrupt::Source::MMC3);
        irq = true;
      }
    }
    return irq;
  }
};

}  // namespace nes::mapper

#endif
