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
  uint8_t nametable_arrangement;
  uint8_t ram_protection;

  volatile bool irq_enable = false;
  volatile bool irq_reloading = false;
  volatile uint8_t irq_latch = 255;

  uint8_t irq_counter = 0;

 public:
  static constexpr uint32_t STATE_SIZE = 32;

  Map004() : Mapper(4, "MMC3") {}

  result_t init() override {
    prgrom_remap(0xE000, prgrom_phys_size - 8192, 8192);
    return result_t::SUCCESS;
  }

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
      perform_remap();
    } else if (0xA000 <= addr && addr <= 0xBFFF) {
      if ((addr & 0x0001) == 0) {
        nametable_arrangement = value;
        perform_nametable_arrangement();
      } else {
        // PRG RAM protect
        ram_protection = value;
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
        interrupt::deassert_irq(interrupt::source_t::MAPPER);
      }
      irq_enable = enable_new;
    }
  }

  bool hblank(const nes::ppu::registers_t &reg, int y) override {
    bool irq = false;
    if (reg.mask.bg_enable && reg.mask.sprite_enable) {
      if (irq_counter == 0 || irq_reloading) {
        irq_reloading = false;
        irq_counter = irq_latch;
      } else {
        irq_counter--;
      }
      if (irq_enable && irq_counter == 0) {
        interrupt::assert_irq(interrupt::source_t::MAPPER);
        irq = true;
      }
    }
    return irq;
  }

  void perform_remap() {
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
  }

  void perform_nametable_arrangement() {
    if (nametable_arrangement & 0x01) {
      set_nametable_arrangement(nametable_arrangement_t::VERTICAL);
    } else {
      set_nametable_arrangement(nametable_arrangement_t::HORIZONTAL);
    }
  }

  uint32_t get_state_size() override { return STATE_SIZE; }

  result_t save_state(void *file_handle) override {
    uint8_t buffer[STATE_SIZE];
    memset(buffer, 0, sizeof(buffer));
    uint32_t offset = 0;
    buffer[offset++] = reg_sel;
    for (int i = 0; i < 8; i++) {
      buffer[offset++] = map_reg[i];
    }
    buffer[offset++] = nametable_arrangement;
    buffer[offset++] = ram_protection;
    buffer[offset++] = irq_enable ? 1 : 0;
    buffer[offset++] = irq_reloading ? 1 : 0;
    buffer[offset++] = irq_latch;
    buffer[offset++] = irq_counter;
    return nes::fs_write(file_handle, buffer, STATE_SIZE);
  }

  result_t load_state(void *file_handle) override {
    uint8_t buffer[STATE_SIZE];
    SHAPONES_TRY(nes::fs_read(file_handle, buffer, STATE_SIZE));
    uint32_t offset = 0;
    reg_sel = buffer[offset++];
    for (int i = 0; i < 8; i++) {
      map_reg[i] = buffer[offset++];
    }
    nametable_arrangement = buffer[offset++];
    ram_protection = buffer[offset++];
    irq_enable = buffer[offset++] != 0;
    irq_reloading = buffer[offset++] != 0;
    irq_latch = buffer[offset++];
    irq_counter = buffer[offset++];
    perform_remap();
    perform_nametable_arrangement();
    return result_t::SUCCESS;
  }
};

}  // namespace nes::mapper

#endif
