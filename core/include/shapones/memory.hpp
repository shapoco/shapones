#ifndef SHAPONES_MEMORY_HPP
#define SHAPONES_MEMORY_HPP

#include "shapones/common.hpp"

namespace nes::memory {

extern uint8_t wram[WRAM_SIZE];
extern uint8_t vram[VRAM_SIZE];
extern addr_t vram_addr_mask;

bool map_ines(const uint8_t *ines);

static NES_ALWAYS_INLINE uint8_t vram_read(addr_t addr) {
    return vram[addr & vram_addr_mask];
}

static NES_ALWAYS_INLINE void vram_write(addr_t addr, uint8_t value) {
    vram[addr & vram_addr_mask] = value;
}

void set_nametable_mirroring(bool four_screen, bool vertical);

}  // namespace nes::memory

#endif
