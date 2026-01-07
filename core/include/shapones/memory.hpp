#ifndef SHAPONES_MEMORY_HPP
#define SHAPONES_MEMORY_HPP

#include "shapones/shapones.hpp"

namespace nes::memory {

static constexpr int PRGROM_PAGE_SIZE = 16384;
static constexpr int CHRROM_PAGE_SIZE = 8192;

static constexpr int CHRROM_REMAP_ADDR_BITS = 10;
static constexpr int CHRROM_REMAP_SIZE = 1 << CHRROM_REMAP_ADDR_BITS;
static constexpr int CHRROM_REMAP_TABLE_SIZE = CHRROM_RANGE / CHRROM_REMAP_SIZE;

static constexpr int PRGROM_REMAP_ADDR_BITS = 13;
static constexpr int PRGROM_REMAP_SIZE = 1 << PRGROM_REMAP_ADDR_BITS;
static constexpr int PRGROM_REMAP_TABLE_SIZE = PRGROM_RANGE / PRGROM_REMAP_SIZE;

struct MapperState {
    int id;
    int prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
    int chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];
    uint8_t mmc3_reg_sel;
    uint8_t mmc3_map_reg[8];
    uint8_t mmc3_irq_latch;
    uint8_t mmc3_irq_counter;
    bool mmc3_irq_enable;
};

extern const uint8_t *prgrom;
extern const uint8_t *chrrom;
extern uint16_t *chrrom_reordered0;
extern uint16_t *chrrom_reordered1;

extern uint8_t wram[WRAM_SIZE];
extern uint8_t vram[VRAM_SIZE];
extern uint8_t *prgram;

extern addr_t prgram_addr_mask;
extern uint32_t prgrom_phys_size;
extern uint32_t prgrom_phys_addr_mask;
extern uint32_t chrrom_phys_size;
extern uint32_t chrrom_phys_addr_mask;
extern addr_t vram_addr_mask;

extern MapperState mapper;

bool map_ines(const uint8_t *ines);

NES_ALWAYS_INLINE uint8_t prgrom_read(addr_t addr) {
    int cpu_blk = (addr & (PRGROM_RANGE - 1)) / PRGROM_REMAP_SIZE;
    int phys_blk = mapper.prgrom_remap_table[cpu_blk];
    int phys_addr = phys_blk * PRGROM_REMAP_SIZE + (addr % PRGROM_REMAP_SIZE);
    return prgrom[phys_addr & prgrom_phys_addr_mask];
}

NES_ALWAYS_INLINE uint8_t prgram_read(addr_t addr) {
    return prgram[addr & prgram_addr_mask];
}

NES_ALWAYS_INLINE void prgram_write(addr_t addr, uint8_t value) {
    prgram[addr & prgram_addr_mask] = value;
}

NES_ALWAYS_INLINE uint8_t chrrom_read(addr_t addr) {
    int ppu_blk = (addr & (CHRROM_RANGE - 1)) / CHRROM_REMAP_SIZE;
    int phys_blk = mapper.chrrom_remap_table[ppu_blk];
    int phys_addr = phys_blk * CHRROM_REMAP_SIZE + (addr % CHRROM_REMAP_SIZE);
    return chrrom[phys_addr & chrrom_phys_addr_mask];
}

NES_ALWAYS_INLINE uint16_t chrrom_read_w(addr_t addr, bool invert) {
    int ppu_blk = (addr & (CHRROM_RANGE / 2 - 1)) / (CHRROM_REMAP_SIZE / 2);
    int phys_blk = mapper.chrrom_remap_table[ppu_blk];
    int phys_addr =
        phys_blk * (CHRROM_REMAP_SIZE / 2) + (addr % (CHRROM_REMAP_SIZE / 2));
    if (invert) {
        return chrrom_reordered1[phys_addr & (chrrom_phys_addr_mask >> 1)];
    } else {
        return chrrom_reordered0[phys_addr & (chrrom_phys_addr_mask >> 1)];
    }
}

NES_ALWAYS_INLINE uint8_t vram_read(addr_t addr) {
    return vram[addr & vram_addr_mask];
}

NES_ALWAYS_INLINE void vram_write(addr_t addr, uint8_t value) {
    vram[addr & vram_addr_mask] = value;
}

void ext_write(addr_t addr, uint8_t value);

}  // namespace nes::memory

#endif
