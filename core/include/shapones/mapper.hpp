#ifndef SHAPONES_MAPPER_HPP
#define SHAPONES_MAPPER_HPP

#include "shapones/common.hpp"
#include "shapones/cpu.hpp"
#include "shapones/ppu.hpp"

namespace nes::mapper {

static constexpr int PRGROM_PAGE_SIZE = 16384;
static constexpr int CHRROM_PAGE_SIZE = 8192;

static constexpr int CHRROM_REMAP_ADDR_BITS = 10;
static constexpr int CHRROM_REMAP_SIZE = 1 << CHRROM_REMAP_ADDR_BITS;
static constexpr int CHRROM_REMAP_TABLE_SIZE = CHRROM_RANGE / CHRROM_REMAP_SIZE;

static constexpr int PRGROM_REMAP_ADDR_BITS = 13;
static constexpr int PRGROM_REMAP_SIZE = 1 << PRGROM_REMAP_ADDR_BITS;
static constexpr int PRGROM_REMAP_TABLE_SIZE = PRGROM_RANGE / PRGROM_REMAP_SIZE;

extern int id;

extern const uint8_t *prgrom;
extern const uint8_t *chrrom;
extern uint16_t *chrrom_reordered0;
extern uint16_t *chrrom_reordered1;

extern uint8_t *prgram;

extern int prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
extern int chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

extern addr_t prgram_addr_mask;
extern uint32_t prgrom_phys_size;
extern uint32_t prgrom_phys_addr_mask;
extern uint32_t chrrom_phys_size;
extern uint32_t chrrom_phys_addr_mask;

static NES_ALWAYS_INLINE int get_id() { return id; }

static NES_ALWAYS_INLINE void prgrom_remap(addr_t cpu_base, uint32_t phys_base,
                                           uint32_t size) {
    int cpu_block = (cpu_base - cpu::PRGROM_BASE) / PRGROM_REMAP_SIZE;
    int phys_block = phys_base / PRGROM_REMAP_SIZE;
    int num_blocks = size / PRGROM_REMAP_SIZE;
    for (int i = 0; i < num_blocks; i++) {
        prgrom_remap_table[cpu_block + i] = phys_block + i;
    }
    // NES_PRINTF("PRGROM remap: CPU 0x%04x -> ROM 0x%05x (%d blocks)\n",
    //            (unsigned int)cpu_base, (unsigned int)rom_base, num_blocks);
}

static NES_ALWAYS_INLINE void chrrom_remap(addr_t ppu_base, uint32_t phys_base,
                                           uint32_t size) {
    int ppu_block = (ppu_base - ppu::CHRROM_BASE) / CHRROM_REMAP_SIZE;
    int phys_block = phys_base / CHRROM_REMAP_SIZE;
    int num_blocks = size / CHRROM_REMAP_SIZE;
    for (int i = 0; i < num_blocks; i++) {
        chrrom_remap_table[ppu_block + i] = phys_block + i;
    }
}

static NES_ALWAYS_INLINE uint8_t prgrom_read(addr_t addr) {
    int cpu_blk = (addr & (PRGROM_RANGE - 1)) / PRGROM_REMAP_SIZE;
    int phys_blk = prgrom_remap_table[cpu_blk];
    int phys_addr = phys_blk * PRGROM_REMAP_SIZE + (addr % PRGROM_REMAP_SIZE);
    return prgrom[phys_addr & prgrom_phys_addr_mask];
}

static NES_ALWAYS_INLINE uint8_t prgram_read(addr_t addr) {
    return prgram[addr & prgram_addr_mask];
}

static NES_ALWAYS_INLINE void prgram_write(addr_t addr, uint8_t value) {
    prgram[addr & prgram_addr_mask] = value;
}

static NES_ALWAYS_INLINE uint8_t chrrom_read(addr_t addr) {
    int ppu_blk = (addr & (CHRROM_RANGE - 1)) / CHRROM_REMAP_SIZE;
    int phys_blk = chrrom_remap_table[ppu_blk];
    int phys_addr = phys_blk * CHRROM_REMAP_SIZE + (addr % CHRROM_REMAP_SIZE);
    return chrrom[phys_addr & chrrom_phys_addr_mask];
}

static NES_ALWAYS_INLINE uint16_t chrrom_read_w(addr_t addr, bool invert) {
    int ppu_blk = (addr & (CHRROM_RANGE / 2 - 1)) / (CHRROM_REMAP_SIZE / 2);
    int phys_blk = chrrom_remap_table[ppu_blk];
    int phys_addr =
        phys_blk * (CHRROM_REMAP_SIZE / 2) + (addr % (CHRROM_REMAP_SIZE / 2));
    if (invert) {
        return chrrom_reordered1[phys_addr & (chrrom_phys_addr_mask >> 1)];
    } else {
        return chrrom_reordered0[phys_addr & (chrrom_phys_addr_mask >> 1)];
    }
}

void init(const uint8_t *ines);
void ext_write(addr_t addr, uint8_t value);

}  // namespace nes::mapper

#endif