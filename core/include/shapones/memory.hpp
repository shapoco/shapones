#ifndef SHAPONES_MEMORY_HPP
#define SHAPONES_MEMORY_HPP

#include "shapones/common.hpp"

namespace nes::memory {

static constexpr int PRGROM_PAGE_SIZE = 16384;
static constexpr int CHRROM_PAGE_SIZE = 8192;

static constexpr int PRGROM_BLOCK_ADDR_BITS = 13;
static constexpr int PRGROM_BLOCK_SIZE = 1 << PRGROM_BLOCK_ADDR_BITS;
static constexpr int PRGROM_REMAP_TABLE_SIZE = PRGROM_RANGE / PRGROM_BLOCK_SIZE;

static constexpr int CHRROM_BLOCK_ADDR_BITS = 10;
static constexpr int CHRROM_BLOCK_SIZE = 1 << CHRROM_BLOCK_ADDR_BITS;
static constexpr int CHRROM_REMAP_TABLE_SIZE = CHRROM_RANGE / CHRROM_BLOCK_SIZE;

static constexpr addr_t CHRROM_BASE = 0x0000;
static constexpr addr_t PRGRAM_BASE = 0x6000;
static constexpr addr_t PRGROM_BASE = 0x8000;

static constexpr uint16_t expfwd(uint8_t val) {
  uint_fast16_t tmp = val;
  tmp = ((tmp & 0x00f0) << 4) | (tmp & 0x000f);
  tmp = ((tmp & 0x0c0c) << 2) | (tmp & 0x0303);
  tmp = ((tmp & 0x2222) << 1) | (tmp & 0x1111);
  return tmp;
}

static constexpr uint16_t exprev(uint8_t val) {
  uint_fast16_t tmp = val;
  tmp = ((tmp & 0x000f) << 12) | (tmp & 0x00f0);
  tmp = ((tmp & 0xc0c0) >> 6) | (tmp & 0x3030);
  tmp = ((tmp & 0x1111) << 3) | (tmp & 0x2222);
  return tmp;
}

// clang-format off
static const uint16_t EXPAND_FWD_TABLE[] = {
    expfwd(0x00), expfwd(0x01), expfwd(0x02), expfwd(0x03), expfwd(0x04), expfwd(0x05), expfwd(0x06), expfwd(0x07), expfwd(0x08), expfwd(0x09), expfwd(0x0A), expfwd(0x0B), expfwd(0x0C), expfwd(0x0D), expfwd(0x0E), expfwd(0x0F),
    expfwd(0x10), expfwd(0x11), expfwd(0x12), expfwd(0x13), expfwd(0x14), expfwd(0x15), expfwd(0x16), expfwd(0x17), expfwd(0x18), expfwd(0x19), expfwd(0x1A), expfwd(0x1B), expfwd(0x1C), expfwd(0x1D), expfwd(0x1E), expfwd(0x1F),
    expfwd(0x20), expfwd(0x21), expfwd(0x22), expfwd(0x23), expfwd(0x24), expfwd(0x25), expfwd(0x26), expfwd(0x27), expfwd(0x28), expfwd(0x29), expfwd(0x2A), expfwd(0x2B), expfwd(0x2C), expfwd(0x2D), expfwd(0x2E), expfwd(0x2F),
    expfwd(0x30), expfwd(0x31), expfwd(0x32), expfwd(0x33), expfwd(0x34), expfwd(0x35), expfwd(0x36), expfwd(0x37), expfwd(0x38), expfwd(0x39), expfwd(0x3A), expfwd(0x3B), expfwd(0x3C), expfwd(0x3D), expfwd(0x3E), expfwd(0x3F),
    expfwd(0x40), expfwd(0x41), expfwd(0x42), expfwd(0x43), expfwd(0x44), expfwd(0x45), expfwd(0x46), expfwd(0x47), expfwd(0x48), expfwd(0x49), expfwd(0x4A), expfwd(0x4B), expfwd(0x4C), expfwd(0x4D), expfwd(0x4E), expfwd(0x4F),
    expfwd(0x50), expfwd(0x51), expfwd(0x52), expfwd(0x53), expfwd(0x54), expfwd(0x55), expfwd(0x56), expfwd(0x57), expfwd(0x58), expfwd(0x59), expfwd(0x5A), expfwd(0x5B), expfwd(0x5C), expfwd(0x5D), expfwd(0x5E), expfwd(0x5F),
    expfwd(0x60), expfwd(0x61), expfwd(0x62), expfwd(0x63), expfwd(0x64), expfwd(0x65), expfwd(0x66), expfwd(0x67), expfwd(0x68), expfwd(0x69), expfwd(0x6A), expfwd(0x6B), expfwd(0x6C), expfwd(0x6D), expfwd(0x6E), expfwd(0x6F),
    expfwd(0x70), expfwd(0x71), expfwd(0x72), expfwd(0x73), expfwd(0x74), expfwd(0x75), expfwd(0x76), expfwd(0x77), expfwd(0x78), expfwd(0x79), expfwd(0x7A), expfwd(0x7B), expfwd(0x7C), expfwd(0x7D), expfwd(0x7E), expfwd(0x7F),
    expfwd(0x80), expfwd(0x81), expfwd(0x82), expfwd(0x83), expfwd(0x84), expfwd(0x85), expfwd(0x86), expfwd(0x87), expfwd(0x88), expfwd(0x89), expfwd(0x8A), expfwd(0x8B), expfwd(0x8C), expfwd(0x8D), expfwd(0x8E), expfwd(0x8F),
    expfwd(0x90), expfwd(0x91), expfwd(0x92), expfwd(0x93), expfwd(0x94), expfwd(0x95), expfwd(0x96), expfwd(0x97), expfwd(0x98), expfwd(0x99), expfwd(0x9A), expfwd(0x9B), expfwd(0x9C), expfwd(0x9D), expfwd(0x9E), expfwd(0x9F),
    expfwd(0xA0), expfwd(0xA1), expfwd(0xA2), expfwd(0xA3), expfwd(0xA4), expfwd(0xA5), expfwd(0xA6), expfwd(0xA7), expfwd(0xA8), expfwd(0xA9), expfwd(0xAA), expfwd(0xAB), expfwd(0xAC), expfwd(0xAD), expfwd(0xAE), expfwd(0xAF),
    expfwd(0xB0), expfwd(0xB1), expfwd(0xB2), expfwd(0xB3), expfwd(0xB4), expfwd(0xB5), expfwd(0xB6), expfwd(0xB7), expfwd(0xB8), expfwd(0xB9), expfwd(0xBA), expfwd(0xBB), expfwd(0xBC), expfwd(0xBD), expfwd(0xBE), expfwd(0xBF),
    expfwd(0xC0), expfwd(0xC1), expfwd(0xC2), expfwd(0xC3), expfwd(0xC4), expfwd(0xC5), expfwd(0xC6), expfwd(0xC7), expfwd(0xC8), expfwd(0xC9), expfwd(0xCA), expfwd(0xCB), expfwd(0xCC), expfwd(0xCD), expfwd(0xCE), expfwd(0xCF),
    expfwd(0xD0), expfwd(0xD1), expfwd(0xD2), expfwd(0xD3), expfwd(0xD4), expfwd(0xD5), expfwd(0xD6), expfwd(0xD7), expfwd(0xD8), expfwd(0xD9), expfwd(0xDA), expfwd(0xDB), expfwd(0xDC), expfwd(0xDD), expfwd(0xDE), expfwd(0xDF),
    expfwd(0xE0), expfwd(0xE1), expfwd(0xE2), expfwd(0xE3), expfwd(0xE4), expfwd(0xE5), expfwd(0xE6), expfwd(0xE7), expfwd(0xE8), expfwd(0xE9), expfwd(0xEA), expfwd(0xEB), expfwd(0xEC), expfwd(0xED), expfwd(0xEE), expfwd(0xEF),
    expfwd(0xF0), expfwd(0xF1), expfwd(0xF2), expfwd(0xF3), expfwd(0xF4), expfwd(0xF5), expfwd(0xF6), expfwd(0xF7), expfwd(0xF8), expfwd(0xF9), expfwd(0xFA), expfwd(0xFB), expfwd(0xFC), expfwd(0xFD), expfwd(0xFE), expfwd(0xFF),
};

static const uint16_t EXPAND_REV_TABLE[] = {
    exprev(0x00), exprev(0x01), exprev(0x02), exprev(0x03), exprev(0x04), exprev(0x05), exprev(0x06), exprev(0x07), exprev(0x08), exprev(0x09), exprev(0x0A), exprev(0x0B), exprev(0x0C), exprev(0x0D), exprev(0x0E), exprev(0x0F),
    exprev(0x10), exprev(0x11), exprev(0x12), exprev(0x13), exprev(0x14), exprev(0x15), exprev(0x16), exprev(0x17), exprev(0x18), exprev(0x19), exprev(0x1A), exprev(0x1B), exprev(0x1C), exprev(0x1D), exprev(0x1E), exprev(0x1F),
    exprev(0x20), exprev(0x21), exprev(0x22), exprev(0x23), exprev(0x24), exprev(0x25), exprev(0x26), exprev(0x27), exprev(0x28), exprev(0x29), exprev(0x2A), exprev(0x2B), exprev(0x2C), exprev(0x2D), exprev(0x2E), exprev(0x2F),
    exprev(0x30), exprev(0x31), exprev(0x32), exprev(0x33), exprev(0x34), exprev(0x35), exprev(0x36), exprev(0x37), exprev(0x38), exprev(0x39), exprev(0x3A), exprev(0x3B), exprev(0x3C), exprev(0x3D), exprev(0x3E), exprev(0x3F),
    exprev(0x40), exprev(0x41), exprev(0x42), exprev(0x43), exprev(0x44), exprev(0x45), exprev(0x46), exprev(0x47), exprev(0x48), exprev(0x49), exprev(0x4A), exprev(0x4B), exprev(0x4C), exprev(0x4D), exprev(0x4E), exprev(0x4F),
    exprev(0x50), exprev(0x51), exprev(0x52), exprev(0x53), exprev(0x54), exprev(0x55), exprev(0x56), exprev(0x57), exprev(0x58), exprev(0x59), exprev(0x5A), exprev(0x5B), exprev(0x5C), exprev(0x5D), exprev(0x5E), exprev(0x5F),
    exprev(0x60), exprev(0x61), exprev(0x62), exprev(0x63), exprev(0x64), exprev(0x65), exprev(0x66), exprev(0x67), exprev(0x68), exprev(0x69), exprev(0x6A), exprev(0x6B), exprev(0x6C), exprev(0x6D), exprev(0x6E), exprev(0x6F),
    exprev(0x70), exprev(0x71), exprev(0x72), exprev(0x73), exprev(0x74), exprev(0x75), exprev(0x76), exprev(0x77), exprev(0x78), exprev(0x79), exprev(0x7A), exprev(0x7B), exprev(0x7C), exprev(0x7D), exprev(0x7E), exprev(0x7F),
    exprev(0x80), exprev(0x81), exprev(0x82), exprev(0x83), exprev(0x84), exprev(0x85), exprev(0x86), exprev(0x87), exprev(0x88), exprev(0x89), exprev(0x8A), exprev(0x8B), exprev(0x8C), exprev(0x8D), exprev(0x8E), exprev(0x8F),
    exprev(0x90), exprev(0x91), exprev(0x92), exprev(0x93), exprev(0x94), exprev(0x95), exprev(0x96), exprev(0x97), exprev(0x98), exprev(0x99), exprev(0x9A), exprev(0x9B), exprev(0x9C), exprev(0x9D), exprev(0x9E), exprev(0x9F),
    exprev(0xA0), exprev(0xA1), exprev(0xA2), exprev(0xA3), exprev(0xA4), exprev(0xA5), exprev(0xA6), exprev(0xA7), exprev(0xA8), exprev(0xA9), exprev(0xAA), exprev(0xAB), exprev(0xAC), exprev(0xAD), exprev(0xAE), exprev(0xAF),
    exprev(0xB0), exprev(0xB1), exprev(0xB2), exprev(0xB3), exprev(0xB4), exprev(0xB5), exprev(0xB6), exprev(0xB7), exprev(0xB8), exprev(0xB9), exprev(0xBA), exprev(0xBB), exprev(0xBC), exprev(0xBD), exprev(0xBE), exprev(0xBF),
    exprev(0xC0), exprev(0xC1), exprev(0xC2), exprev(0xC3), exprev(0xC4), exprev(0xC5), exprev(0xC6), exprev(0xC7), exprev(0xC8), exprev(0xC9), exprev(0xCA), exprev(0xCB), exprev(0xCC), exprev(0xCD), exprev(0xCE), exprev(0xCF),
    exprev(0xD0), exprev(0xD1), exprev(0xD2), exprev(0xD3), exprev(0xD4), exprev(0xD5), exprev(0xD6), exprev(0xD7), exprev(0xD8), exprev(0xD9), exprev(0xDA), exprev(0xDB), exprev(0xDC), exprev(0xDD), exprev(0xDE), exprev(0xDF),
    exprev(0xE0), exprev(0xE1), exprev(0xE2), exprev(0xE3), exprev(0xE4), exprev(0xE5), exprev(0xE6), exprev(0xE7), exprev(0xE8), exprev(0xE9), exprev(0xEA), exprev(0xEB), exprev(0xEC), exprev(0xED), exprev(0xEE), exprev(0xEF),
    exprev(0xF0), exprev(0xF1), exprev(0xF2), exprev(0xF3), exprev(0xF4), exprev(0xF5), exprev(0xF6), exprev(0xF7), exprev(0xF8), exprev(0xF9), exprev(0xFA), exprev(0xFB), exprev(0xFC), exprev(0xFD), exprev(0xFE), exprev(0xFF),
};
// clang-format on

extern uint8_t wram[WRAM_SIZE];
extern uint8_t vram[VRAM_SIZE];
extern addr_t vram_addr_and;
extern addr_t vram_addr_or;

extern uint8_t *prgram;
extern uint8_t *chrram;
extern const uint8_t *prgrom;
extern const uint8_t *chrrom;

extern int prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
extern int chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

extern addr_t prgram_addr_mask;
extern uint32_t prgrom_phys_size;
extern uint32_t prgrom_phys_addr_mask;
extern uint32_t chrrom_phys_size;
extern uint32_t chrrom_phys_addr_mask;

result_t init();
void deinit();

result_t map_ines(const uint8_t *ines);
void unmap();

static SHAPONES_INLINE uint8_t vram_read(addr_t addr) {
  return vram[(addr & vram_addr_and) | vram_addr_or];
}

static SHAPONES_INLINE void vram_write(addr_t addr, uint8_t value) {
  vram[(addr & vram_addr_and) | vram_addr_or] = value;
}

static SHAPONES_INLINE void prgrom_remap(addr_t cpu_base, uint32_t phys_base,
                                         uint32_t size) {
  uint32_t cpu_block = (cpu_base - PRGROM_BASE) >> PRGROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = phys_base >> PRGROM_BLOCK_ADDR_BITS;
  uint32_t num_blocks = size >> PRGROM_BLOCK_ADDR_BITS;
  for (int i = 0; i < num_blocks; i++) {
    prgrom_remap_table[cpu_block + i] = phys_block + i;
  }
}

static SHAPONES_INLINE void chrrom_remap(addr_t ppu_base, uint32_t phys_base,
                                         uint32_t size) {
  uint32_t ppu_block = (ppu_base - CHRROM_BASE) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = phys_base >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t num_blocks = size >> CHRROM_BLOCK_ADDR_BITS;
  for (int i = 0; i < num_blocks; i++) {
    chrrom_remap_table[ppu_block + i] = phys_block + i;
  }
}

static SHAPONES_INLINE uint8_t prgrom_read(addr_t addr) {
  uint32_t cpu_block = (addr & (PRGROM_RANGE - 1)) >> PRGROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = prgrom_remap_table[cpu_block];
  uint32_t phys_addr =
      (phys_block << PRGROM_BLOCK_ADDR_BITS) + (addr & (PRGROM_BLOCK_SIZE - 1));
  return prgrom[phys_addr & prgrom_phys_addr_mask];
}

static SHAPONES_INLINE uint8_t prgram_read(addr_t addr) {
  return prgram[addr & prgram_addr_mask];
}

static SHAPONES_INLINE void prgram_write(addr_t addr, uint8_t value) {
  prgram[addr & prgram_addr_mask] = value;
}

static SHAPONES_INLINE uint8_t chrrom_read(addr_t addr) {
  uint32_t ppu_block = (addr & (CHRROM_RANGE - 1)) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = chrrom_remap_table[ppu_block];
  uint32_t phys_addr =
      (phys_block << CHRROM_BLOCK_ADDR_BITS) + (addr & (CHRROM_BLOCK_SIZE - 1));
  return chrrom[phys_addr & chrrom_phys_addr_mask];
}

static SHAPONES_INLINE void chrram_write(addr_t addr, uint8_t value) {
  uint32_t ppu_block = (addr & (CHRROM_RANGE - 1)) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = chrrom_remap_table[ppu_block];
  uint32_t phys_addr =
      (phys_block << CHRROM_BLOCK_ADDR_BITS) + (addr & (CHRROM_BLOCK_SIZE - 1));
  chrram[phys_addr & chrrom_phys_addr_mask] = value;
}

static SHAPONES_INLINE uint_fast16_t chrrom_read_double(addr_t addr,
                                                        bool reverse) {
  uint32_t ppu_block = (addr & (CHRROM_RANGE - 1)) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = chrrom_remap_table[ppu_block];
  uint32_t phys_addr =
      (phys_block << CHRROM_BLOCK_ADDR_BITS) + (addr & (CHRROM_BLOCK_SIZE - 1));

  uint8_t lo = chrrom[phys_addr & chrrom_phys_addr_mask];
  uint8_t hi = chrrom[(phys_addr & chrrom_phys_addr_mask) + 8];
  if (reverse) {
    return (EXPAND_FWD_TABLE[hi] << 1) | EXPAND_FWD_TABLE[lo];
  } else {
    return EXPAND_REV_TABLE[hi] | (EXPAND_REV_TABLE[lo] >> 1);
  }
}

void set_nametable_arrangement(NametableArrangement mode);

}  // namespace nes::memory

#endif
