#ifndef SHAPONES_MEMORY_HPP
#define SHAPONES_MEMORY_HPP

#include "shapones/shapones.hpp"

namespace nes::memory {

static constexpr int WRAM_SIZE = 0x800;
static constexpr int VRAM_SIZE = 0x1000;
static constexpr int PRGROM_SIZE = 0x8000;

struct MapperState {
    int id;
    int chr_bank;
    int chr_bank_mask;
};

extern const uint8_t *prgrom;
extern const uint8_t *chrrom;
extern uint16_t *chrrom_reordered0;
extern uint16_t *chrrom_reordered1;

extern uint8_t wram[WRAM_SIZE];
extern uint8_t vram[VRAM_SIZE];

extern addr_t prgrom_addr_mask;
extern addr_t vram_addr_mask;

extern MapperState mapper;

bool map_ines(const uint8_t *ines);

inline uint8_t prgrom_read(addr_t addr) {
    return prgrom[addr & prgrom_addr_mask];
}

inline uint8_t chrrom_read(addr_t addr) {
    addr += mapper.chr_bank * 0x2000;
    return chrrom[addr];
}

inline uint16_t chrrom_read_w(addr_t addr, bool invert) {
    addr += mapper.chr_bank * 0x1000;
    if (invert) {
        return chrrom_reordered1[addr];
    }
    else {
        return chrrom_reordered0[addr];
    }
}

inline uint8_t vram_read(addr_t addr) {
    return vram[addr & vram_addr_mask];
}

inline void vram_write(addr_t addr, uint8_t value) {
    vram[addr & vram_addr_mask] = value;
}

void ext_write(addr_t addr, uint8_t value);

}

#endif
