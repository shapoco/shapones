#include "shapones/memory.hpp"
#include "shapones/mapper.hpp"
#include "shapones/ppu.hpp"

namespace nes::memory {

uint8_t wram[WRAM_SIZE];
uint8_t vram[VRAM_SIZE];
addr_t vram_addr_mask;

bool map_ines(const uint8_t *ines) {
    // iNES file format
    // https://www.nesdev.org/wiki/INES

    // marker
    if (ines[0] != 0x4e && ines[1] != 0x45 && ines[2] != 0x53 &&
        ines[3] != 0x1a) {
        return false;
    }

    uint8_t flags6 = ines[6];
    SHAPONES_PRINTF("Flags6 = 0x%02x\n", (int)flags6);

    bool four_screen = (flags6 & 0x8) != 0;
    bool vertical = (flags6 & 0x1) != 0;
    if (four_screen) {
        SHAPONES_PRINTF("Nametable mirroring: Four-screen\n");
    } else if (vertical) {
        SHAPONES_PRINTF("Nametable mirroring: Vertical\n");
    } else {
        SHAPONES_PRINTF("Nametable mirroring: Horizontal\n");
    }
    set_nametable_mirroring(four_screen, vertical);

    mapper::init(ines);

    return true;
}

void set_nametable_mirroring(bool four_screen, bool vertical) {
    if (four_screen) {
        vram_addr_mask = VRAM_SIZE - 1;
    } else if (vertical) {
        vram_addr_mask = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
    } else {
        vram_addr_mask = (VRAM_SIZE - 1) - (VRAM_SIZE / 4);
    }
}

}  // namespace nes::memory
