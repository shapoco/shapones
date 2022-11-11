#include "shapones/shapones.hpp"

namespace nes::memory {

const uint8_t *prgrom;
const uint8_t *chrrom;
uint16_t *chrrom_reordered0;
uint16_t *chrrom_reordered1;

uint8_t wram[WRAM_SIZE];
uint8_t vram[VRAM_SIZE];

addr_t prgrom_addr_mask = PRGROM_SIZE - 1;
addr_t vram_addr_mask = VRAM_SIZE - 1;

MapperState mapper;

bool map_ines(const uint8_t *ines) {
    // iNES file format
    // https://www.nesdev.org/wiki/INES

    // marker
    if (ines[0] != 0x4e && ines[1] != 0x45 && ines[2] != 0x53 && ines[3] != 0x1a) {
        return false;
    }

    // Size of PRG ROM in 16 KB units
    int num_prg_rom_pages = ines[4];
    switch (num_prg_rom_pages) {
    case 1: prgrom_addr_mask = 0x3fff; break;
    case 2: prgrom_addr_mask = 0x7fff; break;
    default:
        NES_ERRORF("Unsupported number of PRGROM pages: %d\n", num_prg_rom_pages);
    }
        
    NES_PRINTF("Number of PRGROM pages = %d (%dkB)\n", num_prg_rom_pages, num_prg_rom_pages * 16);

    // Size of CHR ROM in 8 KB units
    int num_chr_rom_pages = ines[5];
    NES_PRINTF("Number of CHRROM pages = %d (%dkB)\n", num_chr_rom_pages, num_chr_rom_pages * 8);

    uint8_t flags6 = ines[6];
    NES_PRINTF("Flags6 = 0x%02x\n", (int)flags6);
    // 512-byte trainer at $7000-$71FF (stored before PRG data)
    bool has_trainer = (flags6 & 0x2) != 0;

    if (flags6 & 0x8) {
        // Ignore mirroring control or above mirroring bit; instead provide four-screen VRAM
        vram_addr_mask = VRAM_SIZE - 1;
        NES_PRINTF("Mirroring = NONE (vram_addr_mask=%x)\n", vram_addr_mask);
    }
    else if (flags6 & 0x1) {
        vram_addr_mask = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
        NES_PRINTF("Mirroring = VERTICAL (vram_addr_mask=%x)\n", vram_addr_mask);
    }
    else {
        vram_addr_mask = (VRAM_SIZE - 1) - (VRAM_SIZE / 4);
        NES_PRINTF("Mirroring = HORIZONTAL (vram_addr_mask=%x)\n", vram_addr_mask);
    }

    uint8_t flags7 = ines[7];

    mapper.id = (flags7 & 0xf0) | ((flags6 >> 4) & 0xf);
    NES_PRINTF("Mapper No.%d\n", mapper.id);
    switch(mapper.id) {
    case 0:
        mapper.chr_bank_mask = 0;
        break;
    case 3:
        mapper.chr_bank_mask = num_chr_rom_pages - 1;
        break;
    default:
        NES_ERRORF("Unsupported Mapper Number\n");
        break;
    }
    mapper.chr_bank = 0;
    NES_PRINTF("  CHRROM bank mask = 0x%x\n", mapper.chr_bank_mask);

    int start_of_prg_rom = 0x10;
    if (has_trainer) start_of_prg_rom += 0x200;
    prgrom = ines + start_of_prg_rom;

    int start_of_chr_rom = start_of_prg_rom + num_prg_rom_pages * 0x4000;
    chrrom = ines + start_of_chr_rom;

    // reorder CHRROM bits for fast access
    int num_chars = num_chr_rom_pages * 0x2000 / 16;
    chrrom_reordered0 = new uint16_t[num_chars * 8];
    chrrom_reordered1 = new uint16_t[num_chars * 8];
    for (int ic = 0; ic < num_chars; ic++) {
        for (int iy = 0; iy < 8; iy++) {
            uint8_t chr0 = chrrom[ic * 16 + iy];
            uint8_t chr1 = chrrom[ic * 16 + iy + 8];

            {
                uint16_t chr = 0;
                chr  = (uint16_t)(chr1 & 0x01) << 15;
                chr |= (uint16_t)(chr0 & 0x01) << 14;
                chr |= (uint16_t)(chr1 & 0x02) << 12;
                chr |= (uint16_t)(chr0 & 0x02) << 11;
                chr |= (uint16_t)(chr1 & 0x04) <<  9;
                chr |= (uint16_t)(chr0 & 0x04) <<  8;
                chr |= (uint16_t)(chr1 & 0x08) <<  6;
                chr |= (uint16_t)(chr0 & 0x08) <<  5;
                chr |= (uint16_t)(chr1 & 0x10) <<  3;
                chr |= (uint16_t)(chr0 & 0x10) <<  2;
                chr |= (uint16_t)(chr1 & 0x20);
                chr |= (uint16_t)(chr0 & 0x20) >>  1;
                chr |= (uint16_t)(chr1 & 0x40) >>  3;
                chr |= (uint16_t)(chr0 & 0x40) >>  4;
                chr |= (uint16_t)(chr1 & 0x80) >>  6;
                chr |= (uint16_t)(chr0 & 0x80) >>  7;
                chrrom_reordered0[ic * 8 + iy] = chr;
            }
            
            {
                uint16_t chr = 0;
                chr  = (uint16_t)(chr1 & 0x80) <<  8;
                chr |= (uint16_t)(chr0 & 0x80) <<  7;
                chr |= (uint16_t)(chr1 & 0x40) <<  7;
                chr |= (uint16_t)(chr0 & 0x40) <<  6;
                chr |= (uint16_t)(chr1 & 0x20) <<  6;
                chr |= (uint16_t)(chr0 & 0x20) <<  5;
                chr |= (uint16_t)(chr1 & 0x10) <<  5;
                chr |= (uint16_t)(chr0 & 0x10) <<  4;
                chr |= (uint16_t)(chr1 & 0x08) <<  4;
                chr |= (uint16_t)(chr0 & 0x08) <<  3;
                chr |= (uint16_t)(chr1 & 0x04) <<  3;
                chr |= (uint16_t)(chr0 & 0x04) <<  2;
                chr |= (uint16_t)(chr1 & 0x02) <<  2;
                chr |= (uint16_t)(chr0 & 0x02) <<  1;
                chr |= (uint16_t)(chr1 & 0x01) <<  1;
                chr |= (uint16_t)(chr0 & 0x01);
                chrrom_reordered1[ic * 8 + iy] = chr;
            }
        }
    }

    return true;
}

void ext_write(addr_t addr, uint8_t value) {
    switch(mapper.id) {
    case 3:
        if (0x8000 <= addr && addr <= 0xffff) {
            mapper.chr_bank = value & mapper.chr_bank_mask;
        }
    }
}

}
