#include "shapones/shapones.hpp"

namespace nes::memory {

const uint8_t *prgrom;
const uint8_t *chrrom;
uint16_t *chrrom_reordered0;
uint16_t *chrrom_reordered1;

uint8_t wram[WRAM_SIZE];
uint8_t vram[VRAM_SIZE];
uint8_t *prgram = nullptr;

addr_t prgram_addr_mask;
uint32_t prgrom_phys_size;
uint32_t prgrom_phys_addr_mask;
uint32_t chrrom_phys_size;
uint32_t chrrom_phys_addr_mask;
addr_t prgrom_cpu_addr_mask = PRGROM_RANGE - 1;
addr_t vram_addr_mask = VRAM_SIZE - 1;

MapperState mapper;

NES_ALWAYS_INLINE void prgrom_remap(addr_t cpu_base, uint32_t rom_base,
                                    uint32_t size) {
    int cpu_blk = (cpu_base - cpu::PRGROM_BASE) / PRGROM_REMAP_SIZE;
    int phys_blk = rom_base / PRGROM_REMAP_SIZE;
    int num_banks = size / PRGROM_REMAP_SIZE;
    for (int i = 0; i < num_banks; i++) {
        mapper.prgrom_remap_table[cpu_blk + i] = phys_blk + i;
    }
}

NES_ALWAYS_INLINE void chrrom_remap(addr_t ppu_base, uint32_t rom_base,
                                    uint32_t size) {
    int ppu_blk = (ppu_base - ppu::CHRROM_BASE) / CHRROM_REMAP_SIZE;
    int phys_blk = rom_base / CHRROM_REMAP_SIZE;
    int num_banks = size / CHRROM_REMAP_SIZE;
    for (int i = 0; i < num_banks; i++) {
        mapper.chrrom_remap_table[ppu_blk + i] = phys_blk + i;
    }
}

static void set_nametable_mirroring(bool four_screen, bool vertical);

static void mmc3_ext_write(addr_t addr, uint8_t value);

bool map_ines(const uint8_t *ines) {
    // iNES file format
    // https://www.nesdev.org/wiki/INES

    // marker
    if (ines[0] != 0x4e && ines[1] != 0x45 && ines[2] != 0x53 &&
        ines[3] != 0x1a) {
        return false;
    }

    // Size of PRG ROM in 16 KB units
    int num_prg_rom_pages = ines[4];
    prgrom_phys_size = num_prg_rom_pages * PRGROM_PAGE_SIZE;
    prgrom_phys_addr_mask = prgrom_phys_size - 1;
    if (num_prg_rom_pages <= 1) {
        prgrom_cpu_addr_mask = 0x3fff;
    } else {
        prgrom_cpu_addr_mask = PRGROM_RANGE - 1;
    }
    NES_PRINTF("Number of PRGROM pages = %d (%dkB)\n", num_prg_rom_pages,
               prgrom_phys_size / 1024);

    // Size of CHR ROM in 8 KB units
    int num_chr_rom_pages = ines[5];
    chrrom_phys_size = num_chr_rom_pages * CHRROM_PAGE_SIZE;
    chrrom_phys_addr_mask = chrrom_phys_size - 1;
    NES_PRINTF("Number of CHRROM pages = %d (%dkB)\n", num_chr_rom_pages,
               chrrom_phys_size / 1024);

    uint8_t flags6 = ines[6];
    NES_PRINTF("Flags6 = 0x%02x\n", (int)flags6);
    // 512-byte trainer at $7000-$71FF (stored before PRG data)
    bool has_trainer = (flags6 & 0x2) != 0;

    bool four_screen = (flags6 & 0x8) != 0;
    bool vertical = (flags6 & 0x1) != 0;
    if (four_screen) {
        NES_PRINTF("Nametable mirroring: Four-screen\n");
    } else if (vertical) {
        NES_PRINTF("Nametable mirroring: Vertical\n");
    } else {
        NES_PRINTF("Nametable mirroring: Horizontal\n");
    }
    set_nametable_mirroring(four_screen, vertical);

    uint8_t flags7 = ines[7];

    mapper.id = (flags7 & 0xf0) | ((flags6 >> 4) & 0xf);
    NES_PRINTF("Mapper No.%d\n", mapper.id);

    prgram_addr_mask = 0;
    for (int i = 0; i < PRGROM_REMAP_TABLE_SIZE; i++) {
        mapper.prgrom_remap_table[i] = i;
    }
    for (int i = 0; i < CHRROM_REMAP_TABLE_SIZE; i++) {
        mapper.chrrom_remap_table[i] = i;
    }
    switch (mapper.id) {
        case 0:
        case 3:
            break;
        case 4:
            prgrom_remap(0xE000, prgrom_phys_size - 8192, 8192);
            break;
        default:
            NES_ERRORF("Unsupported Mapper Number\n");
            break;
    }
    NES_PRINTF("  CHRROM bank mask = 0x%x\n", chrrom_phys_addr_mask);

    int prgram_size = ines[8] * 8192;
    if (prgram_size == 0) {
        prgram_size = 8192;  // 8KB PRG RAM if not specified
    }
    NES_PRINTF("PRG RAM size = %d kB\n", prgram_size / 1024);
    prgram = new uint8_t[prgram_size];
    prgram_addr_mask = prgram_size - 1;

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
                chr = (uint16_t)(chr1 & 0x01) << 15;
                chr |= (uint16_t)(chr0 & 0x01) << 14;
                chr |= (uint16_t)(chr1 & 0x02) << 12;
                chr |= (uint16_t)(chr0 & 0x02) << 11;
                chr |= (uint16_t)(chr1 & 0x04) << 9;
                chr |= (uint16_t)(chr0 & 0x04) << 8;
                chr |= (uint16_t)(chr1 & 0x08) << 6;
                chr |= (uint16_t)(chr0 & 0x08) << 5;
                chr |= (uint16_t)(chr1 & 0x10) << 3;
                chr |= (uint16_t)(chr0 & 0x10) << 2;
                chr |= (uint16_t)(chr1 & 0x20);
                chr |= (uint16_t)(chr0 & 0x20) >> 1;
                chr |= (uint16_t)(chr1 & 0x40) >> 3;
                chr |= (uint16_t)(chr0 & 0x40) >> 4;
                chr |= (uint16_t)(chr1 & 0x80) >> 6;
                chr |= (uint16_t)(chr0 & 0x80) >> 7;
                chrrom_reordered0[ic * 8 + iy] = chr;
            }

            {
                uint16_t chr = 0;
                chr = (uint16_t)(chr1 & 0x80) << 8;
                chr |= (uint16_t)(chr0 & 0x80) << 7;
                chr |= (uint16_t)(chr1 & 0x40) << 7;
                chr |= (uint16_t)(chr0 & 0x40) << 6;
                chr |= (uint16_t)(chr1 & 0x20) << 6;
                chr |= (uint16_t)(chr0 & 0x20) << 5;
                chr |= (uint16_t)(chr1 & 0x10) << 5;
                chr |= (uint16_t)(chr0 & 0x10) << 4;
                chr |= (uint16_t)(chr1 & 0x08) << 4;
                chr |= (uint16_t)(chr0 & 0x08) << 3;
                chr |= (uint16_t)(chr1 & 0x04) << 3;
                chr |= (uint16_t)(chr0 & 0x04) << 2;
                chr |= (uint16_t)(chr1 & 0x02) << 2;
                chr |= (uint16_t)(chr0 & 0x02) << 1;
                chr |= (uint16_t)(chr1 & 0x01) << 1;
                chr |= (uint16_t)(chr0 & 0x01);
                chrrom_reordered1[ic * 8 + iy] = chr;
            }
        }
    }

    return true;
}

void ext_write(addr_t addr, uint8_t value) {
    switch (mapper.id) {
        case 3:
            if (0x8000 <= addr && addr <= 0xffff) {
                chrrom_remap(0x0000, (value & 0x3) * 0x2000, 0x2000);
            }
            break;
        case 4:
            mmc3_ext_write(addr, value);
            break;
    }
}

static void set_nametable_mirroring(bool four_screen, bool vertical) {
    if (four_screen) {
        vram_addr_mask = VRAM_SIZE - 1;
    } else if (vertical) {
        vram_addr_mask = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
    } else {
        vram_addr_mask = (VRAM_SIZE - 1) - (VRAM_SIZE / 4);
    }
}

// see: https://www.nesdev.org/wiki/MMC3
static void mmc3_ext_write(addr_t addr, uint8_t value) {
    if (0x8000 <= addr && addr <= 0x9FFF) {
        if ((addr & 0x0001) == 0) {
            mapper.mmc3_reg_sel = value;
        } else {
            if ((mapper.mmc3_reg_sel & 0x07) <= 1) {
                value &= 0xfe;  // ignore LSB for 2KB bank
            }
            mapper.mmc3_map_reg[mapper.mmc3_reg_sel & 0x07] = value;
        }

        prgrom_remap(0xA000, mapper.mmc3_map_reg[7] * 8192, 8192);

        constexpr int pbs = 8192;
        if ((mapper.mmc3_reg_sel & 0x40) == 0) {
            prgrom_remap(0x8000, mapper.mmc3_map_reg[6] * pbs, pbs);
            prgrom_remap(0xC000, prgrom_phys_size - pbs * 2, pbs);
        } else {
            prgrom_remap(0x8000, prgrom_phys_size - pbs * 2, pbs);
            prgrom_remap(0xC000, mapper.mmc3_map_reg[6] * pbs, pbs);
        }

        constexpr int cbs = 1024;
        if ((mapper.mmc3_reg_sel & 0x80) == 0) {
            chrrom_remap(0x0000, mapper.mmc3_map_reg[0] * cbs, cbs * 2);
            chrrom_remap(0x0800, mapper.mmc3_map_reg[1] * cbs, cbs * 2);
            chrrom_remap(0x1000, mapper.mmc3_map_reg[2] * cbs, cbs);
            chrrom_remap(0x1400, mapper.mmc3_map_reg[3] * cbs, cbs);
            chrrom_remap(0x1800, mapper.mmc3_map_reg[4] * cbs, cbs);
            chrrom_remap(0x1C00, mapper.mmc3_map_reg[5] * cbs, cbs);
        } else {
            chrrom_remap(0x0000, mapper.mmc3_map_reg[2] * cbs, cbs);
            chrrom_remap(0x0400, mapper.mmc3_map_reg[3] * cbs, cbs);
            chrrom_remap(0x0800, mapper.mmc3_map_reg[4] * cbs, cbs);
            chrrom_remap(0x0C00, mapper.mmc3_map_reg[5] * cbs, cbs);
            chrrom_remap(0x1000, mapper.mmc3_map_reg[0] * cbs, cbs * 2);
            chrrom_remap(0x1800, mapper.mmc3_map_reg[1] * cbs, cbs * 2);
        }
    } else if (0xA000 <= addr && addr <= 0xBFFF) {
        if ((addr & 0x0001) == 0) {
            set_nametable_mirroring(false, (value & 0x01) == 0);
        } else {
            // PRG RAM protect
            // TODO
        }
    } else if (0xC000 <= addr && addr <= 0xDFFF) {
        if ((addr & 0x0001) == 0) {
            // IRQ latch
            mapper.mmc3_irq_latch = value;
        } else {
            // IRQ reload
            mapper.mmc3_irq_counter = 0;
        }
    } else if (0xE000 <= addr && addr <= 0xFFFF) {
        if ((addr & 0x0001) == 0) {
            // IRQ disable
            mapper.mmc3_irq_enable = false;
            interrupt::deassert_irq();
        } else {
            // IRQ enable
            mapper.mmc3_irq_enable = true;
        }
    }
}

}  // namespace nes::memory
