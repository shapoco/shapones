#include "shapones/mapper.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/memory.hpp"

namespace nes::mapper {

int id;

uint8_t *prgram = nullptr;
const uint8_t *prgrom;
const uint8_t *chrrom;

int prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
int chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

#if SHAPONES_ENABLE_CHROM_CACHE
uint16_t *chrrom_reordered0;
uint16_t *chrrom_reordered1;
#endif

addr_t prgram_addr_mask;
uint32_t prgrom_phys_size;
uint32_t prgrom_phys_addr_mask;
uint32_t chrrom_phys_size;
uint32_t chrrom_phys_addr_mask;
addr_t prgrom_cpu_addr_mask = PRGROM_RANGE - 1;
addr_t vram_addr_mask = VRAM_SIZE - 1;

uint8_t mmc3_reg_sel;
uint8_t mmc3_map_reg[8];

static void mmc3_ext_write(addr_t addr, uint8_t value);
static void reorder_chrrom_block(const uint8_t *src, uint16_t *dst_fwd,
                                 uint16_t *dst_rev, int num_words);

void init(const uint8_t *ines) {
    // Size of PRG ROM in 16 KB units
    int num_prg_rom_pages = ines[4];
    prgrom_phys_size = num_prg_rom_pages * PRGROM_PAGE_SIZE;
    prgrom_phys_addr_mask = prgrom_phys_size - 1;
    if (num_prg_rom_pages <= 1) {
        prgrom_cpu_addr_mask = 0x3fff;
    } else {
        prgrom_cpu_addr_mask = PRGROM_RANGE - 1;
    }
    SHAPONES_PRINTF("Number of PRGROM pages = %d (%dkB)\n", num_prg_rom_pages,
               prgrom_phys_size / 1024);

    // Size of CHR ROM in 8 KB units
    int num_chr_rom_pages = ines[5];
    chrrom_phys_size = num_chr_rom_pages * CHRROM_PAGE_SIZE;
    chrrom_phys_addr_mask = chrrom_phys_size - 1;
    SHAPONES_PRINTF("Number of CHRROM pages = %d (%dkB)\n", num_chr_rom_pages,
               chrrom_phys_size / 1024);

    uint8_t flags6 = ines[6];
    uint8_t flags7 = ines[7];

    id = (flags7 & 0xf0) | ((flags6 >> 4) & 0xf);
    SHAPONES_PRINTF("Mapper No.%d\n", id);

    // 512-byte trainer at $7000-$71FF (stored before PRG data)
    bool has_trainer = (flags6 & 0x2) != 0;

    prgram_addr_mask = 0;
    for (int i = 0; i < PRGROM_REMAP_TABLE_SIZE; i++) {
        prgrom_remap_table[i] = i;
    }
    for (int i = 0; i < CHRROM_REMAP_TABLE_SIZE; i++) {
        chrrom_remap_table[i] = i;
    }
    switch (id) {
        case 0:
        case 3: break;
        case 4: prgrom_remap(0xE000, prgrom_phys_size - 8192, 8192); break;
        default: SHAPONES_ERRORF("Unsupported Mapper Number\n"); break;
    }
    SHAPONES_PRINTF("  CHRROM bank mask = 0x%x\n", chrrom_phys_addr_mask);

    int prgram_size = ines[8] * 8192;
    if (prgram_size == 0) {
        prgram_size = 8192;  // 8KB PRG RAM if not specified
    }
    SHAPONES_PRINTF("PRG RAM size = %d kB\n", prgram_size / 1024);
    prgram = new uint8_t[prgram_size];
    prgram_addr_mask = prgram_size - 1;

    int start_of_prg_rom = 0x10;
    if (has_trainer) start_of_prg_rom += 0x200;
    prgrom = ines + start_of_prg_rom;

    int start_of_chr_rom = start_of_prg_rom + num_prg_rom_pages * 0x4000;
    chrrom = ines + start_of_chr_rom;

#if SHAPONES_ENABLE_CHROM_CACHE
    // reorder CHRROM bits for fast access
    int num_words = num_chr_rom_pages * (CHRROM_PAGE_SIZE / 2);
    chrrom_reordered0 = new uint16_t[num_words];
    chrrom_reordered1 = new uint16_t[num_words];
    reorder_chrrom_block(chrrom, chrrom_reordered0, chrrom_reordered1,
                         num_words);
#endif
}

void ext_write(addr_t addr, uint8_t value) {
    switch (id) {
        case 3:
            if (0x8000 <= addr && addr <= 0xffff) {
                chrrom_remap(0x0000, (value & 0x3) * 0x2000, 0x2000);
            }
            break;
        case 4: mmc3_ext_write(addr, value); break;
    }
}

// see: https://www.nesdev.org/wiki/MMC3
static void mmc3_ext_write(addr_t addr, uint8_t value) {
    if (0x8000 <= addr && addr <= 0x9FFF) {
        if ((addr & 0x0001) == 0) {
            mmc3_reg_sel = value;
        } else {
            int ireg = mmc3_reg_sel & 0x07;
            if (ireg <= 1) {
                value &= 0xfe;  // ignore LSB for 2KB bank
            }
            mmc3_map_reg[ireg] = value;
        }

        constexpr int pbs = 8192;
        prgrom_remap(0xA000, mmc3_map_reg[7] * pbs, pbs);
        if ((mmc3_reg_sel & 0x40) == 0) {
            prgrom_remap(0x8000, mmc3_map_reg[6] * pbs, pbs);
            prgrom_remap(0xC000, prgrom_phys_size - pbs * 2, pbs);
        } else {
            prgrom_remap(0x8000, prgrom_phys_size - pbs * 2, pbs);
            prgrom_remap(0xC000, mmc3_map_reg[6] * pbs, pbs);
        }

        constexpr int cbs = 1024;
        if ((mmc3_reg_sel & 0x80) == 0) {
            chrrom_remap(0x0000, mmc3_map_reg[0] * cbs, cbs * 2);
            chrrom_remap(0x0800, mmc3_map_reg[1] * cbs, cbs * 2);
            chrrom_remap(0x1000, mmc3_map_reg[2] * cbs, cbs);
            chrrom_remap(0x1400, mmc3_map_reg[3] * cbs, cbs);
            chrrom_remap(0x1800, mmc3_map_reg[4] * cbs, cbs);
            chrrom_remap(0x1C00, mmc3_map_reg[5] * cbs, cbs);
        } else {
            chrrom_remap(0x0000, mmc3_map_reg[2] * cbs, cbs);
            chrrom_remap(0x0400, mmc3_map_reg[3] * cbs, cbs);
            chrrom_remap(0x0800, mmc3_map_reg[4] * cbs, cbs);
            chrrom_remap(0x0C00, mmc3_map_reg[5] * cbs, cbs);
            chrrom_remap(0x1000, mmc3_map_reg[0] * cbs, cbs * 2);
            chrrom_remap(0x1800, mmc3_map_reg[1] * cbs, cbs * 2);
        }
    } else if (0xA000 <= addr && addr <= 0xBFFF) {
        if ((addr & 0x0001) == 0) {
            memory::set_nametable_mirroring(false, (value & 0x01) == 0);
        } else {
            // PRG RAM protect
            // TODO
        }
    } else if (0xC000 <= addr && addr <= 0xDFFF) {
        if ((addr & 0x0001) == 0) {
            // IRQ latch
            ppu::mmc3_irq_set_latch(value);
        } else {
            // IRQ reload
            ppu::mmc3_irq_set_reload();
        }
    } else if (0xE000 <= addr && addr <= 0xFFFF) {
        if ((addr & 0x0001) == 0) {
            // IRQ disable
            ppu::mmc3_irq_set_enable(false);
        } else {
            // IRQ enable
            ppu::mmc3_irq_set_enable(true);
        }
    }
}

static void reorder_chrrom_block(const uint8_t *src, uint16_t *dst_fwd,
                                 uint16_t *dst_rev, int num_words) {
    int num_chars = num_words / 8;
    for (int ic = 0; ic < num_chars; ic++) {
        for (int iy = 0; iy < 8; iy++) {
            uint8_t chr0 = src[ic * 16 + iy];
            uint8_t chr1 = src[ic * 16 + iy + 8];

            uint16_t fwd = 0;
            fwd = (uint16_t)(chr1 & 0x01) << 15;
            fwd |= (uint16_t)(chr0 & 0x01) << 14;
            fwd |= (uint16_t)(chr1 & 0x02) << 12;
            fwd |= (uint16_t)(chr0 & 0x02) << 11;
            fwd |= (uint16_t)(chr1 & 0x04) << 9;
            fwd |= (uint16_t)(chr0 & 0x04) << 8;
            fwd |= (uint16_t)(chr1 & 0x08) << 6;
            fwd |= (uint16_t)(chr0 & 0x08) << 5;
            fwd |= (uint16_t)(chr1 & 0x10) << 3;
            fwd |= (uint16_t)(chr0 & 0x10) << 2;
            fwd |= (uint16_t)(chr1 & 0x20);
            fwd |= (uint16_t)(chr0 & 0x20) >> 1;
            fwd |= (uint16_t)(chr1 & 0x40) >> 3;
            fwd |= (uint16_t)(chr0 & 0x40) >> 4;
            fwd |= (uint16_t)(chr1 & 0x80) >> 6;
            fwd |= (uint16_t)(chr0 & 0x80) >> 7;
            dst_fwd[ic * 8 + iy] = fwd;

            uint16_t rev = 0;
            rev = (uint16_t)(chr1 & 0x80) << 8;
            rev |= (uint16_t)(chr0 & 0x80) << 7;
            rev |= (uint16_t)(chr1 & 0x40) << 7;
            rev |= (uint16_t)(chr0 & 0x40) << 6;
            rev |= (uint16_t)(chr1 & 0x20) << 6;
            rev |= (uint16_t)(chr0 & 0x20) << 5;
            rev |= (uint16_t)(chr1 & 0x10) << 5;
            rev |= (uint16_t)(chr0 & 0x10) << 4;
            rev |= (uint16_t)(chr1 & 0x08) << 4;
            rev |= (uint16_t)(chr0 & 0x08) << 3;
            rev |= (uint16_t)(chr1 & 0x04) << 3;
            rev |= (uint16_t)(chr0 & 0x04) << 2;
            rev |= (uint16_t)(chr1 & 0x02) << 2;
            rev |= (uint16_t)(chr0 & 0x02) << 1;
            rev |= (uint16_t)(chr1 & 0x01) << 1;
            rev |= (uint16_t)(chr0 & 0x01);
            dst_rev[ic * 8 + iy] = rev;
        }
    }
}

}  // namespace nes::mapper
