#include "shapones/memory.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/mapper.hpp"
#include "shapones/ppu.hpp"

namespace nes::memory {

uint8_t wram[WRAM_SIZE];
uint8_t vram[VRAM_SIZE];
addr_t vram_addr_and = VRAM_SIZE - 1;
addr_t vram_addr_or = 0;

uint8_t dummy_memory = 0;

uint8_t *prgram = nullptr;
uint8_t *chrram = nullptr;
const uint8_t *prgrom = &dummy_memory;
const uint8_t *chrrom = &dummy_memory;

uint16_t prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
uint16_t chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

uint32_t prgrom_phys_size = PRGROM_RANGE;
uint32_t prgrom_phys_addr_mask = 0;
uint32_t prgram_size = 0;
addr_t prgram_addr_mask = 0;

uint32_t chrrom_phys_size = CHRROM_RANGE;
uint32_t chrrom_phys_addr_mask = 0;
addr_t prgrom_cpu_addr_mask = PRGROM_RANGE - 1;
uint32_t chrram_size = 0;

result_t init() {
  deinit();
  unmap_ines();
  if (prgram) prgram = new uint8_t[1];
  if (chrram == nullptr) chrram = new uint8_t[1];
  return result_t::SUCCESS;
}

void deinit() {
  if (prgram) {
    delete[] prgram;
    prgram = nullptr;
  }
  if (chrram) {
    delete[] chrram;
    chrram = nullptr;
  }
}

result_t map_ines(const uint8_t *ines) {
  // iNES file format
  // https://www.nesdev.org/wiki/INES

  unmap_ines();

  // marker
  if (ines[0] != 0x4e && ines[1] != 0x45 && ines[2] != 0x53 &&
      ines[3] != 0x1a) {
    return result_t::ERR_INVALID_NES_FORMAT;
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
  SHAPONES_PRINTF("Number of PRGROM pages = %d (%dkB)\n", num_prg_rom_pages,
                  prgrom_phys_size / 1024);

  // Size of CHR ROM in 8 KB units
  int num_chr_rom_pages = ines[5];
  if (num_chr_rom_pages == 0) {
    chrrom_phys_size = CHRROM_RANGE;
    SHAPONES_PRINTF("Number of CHRROM pages = %d (%dkB CHRRAM)\n",
                    num_chr_rom_pages, chrrom_phys_size / 1024);
  } else {
    chrrom_phys_size = num_chr_rom_pages * CHRROM_PAGE_SIZE;
    SHAPONES_PRINTF("Number of CHRROM pages = %d (%dkB CHRROM)\n",
                    num_chr_rom_pages, chrrom_phys_size / 1024);
  }
  chrrom_phys_addr_mask = chrrom_phys_size - 1;

  prgram_addr_mask = 0;
  for (int i = 0; i < PRGROM_REMAP_TABLE_SIZE; i++) {
    prgrom_remap_table[i] = i;
  }
  for (int i = 0; i < CHRROM_REMAP_TABLE_SIZE; i++) {
    chrrom_remap_table[i] = i;
  }

  uint8_t flags6 = ines[6];
  SHAPONES_PRINTF("Flags6 = 0x%02x\n", (int)flags6);

  nametable_arrangement_t mode;
  if ((flags6 & 0x8) != 0) {
    mode = nametable_arrangement_t::FOUR_SCREEN;
  } else if ((flags6 & 0x1) == 0) {
    mode = nametable_arrangement_t::VERTICAL;
  } else {
    mode = nametable_arrangement_t::HORIZONTAL;
  }
  switch (mode) {
    case nametable_arrangement_t::FOUR_SCREEN:
      SHAPONES_PRINTF("Nametable arrange: Four-screen\n");
      break;
    case nametable_arrangement_t::VERTICAL:
      SHAPONES_PRINTF("Nametable arrange: Vertical\n");
      break;
    case nametable_arrangement_t::HORIZONTAL:
      SHAPONES_PRINTF("Nametable arrange: Horizontal\n");
      break;
    case nametable_arrangement_t::SINGLE_LOWER:
      SHAPONES_PRINTF("Nametable arrange: One-screen lower\n");
      break;
    case nametable_arrangement_t::SINGLE_UPPER:
      SHAPONES_PRINTF("Nametable arrange: One-screen upper\n");
      break;
    default:
      SHAPONES_ERRORF("Invalid nametable mirroring mode: %d\n",
                      static_cast<int>(mode));
      break;
  }
  set_nametable_arrangement(mode);

  prgram_size = ines[8] * 8192;
  if (prgram_size == 0) {
    prgram_size = 8192;  // 8KB PRG RAM if not specified
  }
  SHAPONES_PRINTF("PRG RAM size = %d kB\n", prgram_size / 1024);
  if (prgram) {
    delete[] prgram;
  }
  prgram = new uint8_t[prgram_size];
  prgram_addr_mask = prgram_size - 1;

  // 512-byte trainer at $7000-$71FF (stored before PRG data)
  bool has_trainer = (flags6 & 0x4) != 0;

  int start_of_prg_rom = 0x10;
  if (has_trainer) start_of_prg_rom += 0x200;
  prgrom = ines + start_of_prg_rom;

  if (chrram) {
    delete[] chrram;
  }
  if (num_chr_rom_pages == 0) {
    chrram_size = CHRROM_RANGE;
    chrram = new uint8_t[CHRROM_RANGE];
    chrrom = chrram;
  } else {
    chrram_size = 0;
    chrram = new uint8_t[1];
    int start_of_chr_rom = start_of_prg_rom + num_prg_rom_pages * 0x4000;
    chrrom = ines + start_of_chr_rom;
  }

  SHAPONES_TRY(mapper::map_ines(ines));

  return result_t::SUCCESS;
}

void unmap_ines() {
  prgrom = &dummy_memory;
  chrrom = &dummy_memory;
  prgram_addr_mask = 0;
  prgrom_phys_addr_mask = 0;
  chrrom_phys_addr_mask = 0;
}

void set_nametable_arrangement(nametable_arrangement_t mode) {
  switch (mode) {
    case nametable_arrangement_t::FOUR_SCREEN:
      vram_addr_and = VRAM_SIZE - 1;
      vram_addr_or = 0;
      break;
    case nametable_arrangement_t::HORIZONTAL:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
      vram_addr_or = 0;
      break;
    case nametable_arrangement_t::VERTICAL:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 4);
      vram_addr_or = 0;
      break;
    case nametable_arrangement_t::SINGLE_LOWER:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
      vram_addr_or = 0;
      break;
    case nametable_arrangement_t::SINGLE_UPPER:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
      vram_addr_or = VRAM_SIZE / 2;
      break;
    default:
      SHAPONES_ERRORF("Invalid nametable mirroring mode: %d\n",
                      static_cast<int>(mode));
      break;
  }
}

uint32_t get_state_size() {
  return WRAM_SIZE + VRAM_SIZE + prgram_size + chrram_size;
}

result_t save_state(void *file_handle) {
  SHAPONES_TRY(fs_write(file_handle, wram, WRAM_SIZE));
  SHAPONES_TRY(fs_write(file_handle, vram, VRAM_SIZE));
  if (prgram_size > 0) {
    SHAPONES_TRY(fs_write(file_handle, prgram, prgram_size));
  }
  if (chrram_size > 0) {
    SHAPONES_TRY(fs_write(file_handle, chrram, chrram_size));
  }
  return result_t::SUCCESS;
}

result_t load_state(void *file_handle) {
  SHAPONES_TRY(fs_read(file_handle, wram, WRAM_SIZE));
  SHAPONES_TRY(fs_read(file_handle, vram, VRAM_SIZE));
  if (prgram_size > 0) {
    SHAPONES_TRY(fs_read(file_handle, prgram, prgram_size));
  }
  if (chrram_size > 0) {
    SHAPONES_TRY(fs_read(file_handle, chrram, chrram_size));
  }
  return result_t::SUCCESS;
}

}  // namespace nes::memory
