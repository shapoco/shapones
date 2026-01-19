#include "shapones/memory.hpp"
#include "shapones/mapper.hpp"
#include "shapones/ppu.hpp"

#pragma GCC optimize("Ofast")

namespace nes::memory {

uint8_t wram[WRAM_SIZE];
uint8_t vram[VRAM_SIZE];
addr_t vram_addr_and = VRAM_SIZE - 1;
addr_t vram_addr_or = 0;

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

  NametableArrangement mode;
  if ((flags6 & 0x8) != 0) {
    mode = NametableArrangement::FOUR_SCREEN;
  } else if ((flags6 & 0x1) == 0) {
    mode = NametableArrangement::VERTICAL;
  } else {
    mode = NametableArrangement::HORIZONTAL;
  }
  switch (mode) {
    case NametableArrangement::FOUR_SCREEN:
      SHAPONES_PRINTF("Nametable arrange: Four-screen\n");
      break;
    case NametableArrangement::VERTICAL:
      SHAPONES_PRINTF("Nametable arrange: Vertical\n");
      break;
    case NametableArrangement::HORIZONTAL:
      SHAPONES_PRINTF("Nametable arrange: Horizontal\n");
      break;
    case NametableArrangement::SINGLE_LOWER:
      SHAPONES_PRINTF("Nametable arrange: One-screen lower\n");
      break;
    case NametableArrangement::SINGLE_UPPER:
      SHAPONES_PRINTF("Nametable arrange: One-screen upper\n");
      break;
    default:
      SHAPONES_ERRORF("Invalid nametable mirroring mode: %d\n",
                      static_cast<int>(mode));
      break;
  }
  set_nametable_mirroring(mode);

  mapper::init(ines);

  return true;
}

void set_nametable_mirroring(NametableArrangement mode) {
  switch (mode) {
    case NametableArrangement::FOUR_SCREEN:
      vram_addr_and = VRAM_SIZE - 1;
      vram_addr_or = 0;
      break;
    case NametableArrangement::HORIZONTAL:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
      vram_addr_or = 0;
      break;
    case NametableArrangement::VERTICAL:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 4);
      vram_addr_or = 0;
      break;
    case NametableArrangement::SINGLE_LOWER:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
      vram_addr_or = 0;
      break;
    case NametableArrangement::SINGLE_UPPER:
      vram_addr_and = (VRAM_SIZE - 1) - (VRAM_SIZE / 2);
      vram_addr_or = VRAM_SIZE / 2;
      break;
    default:
      SHAPONES_ERRORF("Invalid nametable mirroring mode: %d\n",
                      static_cast<int>(mode));
      break;
  }
}

}  // namespace nes::memory
