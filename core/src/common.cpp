#include "shapones/common.hpp"

namespace nes {

const uint32_t NES_PALETTE_24BPP[64] = {
    0x808080, 0x003DA6, 0x0012B0, 0x440096, 0xA1005E, 0xC70028, 0xBA0600,
    0x8C1700, 0x5C2F00, 0x104500, 0x054A00, 0x00472E, 0x004166, 0x000000,
    0x050505, 0x050505, 0xC7C7C7, 0x0077FF, 0x2155FF, 0x8237FA, 0xEB2FB5,
    0xFF2950, 0xFF2200, 0xD63200, 0xC46200, 0x358000, 0x058F00, 0x008A55,
    0x0099CC, 0x212121, 0x090909, 0x090909, 0xFFFFFF, 0x0FD7FF, 0x69A2FF,
    0xD480FF, 0xFF45F3, 0xFF618B, 0xFF8833, 0xFF9C12, 0xFABC20, 0x9FE30E,
    0x2BF035, 0x0CF0A4, 0x05FBFF, 0x5E5E5E, 0x0D0D0D, 0x0D0D0D, 0xFFFFFF,
    0xA6FCFF, 0xB3ECFF, 0xDAABEB, 0xFFA8F9, 0xFFABB3, 0xFFD2B0, 0xFFEFA6,
    0xFFF79C, 0xD7E895, 0xA6EDAF, 0xA2F2DA, 0x99FFFC, 0xDDDDDD, 0x111111,
    0x111111,
};

uint8_t blend_table[64 * 64];

const char* result_to_string(result_t res) {
  switch (res) {
    case result_t::SUCCESS: return "Ok";
    case result_t::ERR_INVALID_NES_FORMAT: return "Bad iNES";
    case result_t::ERR_NO_DISK: return "No Disk";
    case result_t::ERR_DIR_NOT_FOUND: return "Dir Not Found";
    case result_t::ERR_PATH_TOO_LONG: return "Path Too Long";
    case result_t::ERR_FAILED_TO_OPEN_FILE: return "Open Failed";
    case result_t::ERR_FILE_NOT_OPEN: return "File Not Open";
    case result_t::ERR_FAILED_TO_SEEK_FILE: return "Seek Failed";
    case result_t::ERR_FAILED_TO_READ_FILE: return "Read Failed";
    case result_t::ERR_FAILED_TO_WRITE_FILE: return "Write Failed";
    case result_t::ERR_FAILED_TO_DELETE_FILE: return "Delete Failed";
    case result_t::ERR_INES_TOO_LARGE: return "iNES Too Large";
    case result_t::ERR_INVALID_STATE_FORMAT: return "Bad State Format";
    case result_t::ERR_STATE_SIZE_MISMATCH: return "Bad State Size";
    case result_t::ERR_INES_NOT_LOADED: return "iNES Not Loaded";
    case result_t::ERR_STATE_SLOT_FULL: return "State Slot Full";
    default: return "Unknown Error";
  }
}

uint8_t nearest_rgb888(uint8_t r, uint8_t g, uint8_t b) {
  int best_dist = 99999;
  uint_fast8_t best_index = 0;
  for (uint_fast8_t i = 0; i < 64; i++) {
    if (i == 0x0D) {
      // skip "darker than black"
      continue;
    }
    uint32_t ci = NES_PALETTE_24BPP[i];
    uint8_t ri = (ci >> 16) & 0xff;
    uint8_t gi = (ci >> 8) & 0xff;
    uint8_t bi = ci & 0xff;
    int dr = (r > ri) ? (r - ri) : (ri - r);
    int dg = (g > gi) ? (g - gi) : (gi - g);
    int db = (b > bi) ? (b - bi) : (bi - b);
    int dist = dr + dg + db;
    if (dist < best_dist) {
      best_dist = dist;
      best_index = i;
    }
  }
  return best_index;
}

}  // namespace nes
