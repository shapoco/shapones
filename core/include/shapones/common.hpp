#ifndef SHAPONES_COMMON_HPP
#define SHAPONES_COMMON_HPP

#if !(SHAPONES_NO_STDLIB)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#ifndef SHAPONES_DEFINE_FAST_INT
#define SHAPONES_DEFINE_FAST_INT (0)
#endif

#ifndef SHAPONES_IRQ_PENDING_SUPPORT
#define SHAPONES_IRQ_PENDING_SUPPORT (1)
#endif

#ifndef SHAPONES_MAX_FILENAME_LEN
#define SHAPONES_MAX_FILENAME_LEN (256)
#endif

#ifndef SHAPONES_MAX_PATH_LEN
#define SHAPONES_MAX_PATH_LEN (256)
#endif

#ifndef SHAPONES_MENU_LARGE_FONT
#define SHAPONES_MENU_LARGE_FONT (0)
#endif

#ifndef SHAPONES_MUTEX_FAST
#define SHAPONES_MUTEX_FAST (0)
#endif

#if SHAPONES_DEFINE_FAST_INT
using uint_fast8_t = uint8_t;
using uint_fast16_t = uint16_t;
using uint_fast32_t = uint32_t;
using int_fast8_t = int8_t;
using int_fast16_t = int16_t;
using int_fast32_t = int32_t;
#endif

#if SHAPONES_ENABLE_LOG

#if !(SHAPONES_NO_STDLIB)
#include <stdlib.h>
#endif

#define SHAPONES_PRINTF(fmt, ...)                \
  do {                                           \
    printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
    printf((fmt), ##__VA_ARGS__);                \
    fflush(stdout);                              \
  } while (0)

#define SHAPONES_ERRORF(fmt, ...)                \
  do {                                           \
    printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
    printf("*ERROR: ");                          \
    printf(fmt, ##__VA_ARGS__);                  \
    fflush(stdout);                              \
    nes::stop();                                 \
  } while (0)

#else

#define SHAPONES_PRINTF(fmt, ...) \
  do {                            \
  } while (0)

#define SHAPONES_ERRORF(fmt, ...) \
  do {                            \
    nes::stop();                  \
  } while (0)

#endif

#define SHAPONES_TRY(expr)                           \
  do {                                               \
    ::nes::result_t res = (expr);                    \
    if (res != nes::result_t::SUCCESS) {             \
      SHAPONES_PRINTF("Error Code: %d\n", (int)res); \
      return res;                                    \
    }                                                \
  } while (false)

#define SHAPONES_INLINE inline __attribute__((always_inline))
#define SHAPONES_NOINLINE __attribute__((noinline))

namespace nes {

using addr_t = uint_fast16_t;
using cycle_t = uint32_t;

static constexpr int SCREEN_WIDTH = 256;
static constexpr int SCREEN_HEIGHT = 240;

static constexpr addr_t WRAM_SIZE = 2 * 1024;
static constexpr addr_t VRAM_SIZE = 4 * 1024;
static constexpr addr_t PRGROM_RANGE = 32 * 1024;
static constexpr addr_t PRGRAM_RANGE = 8 * 1024;
static constexpr addr_t CHRROM_RANGE = 8 * 1024;

static constexpr int NUM_LOCKS = 3;
static constexpr int LOCK_PPU = 0;
static constexpr int LOCK_APU = 1;
static constexpr int LOCK_INTERRUPTS = 2;

static constexpr int MAX_FILENAME_LENGTH = SHAPONES_MAX_FILENAME_LEN;
static constexpr int MAX_PATH_LENGTH = SHAPONES_MAX_PATH_LEN;

enum class result_t {
  SUCCESS = 0,
  ERR_INVALID_NES_FORMAT,
  ERR_NO_DISK,
  ERR_DIR_NOT_FOUND,
  ERR_PATH_TOO_LONG,
  ERR_FAILED_TO_OPEN_FILE,
  ERR_FILE_NOT_OPEN,
  ERR_FAILED_TO_READ_FILE,
  ERR_FAILED_TO_WRITE_FILE,
  ERR_INES_TOO_LARGE,
  ERR_INVALID_STATE_FORMAT,
  ERR_STATE_SIZE_MISMATCH,
  ERR_INES_NOT_LOADED,
  ERR_STATE_SLOT_FULL,
};

struct config_t {
  uint32_t apu_sampling_rate;
};

class BufferWriter {
 public:
  uint8_t* buffer;
  BufferWriter(uint8_t* buffer) : buffer(buffer) {}

  void u8(uint8_t value) {
    buffer[0] = value;
    buffer++;
  }

  void u16(uint16_t value) {
    buffer[0] = value & 0xff;
    buffer[1] = (value >> 8) & 0xff;
    buffer += 2;
  }

  void u32(uint32_t value) {
    buffer[0] = value & 0xff;
    buffer[1] = (value >> 8) & 0xff;
    buffer[2] = (value >> 16) & 0xff;
    buffer[3] = (value >> 24) & 0xff;
    buffer += 4;
  }

  void u64(uint64_t value) {
    buffer[0] = value & 0xff;
    buffer[1] = (value >> 8) & 0xff;
    buffer[2] = (value >> 16) & 0xff;
    buffer[3] = (value >> 24) & 0xff;
    buffer[4] = (value >> 32) & 0xff;
    buffer[5] = (value >> 40) & 0xff;
    buffer[6] = (value >> 48) & 0xff;
    buffer[7] = (value >> 56) & 0xff;
    buffer += 8;
  }

  void b(bool value) {
    buffer[0] = value ? 1 : 0;
    buffer++;
  }
};

class BufferReader {
 public:
  const uint8_t* buffer;
  BufferReader(const uint8_t* buffer) : buffer(buffer) {}

  uint8_t u8() {
    uint8_t value = buffer[0];
    buffer++;
    return value;
  }

  uint16_t u16() {
    uint16_t value = buffer[0] | ((uint16_t)buffer[1] << 8);
    buffer += 2;
    return value;
  }

  uint32_t u32() {
    uint32_t value = buffer[0] | ((uint32_t)buffer[1] << 8) |
                     ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
    buffer += 4;
    return value;
  }

  uint64_t u64() {
    uint64_t value = (uint64_t)buffer[0] | ((uint64_t)buffer[1] << 8) |
                     ((uint64_t)buffer[2] << 16) | ((uint64_t)buffer[3] << 24) |
                     ((uint64_t)buffer[4] << 32) | ((uint64_t)buffer[5] << 40) |
                     ((uint64_t)buffer[6] << 48) | ((uint64_t)buffer[7] << 56);
    buffer += 8;
    return value;
  }

  bool b() {
    bool value = buffer[0] ? true : false;
    buffer++;
    return value;
  }
};

enum class nametable_arrangement_t : uint8_t {
  HORIZONTAL = 0,
  VERTICAL = 1,
  SINGLE_LOWER = 2,
  SINGLE_UPPER = 3,
  FOUR_SCREEN = 4,
};

extern const uint32_t NES_PALETTE_24BPP[64];

extern uint8_t blend_table[64 * 64];

const char* result_to_string(result_t res);

void stop();
const char* get_ines_path();

uint8_t nearest_rgb888(uint8_t r, uint8_t g, uint8_t b);

static SHAPONES_INLINE uint8_t blend_colors(uint8_t a, uint8_t b) {
  return blend_table[(a << 6) | b];
}

}  // namespace nes

#endif
