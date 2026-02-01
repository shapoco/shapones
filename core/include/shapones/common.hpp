#ifndef SHAPONES_COMMON_HPP
#define SHAPONES_COMMON_HPP

#if USE_PICOPAD
#define SHAPONES_NO_STDLIB (1)
#define SHAPONES_PICOLIBSDK (1)
#define SHAPONES_DEFINE_FAST_INT (1)
#define SHAPONES_THREAD_FENCE_SEQ_CST() cb()
#endif

#ifndef SHAPONES_NO_STDLIB
#define SHAPONES_NO_STDLIB (0)
#endif

#ifndef SHAPONES_PICOLIBSDK
#define SHAPONES_PICOLIBSDK (0)
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

#ifndef SHAPONES_LOCK_FAST
#define SHAPONES_LOCK_FAST (0)
#endif

#if SHAPONES_PICOLIBSDK
#include "../include.h"
#endif
#if !(SHAPONES_NO_STDLIB)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <atomic>
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

#ifdef ARDUINO

#include <Arduino.h>

#define SHAPONES_PRINTF(fmt, ...)                       \
  do {                                                  \
    Serial.printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
    Serial.printf((fmt), ##__VA_ARGS__);                \
  } while (0)

#define SHAPONES_ERRORF(fmt, ...)                       \
  do {                                                  \
    Serial.printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
    Serial.printf("*ERROR: ");                          \
    Serial.printf(fmt, ##__VA_ARGS__);                  \
    nes::stop();                                        \
  } while (0)

#else

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

#endif

#else

#define SHAPONES_PRINTF(fmt, ...) \
  do {                            \
  } while (0)

#define SHAPONES_ERRORF(fmt, ...) \
  do {                            \
    nes::stop();                  \
  } while (0)

#endif

#define SHAPONES_TRY(expr)                               \
  do {                                                   \
    ::nes::result_t res = (expr);                        \
    if (res != ::nes::result_t::SUCCESS) {               \
      SHAPONES_PRINTF("Error Code: %d (%s)\n", (int)res, \
                      ::nes::result_to_string(res));     \
      return res;                                        \
    }                                                    \
  } while (false)

#define SHAPONES_INLINE inline __attribute__((always_inline))
#define SHAPONES_NOINLINE __attribute__((noinline))

#ifndef SHAPONES_THREAD_FENCE_SEQ_CST
#define SHAPONES_THREAD_FENCE_SEQ_CST() \
  std::atomic_thread_fence(std::memory_order_seq_cst);
#endif

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

static constexpr int NUM_SPINLOCKS = 3;
static constexpr int SPINLOCK_IRQ = 0;
static constexpr int SPINLOCK_PPU = 1;
static constexpr int SPINLOCK_APU = 2;

static constexpr int NUM_SEMAPHORES = 2;
static constexpr int SEMAPHORE_PPU = 0;
static constexpr int SEMAPHORE_APU = 1;

static constexpr int MAX_FILENAME_LENGTH = SHAPONES_MAX_FILENAME_LEN;
static constexpr int MAX_PATH_LENGTH = SHAPONES_MAX_PATH_LEN;

enum class result_t {
  SUCCESS = 0,

  ERR_RAM_BASE = 0x100,
  ERR_RAM_ALLOC_FAILED,

  ERR_INES_BASE = 0x200,
  ERR_INES_TOO_LARGE,
  ERR_INES_INVALID_FORMAT,
  ERR_INES_NOT_LOADED,

  ERR_FS_BASE = 0x300,
  ERR_FS_NO_DISK,
  ERR_FS_PATH_TOO_LONG,
  ERR_FS_DIR_NOT_FOUND,
  ERR_FS_OPEN_FAILED,
  ERR_FS_FILE_NOT_OPEN,
  ERR_FS_SEEK_FAILED,
  ERR_FS_READ_FAILED,
  ERR_FS_WRITE_FAILED,
  ERR_FS_DELETE_FAILED,

  ERR_STATE_BASE = 0x400,
  ERR_STATE_INVALID_FORMAT,
  ERR_STATE_SIZE_MISMATCH,
  ERR_STATE_SLOT_FULL,
  ERR_STATE_NO_SLOT_DATA,

  ERR_HOST_BASE = 0xF00,
  ERR_FLASH_ERASE_FAILED,
  ERR_FLASH_PROGRAM_FAILED,
  ERR_MMAP_FAILED,
};

struct config_t {
  uint32_t apu_sampling_rate;
};

struct reg_write_t {
  addr_t addr;
  uint8_t data;
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

result_t map_ines(const uint8_t* ines, const char* path);
void unmap_ines();

const char* get_ines_path();
void stop();

uint8_t nearest_rgb888(uint8_t r, uint8_t g, uint8_t b);

static SHAPONES_INLINE uint8_t blend_colors(uint8_t a, uint8_t b) {
  return blend_table[(a << 6) | b];
}

}  // namespace nes

#endif
