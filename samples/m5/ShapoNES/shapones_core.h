#ifndef SHAPONES_CORE_H
#define SHAPONES_CORE_H

#define SHAPONES_MENU_LARGE_FONT (1)
#define SHAPONES_ENABLE_LOG (1)

#pragma GCC optimize ("O2")

#ifndef SHAPONES_HPP
#define SHAPONES_HPP

// #include "shapones/apu.hpp"

#ifndef SHAPONES_APU_HPP
#define SHAPONES_APU_HPP

// #include "shapones/common.hpp"

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

#if SHAPONES_PICOLIBSDK
// #include "../include.h"

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

static constexpr int NUM_SPINLOCKS = 1;
static constexpr int SPINLOCK_IRQ = 0;

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

namespace nes::apu {

static constexpr int TIMER_PREC = 16;

static constexpr addr_t REG_PULSE1_REG0 = 0x4000;
static constexpr addr_t REG_PULSE1_REG1 = 0x4001;
static constexpr addr_t REG_PULSE1_REG2 = 0x4002;
static constexpr addr_t REG_PULSE1_REG3 = 0x4003;
static constexpr addr_t REG_PULSE2_REG0 = 0x4004;
static constexpr addr_t REG_PULSE2_REG1 = 0x4005;
static constexpr addr_t REG_PULSE2_REG2 = 0x4006;
static constexpr addr_t REG_PULSE2_REG3 = 0x4007;
static constexpr addr_t REG_TRIANGLE_REG0 = 0x4008;
static constexpr addr_t REG_TRIANGLE_REG2 = 0x400a;
static constexpr addr_t REG_TRIANGLE_REG3 = 0x400b;
static constexpr addr_t REG_NOISE_REG0 = 0x400c;
static constexpr addr_t REG_NOISE_REG2 = 0x400e;
static constexpr addr_t REG_NOISE_REG3 = 0x400f;
static constexpr addr_t REG_DMC_REG0 = 0x4010;
static constexpr addr_t REG_DMC_REG1 = 0x4011;
static constexpr addr_t REG_DMC_REG2 = 0x4012;
static constexpr addr_t REG_DMC_REG3 = 0x4013;
static constexpr addr_t REG_STATUS = 0x4015;
static constexpr addr_t REG_FRAME_COUNTER = 0x4017;

static constexpr uint8_t ENV_FLAG_START = 0x1;
static constexpr uint8_t ENV_FLAG_CONSTANT = 0x2;
static constexpr uint8_t ENV_FLAG_HALT_LOOP = 0x4;

static constexpr uint8_t SWP_FLAG_ENABLE = 0x1;
static constexpr uint8_t SWP_FLAG_NEGATE = 0x2;

static constexpr uint8_t LIN_FLAG_RELOAD = 0x1;
static constexpr uint8_t LIN_FLAG_CONTROL = 0x2;

static constexpr uint8_t NOISE_FLAG_FB_MODE = 0x1;

static constexpr uint8_t DMC_FLAG_LOOP = 0x1;
static constexpr uint8_t DMC_FLAG_IRQ_ENABLE = 0x2;

struct envelope_t {
  static constexpr uint32_t STATE_SIZE = 8;

  uint8_t flags;
  uint8_t volume;
  uint8_t divider;
  uint8_t decay;

  void store(uint8_t *buff) const {
    *(buff++) = flags;
    *(buff++) = volume;
    *(buff++) = divider;
    *(buff++) = decay;
  }

  void load(const uint8_t *buff) {
    flags = *(buff++);
    volume = *(buff++);
    divider = *(buff++);
    decay = *(buff++);
  }
};

struct sweep_t {
  static constexpr uint32_t STATE_SIZE = 8;

  uint8_t flags;
  uint8_t period;
  uint8_t divider;
  uint8_t shift;

  void store(uint8_t *buff) const {
    *(buff++) = flags;
    *(buff++) = period;
    *(buff++) = divider;
    *(buff++) = shift;
  }

  void load(const uint8_t *buff) {
    flags = *(buff++);
    period = *(buff++);
    divider = *(buff++);
    shift = *(buff++);
  }
};

struct linear_counter_t {
  static constexpr uint32_t STATE_SIZE = 8;

  uint8_t flags;
  uint8_t counter;
  uint8_t reload_value;

  void store(uint8_t *buff) const {
    *(buff++) = flags;
    *(buff++) = counter;
    *(buff++) = reload_value;
  }

  void load(const uint8_t *buff) {
    flags = *(buff++);
    counter = *(buff++);
    reload_value = *(buff++);
  }
};

struct pulse_state_t {
  static constexpr uint32_t STATE_SIZE =
       envelope_t::STATE_SIZE + sweep_t::STATE_SIZE + 24;

  envelope_t envelope;
  sweep_t sweep;

  uint32_t timer;
  uint32_t timer_period;
  uint32_t phase;
  uint8_t length;
  uint8_t waveform;

  void store(uint8_t *buff) const {
    envelope.store(buff);
    buff += envelope_t::STATE_SIZE;
    sweep.store(buff);
    buff += sweep_t::STATE_SIZE;
    BufferWriter writer(buff);
    writer.u32(timer);
    writer.u32(timer_period);
    writer.u32(phase);
    writer.u8(length);
    writer.u8(waveform);
  }

  void load(const uint8_t *buff) {
    envelope.load(buff);
    buff += envelope_t::STATE_SIZE;
    sweep.load(buff);
    buff += sweep_t::STATE_SIZE;
    BufferReader reader(buff);
    timer = reader.u32();
    timer_period = reader.u32();
    phase = reader.u32();
    length = reader.u8();
    waveform = reader.u8();
  }
};

struct triangle_state_t {
  static constexpr uint32_t STATE_SIZE =
       linear_counter_t::STATE_SIZE + 24;

  linear_counter_t linear;

  uint32_t timer;
  uint32_t timer_period;
  uint32_t phase;
  uint8_t length;

  void store(uint8_t *buff) const {
    linear.store(buff);
    buff += linear_counter_t::STATE_SIZE;
    BufferWriter writer(buff);
    writer.u32(timer);
    writer.u32(timer_period);
    writer.u32(phase);
    writer.u8(length);
  }

  void load(const uint8_t *buff) {
    linear.load(buff);
    buff += linear_counter_t::STATE_SIZE;
    BufferReader reader(buff);
    timer = reader.u32();
    timer_period = reader.u32();
    phase = reader.u32();
    length = reader.u8();
  }
};

struct noise_state_t {
  static constexpr uint32_t STATE_SIZE = envelope_t::STATE_SIZE + 16;

  envelope_t envelope;

  uint32_t timer;
  uint32_t timer_period;
  uint16_t lfsr;
  uint8_t flags;
  uint8_t length;

  void store(uint8_t *buff) const {
    envelope.store(buff);
    buff += envelope_t::STATE_SIZE;
    BufferWriter writer(buff);
    writer.u32(timer);
    writer.u32(timer_period);
    writer.u16(lfsr);
    writer.u8(flags);
    writer.u8(length);
  }

  void load(const uint8_t *buff) {
    envelope.load(buff);
    buff += envelope_t::STATE_SIZE;
    BufferReader reader(buff);
    timer = reader.u32();
    timer_period = reader.u32();
    lfsr = reader.u16();
    flags = reader.u8();
    length = reader.u8();
  }
};

struct dmc_state_t {
  static constexpr uint32_t STATE_SIZE = 32;

  uint32_t timer_step;
  uint32_t timer;
  addr_t sample_addr;
  addr_t addr_counter;
  uint16_t sample_length;
  uint16_t bytes_remaining;
  uint8_t flags;
  uint8_t shift_reg;
  uint8_t bits_remaining;
  uint8_t out_level;

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u32(timer_step);
    writer.u32(timer);
    writer.u16(sample_addr);
    writer.u16(addr_counter);
    writer.u16(sample_length);
    writer.u16(bytes_remaining);
    writer.u8(flags);
    writer.u8(shift_reg);
    writer.u8(bits_remaining);
    writer.u8(out_level);
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    timer_step = reader.u32();
    timer = reader.u32();
    sample_addr = reader.u16();
    addr_counter = reader.u16();
    sample_length = reader.u16();
    bytes_remaining = reader.u16();
    flags = reader.u8();
    shift_reg = reader.u8();
    bits_remaining = reader.u8();
    out_level = reader.u8();
  }
};

union status_t {
  uint8_t raw;
  struct {
    uint8_t pulse0_enable : 1;
    uint8_t pulse1_enable : 1;
    uint8_t triangle_enable : 1;
    uint8_t noise_enable : 1;
    uint8_t dmc_enable : 1;
    uint8_t reserved : 1;
    uint8_t dummy_frame_interrupt : 1;
    uint8_t dummy_dmc_interrupt : 1;
  };
};

result_t init();
void deinit();
result_t set_sampling_rate(uint32_t rate_hz);

result_t reset();

uint8_t reg_read(addr_t addr);
void reg_write(addr_t addr, uint8_t value);

result_t service(uint8_t *buff, int len);

uint32_t get_state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::apu

#endif
// #include "shapones/common.hpp"

// #include "shapones/cpu.hpp"

#ifndef SHAPONES_CPU_HPP
#define SHAPONES_CPU_HPP

// #include "shapones/common.hpp"


namespace nes::cpu {

static constexpr int CLOCK_FREQ_NTSC = 1789773;

static constexpr addr_t WRAM_BASE = 0x0;
static constexpr addr_t WRAM_MIRROR_BASE = 0x800;
static constexpr addr_t PPUREG_BASE = 0x2000;
static constexpr addr_t OAM_DMA_REG = 0x4014;
static constexpr addr_t INPUT_REG_0 = 0x4016;
static constexpr addr_t INPUT_REG_1 = 0x4017;

static constexpr addr_t VEC_NMI = 0xfffa;
static constexpr addr_t VEC_RESET = 0xfffc;
static constexpr addr_t VEC_IRQ = 0xfffe;

static constexpr uint8_t STATUS_CARRY = 0x1;
static constexpr uint8_t STATUS_ZERO = 0x2;
static constexpr uint8_t STATUS_INTERRUPT = 0x4;
static constexpr uint8_t STATUS_DECIMALMODE = 0x8;
static constexpr uint8_t STATUS_BREAKMODE = 0x10;
static constexpr uint8_t STATUS_OVERFLOW = 0x40;
static constexpr uint8_t STATUS_NEGATIVE = 0x80;

static constexpr uint16_t DMA_TRANSFER_SIZE = 256;

struct registers_t {
  static constexpr uint32_t STATE_SIZE = 16;

  uint8_t A;   // accumulator
  uint8_t X;   // index
  uint8_t Y;   // index
  uint8_t SP;  // stack pointer (low byte)
  union {      // status
    uint8_t raw;
    struct {
      uint8_t carry : 1;
      uint8_t zero : 1;
      uint8_t interrupt : 1;
      uint8_t decimalmode : 1;
      uint8_t breakmode : 1;
      uint8_t reserved : 1;
      uint8_t overflow : 1;
      uint8_t negative : 1;
    };
  } status;
  addr_t PC;  // program counter

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u8(A);
    writer.u8(X);
    writer.u8(Y);
    writer.u8(SP);
    writer.u8(status.raw);
    writer.u16(PC);
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    A = reader.u8();
    X = reader.u8();
    Y = reader.u8();
    SP = reader.u8();
    status.raw = reader.u8();
    PC = reader.u16();
  }
};

constexpr uint32_t STATE_SIZE = registers_t::STATE_SIZE + 16;

extern volatile cycle_t ppu_cycle_count;
static SHAPONES_INLINE cycle_t ppu_cycle_leading() { return ppu_cycle_count; }

result_t init();
void deinit();

bool is_stopped();

result_t reset();
void stop();

result_t service();

uint8_t bus_read(addr_t addr);
void bus_write(addr_t addr, uint8_t data);

uint32_t get_state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::cpu

#endif
// #include "shapones/host_intf.hpp"

#ifndef SHAPONES_HOST_INTF_HPP
#define SHAPONES_HOST_INTF_HPP

// #include "shapones/common.hpp"


namespace nes {

struct file_info_t {
  bool is_dir;
  const char *name;
};

using fs_enum_files_cb_t = bool (*)(const file_info_t &info);

result_t ram_alloc(size_t size, void **out_ptr);
void ram_free(void *ptr);

result_t spinlock_init(int id);
void spinlock_deinit(int id);
void spinlock_get(int id);
void spinlock_release(int id);

result_t semaphore_init(int id);
void semaphore_deinit(int id);
void semaphore_take(int id);
bool semaphore_try_take(int id);
void semaphore_give(int id);

result_t load_ines(const char *path, const uint8_t **out_ines,
                   size_t *out_size);
void unload_ines();

result_t fs_mount();
void fs_unmount();

result_t fs_get_current_dir(char *out_path);
result_t fs_enum_files(const char *path, fs_enum_files_cb_t callback);
bool fs_exists(const char *path);
result_t fs_open(const char *path, bool write, void **handle);
void fs_close(void *handle);
result_t fs_seek(void *handle, size_t offset);
result_t fs_read(void *handle, uint8_t *buff, size_t size);
result_t fs_write(void *handle, const uint8_t *buff, size_t size);
result_t fs_size(void *handle, size_t *out_size);
result_t fs_delete(const char *path);

}  // namespace nes

#endif
// #include "shapones/input.hpp"

#ifndef SHAPONES_INPUT_HPP
#define SHAPONES_INPUT_HPP

// #include "shapones/common.hpp"


namespace nes::input {

static constexpr int BTN_A = 0;
static constexpr int BTN_B = 1;
static constexpr int BTN_SELECT = 2;
static constexpr int BTN_START = 3;
static constexpr int BTN_UP = 4;
static constexpr int BTN_DOWN = 5;
static constexpr int BTN_LEFT = 6;
static constexpr int BTN_RIGHT = 7;

union status_t {
  uint8_t raw;
  struct {
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t select : 1;
    uint8_t start : 1;
    uint8_t up : 1;
    uint8_t down : 1;
    uint8_t left : 1;
    uint8_t right : 1;
  };
};

union control_t {
  uint8_t raw;
  struct {
    uint8_t strobe : 1;
    uint8_t reserved : 7;
  };
};

result_t init();
void deinit();

result_t reset();

status_t get_status(int player);
void set_status(int player, status_t data);

void update();
uint8_t read_latched(int player);
void write_control(uint8_t data);

uint32_t state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::input

#endif
// #include "shapones/interrupt.hpp"

#ifndef SHAPONES_INTERRUPT_HPP
#define SHAPONES_INTERRUPT_HPP

// #include "shapones/common.hpp"


namespace nes::interrupt {

enum class source_t : uint32_t {
  APU_FRAME_COUNTER = (1 << 0),
  APU_DMC = (1 << 1),
  MAPPER = (1 << 2),
};

static SHAPONES_INLINE source_t operator|(source_t a, source_t b) {
  return static_cast<source_t>(static_cast<uint32_t>(a) |
                             static_cast<uint32_t>(b));
}
static SHAPONES_INLINE source_t operator&(source_t a, source_t b) {
  return static_cast<source_t>(static_cast<uint32_t>(a) &
                             static_cast<uint32_t>(b));
}
static SHAPONES_INLINE bool operator!(source_t a) {
  return !static_cast<uint32_t>(a);
}
static SHAPONES_INLINE source_t operator~(source_t a) {
  return static_cast<source_t>(~static_cast<uint32_t>(a));
}
static SHAPONES_INLINE source_t& operator|=(source_t& a, source_t b) {
  a = a | b;
  return a;
}
static SHAPONES_INLINE source_t& operator&=(source_t& a, source_t b) {
  a = a & b;
  return a;
}

result_t init();
void deinit();

result_t reset();

void assert_irq(source_t src);
void deassert_irq(source_t src);
source_t get_irq();

void assert_nmi();
void deassert_nmi();
bool is_nmi_asserted();

uint32_t get_state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::interrupt

#endif
// #include "shapones/lock.hpp"

#ifndef SHAPONES_LOCK_HPP
#define SHAPONES_LOCK_HPP

// #include "shapones/host_intf.hpp"


namespace nes {

class SpinLockBlock {
 public:
  const int id;
  SHAPONES_INLINE SpinLockBlock(int id) : id(id) { spinlock_get(id); }
  SHAPONES_INLINE ~SpinLockBlock() { spinlock_release(id); }
};

class SemaphoreBlock {
 public:
  const int id;
  SHAPONES_INLINE SemaphoreBlock(int id) : id(id) { semaphore_take(id); }
  SHAPONES_INLINE ~SemaphoreBlock() { semaphore_give(id); }
};

}  // namespace nes

#endif// #include "shapones/mapper.hpp"

#ifndef SHAPONES_MAPPER_HPP
#define SHAPONES_MAPPER_HPP

// #include "shapones/common.hpp"

// #include "shapones/ppu.hpp"

#ifndef SHAPONES_PPU_HPP
#define SHAPONES_PPU_HPP

// #include "shapones/common.hpp"


namespace nes::ppu {

static constexpr cycle_t MAX_DELAY_CYCLES = 128;

static constexpr int TILE_SIZE = 8;
static constexpr int NUM_TILE_X = SCREEN_WIDTH / TILE_SIZE;
static constexpr int NUM_TILE_Y = SCREEN_HEIGHT / TILE_SIZE;
static constexpr int BLOCK_SIZE = TILE_SIZE * 2;

static constexpr int REG_SIZE = 8;
static constexpr addr_t REG_PPUCTRL = 0x2000;
static constexpr addr_t REG_PPUMASK = 0x2001;
static constexpr addr_t REG_PPUSTATUS = 0x2002;
static constexpr addr_t REG_OAMADDR = 0x2003;
static constexpr addr_t REG_OAMDATA = 0x2004;
static constexpr addr_t REG_PPUSCROLL = 0x2005;
static constexpr addr_t REG_PPUADDR = 0x2006;
static constexpr addr_t REG_PPUDATA = 0x2007;

static constexpr int PALETTE_SIZE = 4;
static constexpr int PALETTE_NUM_BANK = 8;
static constexpr int PALETTE_FILE_SIZE = PALETTE_SIZE * PALETTE_NUM_BANK;
static constexpr int PALETTE_FILE_SIZE_WITH_MIRROR = 0x100;

static constexpr uint8_t OPAQUE_FLAG = 0x80;
static constexpr uint8_t BEHIND_FLAG = 0x40;

static constexpr addr_t VRAM_BASE = 0x2000;
static constexpr addr_t VRAM_MIRROR_BASE = 0x3000;
static constexpr addr_t VRAM_MIRROR_SIZE = 0xf00;
static constexpr addr_t PALETTE_FILE_BASE = 0x3f00;

static constexpr addr_t OAM_DMA_SRC_ADDR = 0x200;

static constexpr int MAX_VISIBLE_SPRITES = 8;
static constexpr int SPRITE_Y_OFFSET = 1;

static constexpr int LINE_CYCLES = 341;
static constexpr int SCAN_LINES = 262;

enum class timing_t {
  NONE = 0,
  END_OF_VISIBLE_LINE = (1 << 0),
  END_OF_VISIBLE_AREA = (1 << 1),
  START_OF_VBLANK_LINE = (1 << 2),
  END_OF_FRAME = (1 << 3),
};

static SHAPONES_INLINE timing_t operator|(timing_t a, timing_t b) {
  return static_cast<timing_t>(static_cast<uint32_t>(a) |
                               static_cast<uint32_t>(b));
}
static SHAPONES_INLINE timing_t operator&(timing_t a, timing_t b) {
  return static_cast<timing_t>(static_cast<uint32_t>(a) &
                               static_cast<uint32_t>(b));
}
static SHAPONES_INLINE timing_t &operator|=(timing_t &a, timing_t b) {
  a = a | b;
  return a;
}
static SHAPONES_INLINE bool operator!(timing_t a) {
  return static_cast<uint32_t>(a) == 0;
}

static constexpr int NAME_LINE_STRIDE = SCREEN_WIDTH / 8;
static constexpr int NAME_PAGE_STRIDE = 0x400;

static constexpr int MAX_SPRITE_COUNT = 64;
static constexpr int OAM_SIZE = MAX_SPRITE_COUNT * 4;

static constexpr uint16_t SCROLL_MASK_COARSE_X = 0x001fu;
static constexpr uint16_t SCROLL_MASK_COARSE_Y = 0x03e0u;
static constexpr uint16_t SCROLL_MASK_NAME_SEL = 0x0c00u;
static constexpr uint16_t SCROLL_MASK_FINE_Y = 0x7000u;
static constexpr uint16_t SCROLL_MASK_PPU_ADDR = 0x3fffu;

struct registers_t {
 public:
  static constexpr uint32_t STATE_SIZE = 16;

  // Control Register
  union {
    uint8_t raw;
    struct {
      uint8_t unused : 2;             // [1:0]
      uint8_t incr_stride : 1;        // [2] addr increment (0:+1, 1:+32)
      uint8_t sprite_name_sel : 1;    // [3] sprite pattern sel (0:0x0000,
                                      // 1:0x1000)
      uint8_t bg_name_sel : 1;        // [4] BG pattern sel (0:0x0000, 1:0x1000)
      uint8_t sprite_size : 1;        // [5] sprite size (0:8x8, 1:8x16)
      uint8_t ppu_master : 1;         // [6] ppu master/slave
      uint8_t vblank_nmi_enable : 1;  // [7] vblank interrupt enable
    };
  } control;

  // Mask Register
  union {
    uint8_t raw;
    struct {
      uint8_t color : 1;          // [0] 0:color, 1:mono
      uint8_t bg_clip : 1;        // [1] BG clipping
      uint8_t sprite_clip : 1;    // [2] sprite clipping
      uint8_t bg_enable : 1;      // [3] BG enable
      uint8_t sprite_enable : 1;  // [4] sprite enable
      uint8_t emphasis_r : 1;     // [5] emphasis R
      uint8_t emphasis_g : 1;     // [6] emphasis G
      uint8_t emphasis_b : 1;     // [7] emphasis B
    };
  } mask;

  // Status Register
  union {
    uint8_t raw;
    struct {
      uint8_t reserved : 5;     // [4:0]
      uint8_t overflow : 1;     // [5] overflow (unused)
      uint8_t sprite0_hit : 1;  // [6] sprite 0 hit
      uint8_t vblank_flag : 1;  // [7] vblank flag
    };
  } status;

  // OAM Indirect Address
  uint8_t oam_addr;

  // Scroll Registers
  uint8_t fine_x;
  uint16_t scroll;

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u8(control.raw);
    writer.u8(mask.raw);
    writer.u8(status.raw);
    writer.u8(oam_addr);
    writer.u8(fine_x);
    writer.u16(scroll);
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    control.raw = reader.u8();
    mask.raw = reader.u8();
    status.raw = reader.u8();
    oam_addr = reader.u8();
    fine_x = reader.u8();
    scroll = reader.u16();
  }
};

static constexpr uint8_t OAM_ATTR_PALETTE = 0x3;
static constexpr uint8_t OAM_ATTR_PRIORITY = 0x20;
static constexpr uint8_t OAM_ATTR_INVERT_H = 0x40;
static constexpr uint8_t OAM_ATTR_INVERT_V = 0x80;

static constexpr int OAM_ENTRY_OFFSET_Y = 0;
static constexpr int OAM_ENTRY_OFFSET_TILE = 1;
static constexpr int OAM_ENTRY_OFFSET_ATTR = 2;
static constexpr int OAM_ENTRY_OFFSET_X = 3;

static constexpr uint8_t SL_ATTR_BEHIND = 0x1;
static constexpr uint8_t SL_ATTR_ZERO = 0x2;

struct sprite_line_t {
  uint8_t x;
  uint8_t attr;
  uint16_t chr;
  uint8_t palette_offset;
};

static constexpr int FOCUS_HBLANK = 1;
static constexpr int FOCUS_VBLANK = 2;
static constexpr int FOCUS_1STLINE = 3;

struct status_t {
  int focus_y;
  timing_t timing;
};

extern volatile cycle_t cycle_count;

static SHAPONES_INLINE cycle_t cycle_following() { return cycle_count; }

result_t init();
void deinit();

result_t reset();
bool is_in_hblank();
int current_focus_y();

uint8_t reg_read(addr_t addr);
void reg_write(addr_t addr, uint8_t data);

void oam_dma_write(addr_t offset, uint8_t data);

result_t service(uint8_t *line_buff,  bool skip_render = false, status_t* status = nullptr);

uint32_t get_state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::ppu

#endif

namespace nes::mapper {

class Mapper {
 public:
  const int number;
  const char *name;

  Mapper(int number, const char *name) : number(number), name(name) {}
  virtual ~Mapper() {}

  virtual result_t init() { return result_t::SUCCESS; }
  virtual result_t reset() { return result_t::SUCCESS; }
  virtual bool vblank(const nes::ppu::registers_t &reg) { return false; }
  virtual bool hblank(const nes::ppu::registers_t &reg, int y) { return false; }
  virtual uint8_t read(addr_t addr) { return 0; }
  virtual void write(addr_t addr, uint8_t value) {}

  virtual uint32_t get_state_size() { return 0; }
  virtual result_t save_state(void *file_handle) { return result_t::SUCCESS; }
  virtual result_t load_state(void *file_handle) { return result_t::SUCCESS; }
};

extern Mapper *instance;

result_t init();
void deinit();

result_t map_ines(const uint8_t *ines);

}  // namespace nes::mapper

#endif// #include "shapones/memory.hpp"

#ifndef SHAPONES_MEMORY_HPP
#define SHAPONES_MEMORY_HPP

// #include "shapones/common.hpp"


namespace nes::memory {

static constexpr int PRGROM_PAGE_SIZE = 16384;
static constexpr int CHRROM_PAGE_SIZE = 8192;

static constexpr int PRGROM_BLOCK_ADDR_BITS = 13;
static constexpr int PRGROM_BLOCK_SIZE = 1 << PRGROM_BLOCK_ADDR_BITS;
static constexpr int PRGROM_REMAP_TABLE_SIZE = PRGROM_RANGE / PRGROM_BLOCK_SIZE;

static constexpr int CHRROM_BLOCK_ADDR_BITS = 10;
static constexpr int CHRROM_BLOCK_SIZE = 1 << CHRROM_BLOCK_ADDR_BITS;
static constexpr int CHRROM_REMAP_TABLE_SIZE = CHRROM_RANGE / CHRROM_BLOCK_SIZE;

static constexpr addr_t CHRROM_BASE = 0x0000;
static constexpr addr_t PRGRAM_BASE = 0x6000;
static constexpr addr_t PRGROM_BASE = 0x8000;

static constexpr uint16_t expfwd(uint8_t val) {
  uint_fast16_t tmp = val;
  tmp = ((tmp & 0x00f0) << 4) | (tmp & 0x000f);
  tmp = ((tmp & 0x0c0c) << 2) | (tmp & 0x0303);
  tmp = ((tmp & 0x2222) << 1) | (tmp & 0x1111);
  return tmp;
}

static constexpr uint16_t exprev(uint8_t val) {
  uint_fast16_t tmp = val;
  tmp = ((tmp & 0x000f) << 12) | (tmp & 0x00f0);
  tmp = ((tmp & 0xc0c0) >> 6) | (tmp & 0x3030);
  tmp = ((tmp & 0x1111) << 3) | (tmp & 0x2222);
  return tmp;
}

// clang-format off
static const uint16_t EXPAND_FWD_TABLE[] = {
    expfwd(0x00), expfwd(0x01), expfwd(0x02), expfwd(0x03), expfwd(0x04), expfwd(0x05), expfwd(0x06), expfwd(0x07), expfwd(0x08), expfwd(0x09), expfwd(0x0A), expfwd(0x0B), expfwd(0x0C), expfwd(0x0D), expfwd(0x0E), expfwd(0x0F),
    expfwd(0x10), expfwd(0x11), expfwd(0x12), expfwd(0x13), expfwd(0x14), expfwd(0x15), expfwd(0x16), expfwd(0x17), expfwd(0x18), expfwd(0x19), expfwd(0x1A), expfwd(0x1B), expfwd(0x1C), expfwd(0x1D), expfwd(0x1E), expfwd(0x1F),
    expfwd(0x20), expfwd(0x21), expfwd(0x22), expfwd(0x23), expfwd(0x24), expfwd(0x25), expfwd(0x26), expfwd(0x27), expfwd(0x28), expfwd(0x29), expfwd(0x2A), expfwd(0x2B), expfwd(0x2C), expfwd(0x2D), expfwd(0x2E), expfwd(0x2F),
    expfwd(0x30), expfwd(0x31), expfwd(0x32), expfwd(0x33), expfwd(0x34), expfwd(0x35), expfwd(0x36), expfwd(0x37), expfwd(0x38), expfwd(0x39), expfwd(0x3A), expfwd(0x3B), expfwd(0x3C), expfwd(0x3D), expfwd(0x3E), expfwd(0x3F),
    expfwd(0x40), expfwd(0x41), expfwd(0x42), expfwd(0x43), expfwd(0x44), expfwd(0x45), expfwd(0x46), expfwd(0x47), expfwd(0x48), expfwd(0x49), expfwd(0x4A), expfwd(0x4B), expfwd(0x4C), expfwd(0x4D), expfwd(0x4E), expfwd(0x4F),
    expfwd(0x50), expfwd(0x51), expfwd(0x52), expfwd(0x53), expfwd(0x54), expfwd(0x55), expfwd(0x56), expfwd(0x57), expfwd(0x58), expfwd(0x59), expfwd(0x5A), expfwd(0x5B), expfwd(0x5C), expfwd(0x5D), expfwd(0x5E), expfwd(0x5F),
    expfwd(0x60), expfwd(0x61), expfwd(0x62), expfwd(0x63), expfwd(0x64), expfwd(0x65), expfwd(0x66), expfwd(0x67), expfwd(0x68), expfwd(0x69), expfwd(0x6A), expfwd(0x6B), expfwd(0x6C), expfwd(0x6D), expfwd(0x6E), expfwd(0x6F),
    expfwd(0x70), expfwd(0x71), expfwd(0x72), expfwd(0x73), expfwd(0x74), expfwd(0x75), expfwd(0x76), expfwd(0x77), expfwd(0x78), expfwd(0x79), expfwd(0x7A), expfwd(0x7B), expfwd(0x7C), expfwd(0x7D), expfwd(0x7E), expfwd(0x7F),
    expfwd(0x80), expfwd(0x81), expfwd(0x82), expfwd(0x83), expfwd(0x84), expfwd(0x85), expfwd(0x86), expfwd(0x87), expfwd(0x88), expfwd(0x89), expfwd(0x8A), expfwd(0x8B), expfwd(0x8C), expfwd(0x8D), expfwd(0x8E), expfwd(0x8F),
    expfwd(0x90), expfwd(0x91), expfwd(0x92), expfwd(0x93), expfwd(0x94), expfwd(0x95), expfwd(0x96), expfwd(0x97), expfwd(0x98), expfwd(0x99), expfwd(0x9A), expfwd(0x9B), expfwd(0x9C), expfwd(0x9D), expfwd(0x9E), expfwd(0x9F),
    expfwd(0xA0), expfwd(0xA1), expfwd(0xA2), expfwd(0xA3), expfwd(0xA4), expfwd(0xA5), expfwd(0xA6), expfwd(0xA7), expfwd(0xA8), expfwd(0xA9), expfwd(0xAA), expfwd(0xAB), expfwd(0xAC), expfwd(0xAD), expfwd(0xAE), expfwd(0xAF),
    expfwd(0xB0), expfwd(0xB1), expfwd(0xB2), expfwd(0xB3), expfwd(0xB4), expfwd(0xB5), expfwd(0xB6), expfwd(0xB7), expfwd(0xB8), expfwd(0xB9), expfwd(0xBA), expfwd(0xBB), expfwd(0xBC), expfwd(0xBD), expfwd(0xBE), expfwd(0xBF),
    expfwd(0xC0), expfwd(0xC1), expfwd(0xC2), expfwd(0xC3), expfwd(0xC4), expfwd(0xC5), expfwd(0xC6), expfwd(0xC7), expfwd(0xC8), expfwd(0xC9), expfwd(0xCA), expfwd(0xCB), expfwd(0xCC), expfwd(0xCD), expfwd(0xCE), expfwd(0xCF),
    expfwd(0xD0), expfwd(0xD1), expfwd(0xD2), expfwd(0xD3), expfwd(0xD4), expfwd(0xD5), expfwd(0xD6), expfwd(0xD7), expfwd(0xD8), expfwd(0xD9), expfwd(0xDA), expfwd(0xDB), expfwd(0xDC), expfwd(0xDD), expfwd(0xDE), expfwd(0xDF),
    expfwd(0xE0), expfwd(0xE1), expfwd(0xE2), expfwd(0xE3), expfwd(0xE4), expfwd(0xE5), expfwd(0xE6), expfwd(0xE7), expfwd(0xE8), expfwd(0xE9), expfwd(0xEA), expfwd(0xEB), expfwd(0xEC), expfwd(0xED), expfwd(0xEE), expfwd(0xEF),
    expfwd(0xF0), expfwd(0xF1), expfwd(0xF2), expfwd(0xF3), expfwd(0xF4), expfwd(0xF5), expfwd(0xF6), expfwd(0xF7), expfwd(0xF8), expfwd(0xF9), expfwd(0xFA), expfwd(0xFB), expfwd(0xFC), expfwd(0xFD), expfwd(0xFE), expfwd(0xFF),
};

static const uint16_t EXPAND_REV_TABLE[] = {
    exprev(0x00), exprev(0x01), exprev(0x02), exprev(0x03), exprev(0x04), exprev(0x05), exprev(0x06), exprev(0x07), exprev(0x08), exprev(0x09), exprev(0x0A), exprev(0x0B), exprev(0x0C), exprev(0x0D), exprev(0x0E), exprev(0x0F),
    exprev(0x10), exprev(0x11), exprev(0x12), exprev(0x13), exprev(0x14), exprev(0x15), exprev(0x16), exprev(0x17), exprev(0x18), exprev(0x19), exprev(0x1A), exprev(0x1B), exprev(0x1C), exprev(0x1D), exprev(0x1E), exprev(0x1F),
    exprev(0x20), exprev(0x21), exprev(0x22), exprev(0x23), exprev(0x24), exprev(0x25), exprev(0x26), exprev(0x27), exprev(0x28), exprev(0x29), exprev(0x2A), exprev(0x2B), exprev(0x2C), exprev(0x2D), exprev(0x2E), exprev(0x2F),
    exprev(0x30), exprev(0x31), exprev(0x32), exprev(0x33), exprev(0x34), exprev(0x35), exprev(0x36), exprev(0x37), exprev(0x38), exprev(0x39), exprev(0x3A), exprev(0x3B), exprev(0x3C), exprev(0x3D), exprev(0x3E), exprev(0x3F),
    exprev(0x40), exprev(0x41), exprev(0x42), exprev(0x43), exprev(0x44), exprev(0x45), exprev(0x46), exprev(0x47), exprev(0x48), exprev(0x49), exprev(0x4A), exprev(0x4B), exprev(0x4C), exprev(0x4D), exprev(0x4E), exprev(0x4F),
    exprev(0x50), exprev(0x51), exprev(0x52), exprev(0x53), exprev(0x54), exprev(0x55), exprev(0x56), exprev(0x57), exprev(0x58), exprev(0x59), exprev(0x5A), exprev(0x5B), exprev(0x5C), exprev(0x5D), exprev(0x5E), exprev(0x5F),
    exprev(0x60), exprev(0x61), exprev(0x62), exprev(0x63), exprev(0x64), exprev(0x65), exprev(0x66), exprev(0x67), exprev(0x68), exprev(0x69), exprev(0x6A), exprev(0x6B), exprev(0x6C), exprev(0x6D), exprev(0x6E), exprev(0x6F),
    exprev(0x70), exprev(0x71), exprev(0x72), exprev(0x73), exprev(0x74), exprev(0x75), exprev(0x76), exprev(0x77), exprev(0x78), exprev(0x79), exprev(0x7A), exprev(0x7B), exprev(0x7C), exprev(0x7D), exprev(0x7E), exprev(0x7F),
    exprev(0x80), exprev(0x81), exprev(0x82), exprev(0x83), exprev(0x84), exprev(0x85), exprev(0x86), exprev(0x87), exprev(0x88), exprev(0x89), exprev(0x8A), exprev(0x8B), exprev(0x8C), exprev(0x8D), exprev(0x8E), exprev(0x8F),
    exprev(0x90), exprev(0x91), exprev(0x92), exprev(0x93), exprev(0x94), exprev(0x95), exprev(0x96), exprev(0x97), exprev(0x98), exprev(0x99), exprev(0x9A), exprev(0x9B), exprev(0x9C), exprev(0x9D), exprev(0x9E), exprev(0x9F),
    exprev(0xA0), exprev(0xA1), exprev(0xA2), exprev(0xA3), exprev(0xA4), exprev(0xA5), exprev(0xA6), exprev(0xA7), exprev(0xA8), exprev(0xA9), exprev(0xAA), exprev(0xAB), exprev(0xAC), exprev(0xAD), exprev(0xAE), exprev(0xAF),
    exprev(0xB0), exprev(0xB1), exprev(0xB2), exprev(0xB3), exprev(0xB4), exprev(0xB5), exprev(0xB6), exprev(0xB7), exprev(0xB8), exprev(0xB9), exprev(0xBA), exprev(0xBB), exprev(0xBC), exprev(0xBD), exprev(0xBE), exprev(0xBF),
    exprev(0xC0), exprev(0xC1), exprev(0xC2), exprev(0xC3), exprev(0xC4), exprev(0xC5), exprev(0xC6), exprev(0xC7), exprev(0xC8), exprev(0xC9), exprev(0xCA), exprev(0xCB), exprev(0xCC), exprev(0xCD), exprev(0xCE), exprev(0xCF),
    exprev(0xD0), exprev(0xD1), exprev(0xD2), exprev(0xD3), exprev(0xD4), exprev(0xD5), exprev(0xD6), exprev(0xD7), exprev(0xD8), exprev(0xD9), exprev(0xDA), exprev(0xDB), exprev(0xDC), exprev(0xDD), exprev(0xDE), exprev(0xDF),
    exprev(0xE0), exprev(0xE1), exprev(0xE2), exprev(0xE3), exprev(0xE4), exprev(0xE5), exprev(0xE6), exprev(0xE7), exprev(0xE8), exprev(0xE9), exprev(0xEA), exprev(0xEB), exprev(0xEC), exprev(0xED), exprev(0xEE), exprev(0xEF),
    exprev(0xF0), exprev(0xF1), exprev(0xF2), exprev(0xF3), exprev(0xF4), exprev(0xF5), exprev(0xF6), exprev(0xF7), exprev(0xF8), exprev(0xF9), exprev(0xFA), exprev(0xFB), exprev(0xFC), exprev(0xFD), exprev(0xFE), exprev(0xFF),
};
// clang-format on

extern uint8_t wram[WRAM_SIZE];
extern uint8_t vram[VRAM_SIZE];
extern addr_t vram_addr_and;
extern addr_t vram_addr_or;

extern uint8_t *prgram;
extern uint8_t *chrram;
extern const uint8_t *prgrom;
extern const uint8_t *chrrom;

extern uint16_t prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
extern uint16_t chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

extern uint32_t prgrom_phys_size;
extern uint32_t prgrom_phys_addr_mask;
extern uint32_t prgram_size;
extern addr_t prgram_addr_mask;

extern uint32_t chrrom_phys_size;
extern uint32_t chrrom_phys_addr_mask;

result_t init();
void deinit();

result_t map_ines(const uint8_t *ines);
void unmap_ines();

static SHAPONES_INLINE uint8_t vram_read(addr_t addr) {
  return vram[(addr & vram_addr_and) | vram_addr_or];
}

static SHAPONES_INLINE void vram_write(addr_t addr, uint8_t value) {
  vram[(addr & vram_addr_and) | vram_addr_or] = value;
}

static SHAPONES_INLINE void prgrom_remap(addr_t cpu_base, uint32_t phys_base,
                                         uint32_t size) {
  uint32_t cpu_block = (cpu_base - PRGROM_BASE) >> PRGROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = phys_base >> PRGROM_BLOCK_ADDR_BITS;
  uint32_t num_blocks = size >> PRGROM_BLOCK_ADDR_BITS;
  for (int i = 0; i < num_blocks; i++) {
    prgrom_remap_table[cpu_block + i] = phys_block + i;
  }
}

static SHAPONES_INLINE void chrrom_remap(addr_t ppu_base, uint32_t phys_base,
                                         uint32_t size) {
  uint32_t ppu_block = (ppu_base - CHRROM_BASE) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = phys_base >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t num_blocks = size >> CHRROM_BLOCK_ADDR_BITS;
  for (int i = 0; i < num_blocks; i++) {
    chrrom_remap_table[ppu_block + i] = phys_block + i;
  }
}

static SHAPONES_INLINE uint8_t prgrom_read(addr_t addr) {
  uint32_t cpu_block = (addr & (PRGROM_RANGE - 1)) >> PRGROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = prgrom_remap_table[cpu_block];
  uint32_t phys_addr =
      (phys_block << PRGROM_BLOCK_ADDR_BITS) + (addr & (PRGROM_BLOCK_SIZE - 1));
  return prgrom[phys_addr & prgrom_phys_addr_mask];
}

static SHAPONES_INLINE uint8_t prgram_read(addr_t addr) {
  return prgram[addr & prgram_addr_mask];
}

static SHAPONES_INLINE void prgram_write(addr_t addr, uint8_t value) {
  prgram[addr & prgram_addr_mask] = value;
}

static SHAPONES_INLINE uint8_t chrrom_read(addr_t addr) {
  uint32_t ppu_block = (addr & (CHRROM_RANGE - 1)) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = chrrom_remap_table[ppu_block];
  uint32_t phys_addr =
      (phys_block << CHRROM_BLOCK_ADDR_BITS) + (addr & (CHRROM_BLOCK_SIZE - 1));
  return chrrom[phys_addr & chrrom_phys_addr_mask];
}

static SHAPONES_INLINE void chrram_write(addr_t addr, uint8_t value) {
  uint32_t ppu_block = (addr & (CHRROM_RANGE - 1)) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = chrrom_remap_table[ppu_block];
  uint32_t phys_addr =
      (phys_block << CHRROM_BLOCK_ADDR_BITS) + (addr & (CHRROM_BLOCK_SIZE - 1));
  chrram[phys_addr & chrrom_phys_addr_mask] = value;
}

static SHAPONES_INLINE uint_fast16_t chrrom_read_double(addr_t addr,
                                                        bool reverse) {
  uint32_t ppu_block = (addr & (CHRROM_RANGE - 1)) >> CHRROM_BLOCK_ADDR_BITS;
  uint32_t phys_block = chrrom_remap_table[ppu_block];
  uint32_t phys_addr =
      (phys_block << CHRROM_BLOCK_ADDR_BITS) + (addr & (CHRROM_BLOCK_SIZE - 1));

  uint8_t lo = chrrom[phys_addr & chrrom_phys_addr_mask];
  uint8_t hi = chrrom[(phys_addr & chrrom_phys_addr_mask) + 8];
  if (reverse) {
    return (EXPAND_FWD_TABLE[hi] << 1) | EXPAND_FWD_TABLE[lo];
  } else {
    return EXPAND_REV_TABLE[hi] | (EXPAND_REV_TABLE[lo] >> 1);
  }
}

void set_nametable_arrangement(nametable_arrangement_t mode);

uint32_t get_state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::memory

#endif
// #include "shapones/menu.hpp"

#ifndef SHAPONES_MENU_HPP
#define SHAPONES_MENU_HPP

// #include "shapones/common.hpp"


namespace nes::menu {

extern bool shown;

static SHAPONES_INLINE bool is_shown() { return shown; }

result_t init();
void deinit();

void show();
void hide();

result_t service();
result_t overlay(int y, uint8_t *line_buff);

}  // namespace nes::menu

#endif
// #include "shapones/ppu.hpp"


namespace nes {

config_t get_default_config();

result_t init(const config_t &cfg);
void deinit();

result_t reset();

result_t render_next_line(uint8_t *line_buff, bool skip_render = false,
                          ppu::status_t *status = nullptr);
result_t vsync(uint8_t *line_buff, bool skip_render = false);

}  // namespace nes

#endif
#ifdef SHAPONES_IMPLEMENTATION

// #include "shapones/common.hpp"


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
    case result_t::ERR_RAM_ALLOC_FAILED: return "No RAM";
    case result_t::ERR_INES_INVALID_FORMAT: return "Bad iNES";
    case result_t::ERR_INES_TOO_LARGE: return "iNES Too Large";
    case result_t::ERR_INES_NOT_LOADED: return "iNES Not Loaded";
    case result_t::ERR_FS_NO_DISK: return "No Disk";
    case result_t::ERR_FS_DIR_NOT_FOUND: return "Dir Not Found";
    case result_t::ERR_FS_PATH_TOO_LONG: return "Path Too Long";
    case result_t::ERR_FS_OPEN_FAILED: return "Open Failed";
    case result_t::ERR_FS_FILE_NOT_OPEN: return "File Not Open";
    case result_t::ERR_FS_SEEK_FAILED: return "Seek Failed";
    case result_t::ERR_FS_READ_FAILED: return "Read Failed";
    case result_t::ERR_FS_WRITE_FAILED: return "Write Failed";
    case result_t::ERR_FS_DELETE_FAILED: return "Delete Failed";
    case result_t::ERR_STATE_INVALID_FORMAT: return "Bad State Format";
    case result_t::ERR_STATE_SIZE_MISMATCH: return "Bad State Size";
    case result_t::ERR_STATE_SLOT_FULL: return "State Slot Full";
    case result_t::ERR_FLASH_ERASE_FAILED: return "Erase Failed";
    case result_t::ERR_FLASH_PROGRAM_FAILED: return "Flash Failed";
    case result_t::ERR_STATE_NO_SLOT_DATA: return "No Slot Data";
    case result_t::ERR_MMAP_FAILED: return "MMap Failed";
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
// #include "shapones/apu.hpp"

// #include "shapones/cpu.hpp"

// #include "shapones/fifo.hpp"

#ifndef SHAPONES_FIFO_HPP
#define SHAPONES_FIFO_HPP

// #include "shapones/common.hpp"


namespace nes {

template <typename T, uint32_t prm_ADDR_BITS>
class AsyncFifo {
 public:
  static constexpr uint32_t ADDR_BITS = prm_ADDR_BITS;
  static constexpr uint32_t CAPACITY = 1 << ADDR_BITS;

 private:
  T buffer[CAPACITY];
  volatile uint32_t rd_ptr;
  volatile uint32_t wr_ptr;

 public:
  AsyncFifo() : rd_ptr(0), wr_ptr(0) {}

  void clear() {
    rd_ptr = 0;
    wr_ptr = 0;
  }

  SHAPONES_INLINE bool is_empty() const { return rd_ptr == wr_ptr; }

  SHAPONES_INLINE bool is_full() const {
    uint32_t wp = (wr_ptr + 1) & (CAPACITY - 1);
    return wp == rd_ptr;
  }

  SHAPONES_INLINE bool try_push(const T &item) {
    uint32_t wp = wr_ptr;
    uint32_t wp_next = (wp + 1) & (CAPACITY - 1);
    if (wp_next == rd_ptr) {
      return false;  // full
    }
    buffer[wp] = item;
    SHAPONES_THREAD_FENCE_SEQ_CST();
    wr_ptr = wp_next;
    return true;
  }

  SHAPONES_INLINE void push_blocking(const T &item) {
    uint32_t wp, wp_next;
    do {
      wp = wr_ptr;
      wp_next = (wp + 1) & (CAPACITY - 1);
    } while (wp_next == rd_ptr);  // wait until not full
    SHAPONES_THREAD_FENCE_SEQ_CST();
    buffer[wp] = item;
    wr_ptr = wp_next;
  }

  SHAPONES_INLINE bool try_peek(T *out_item) {
    uint32_t rp = rd_ptr;
    if (rp == wr_ptr) {
      return false;  // empty
    }
    *out_item = buffer[rp];
    return true;
  }

  SHAPONES_INLINE bool try_pop(T *out_item) {
    uint32_t rp = rd_ptr;
    if (rp == wr_ptr) {
      return false;  // empty
    }
    *out_item = buffer[rp];
    SHAPONES_THREAD_FENCE_SEQ_CST();
    rd_ptr = (rp + 1) & (CAPACITY - 1);
    return true;
  }

  SHAPONES_INLINE T pop_blocking() {
    uint32_t rp;
    do {
      rp = rd_ptr;
    } while (rp == wr_ptr);  // wait until not empty
    T item = buffer[rp];
    SHAPONES_THREAD_FENCE_SEQ_CST();
    rd_ptr = (rp + 1) & (CAPACITY - 1);
    return item;
  }
};
}  // namespace nes

#endif
// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

// #include "shapones/menu.hpp"


namespace nes::apu {

static constexpr int QUARTER_FRAME_FREQUENCY = 240;
static constexpr int QUARTER_FRAME_PHASE_PREC = 16;
static constexpr uint32_t QUARTER_FRAME_PHASE_PERIOD =
    1ul << QUARTER_FRAME_PHASE_PREC;

static constexpr uint8_t STEP_FLAG_FRAME = 0x01;
static constexpr uint8_t STEP_FLAG_HALF = 0x02;
static constexpr uint8_t STEP_FLAG_QUARTER = 0x04;

// see: https://www.nesdev.org/wiki/APU_Length_Counter
static const uint8_t LENGTH_TABLE[] = {
    10, 254, 20, 2,  40, 4,  80, 6,  160, 8,  60, 10, 14, 12, 26, 14,
    12, 16,  24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30,
};

// see: https://www.nesdev.org/wiki/APU_Noise
static const uint16_t NOISE_PERIOD_TABLE[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
};

// see: https://www.nesdev.org/wiki/APU_DMC
static const uint16_t DMC_RATE_TABLE[16] = {
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
};

static constexpr uint32_t STATE_SIZE =
    pulse_state_t::STATE_SIZE * 2 + triangle_state_t::STATE_SIZE +
    noise_state_t::STATE_SIZE + dmc_state_t::STATE_SIZE + 16;

static uint32_t sampling_rate;
static uint32_t pulse_timer_step;
static uint32_t triangle_timer_step;
static uint32_t noise_timer_step;
static uint32_t dmc_step_coeff;
static uint32_t quarter_frame_phase_step;

static uint32_t quarter_frame_phase;
static uint8_t quarter_frame_count;
static uint8_t frame_step_flags;
static status_t status;

static pulse_state_t pulse[2];
static triangle_state_t triangle;
static noise_state_t noise;
static dmc_state_t dmc;

static AsyncFifo<reg_write_t, 6> write_queue;

static void flush_write_queue();

static void pulse_write_reg0(pulse_state_t &s, uint8_t value);
static void pulse_write_reg1(pulse_state_t &s, uint8_t value);
static void pulse_write_reg2(pulse_state_t &s, uint8_t value);
static void pulse_write_reg3(pulse_state_t &s, uint8_t value);

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  for (int i = 0; i < 2; i++) {
    memset(&pulse[i], 0, sizeof(pulse_state_t));
  }
  memset(&triangle, 0, sizeof(triangle_state_t));
  memset(&noise, 0, sizeof(noise_state_t));
  noise.lfsr = 0x7FFF;
  memset(&dmc, 0, sizeof(dmc_state_t));
  status.raw = 0;
  quarter_frame_phase = 0;
  quarter_frame_count = 0;
  frame_step_flags = 0;

  write_queue.clear();

  return result_t::SUCCESS;
}

uint8_t reg_read(addr_t addr) {
  if (!write_queue.is_empty()) {
    SemaphoreBlock block(SEMAPHORE_APU);
    flush_write_queue();
  }

  switch (addr) {
    case REG_STATUS: {
      // see: https://www.nesdev.org/wiki/IRQ
      uint8_t ret = status.raw;
      auto irqs = interrupt::get_irq();
      if (!!(irqs & interrupt::source_t::APU_DMC)) {
        ret |= 0x80;
      }
      if (!!(irqs & interrupt::source_t::APU_FRAME_COUNTER)) {
        ret |= 0x40;
      }
      interrupt::deassert_irq(interrupt::source_t::APU_FRAME_COUNTER);
      return ret;
    } break;

    default: return 0;
  }
}

void reg_write(addr_t addr, uint8_t data) {
  reg_write_t req;
  req.addr = addr;
  req.data = data;
  if (!write_queue.try_push(req)) {
    SemaphoreBlock block(SEMAPHORE_APU);
    flush_write_queue();
    SHAPONES_THREAD_FENCE_SEQ_CST();
    write_queue.push_blocking(req);
    SHAPONES_THREAD_FENCE_SEQ_CST();
  }
}

static SHAPONES_INLINE void pulse_write_reg0(pulse_state_t &s, uint8_t value) {
  //  duty pattern
  switch ((value >> 6) & 0x3) {
    default:
    case 0: s.waveform = 0b00000001; break;
    case 1: s.waveform = 0b00000011; break;
    case 2: s.waveform = 0b00001111; break;
    case 3: s.waveform = 0b11111100; break;
  }

  s.envelope.flags = 0;
  if (value & 0x10) s.envelope.flags |= ENV_FLAG_CONSTANT;
  if (value & 0x20) s.envelope.flags |= ENV_FLAG_HALT_LOOP;
  s.envelope.volume = value & 0xf;

  if (!(s.envelope.flags & ENV_FLAG_CONSTANT)) {
    s.envelope.flags |= ENV_FLAG_START;
    s.envelope.divider = s.envelope.volume;
  }
}

static SHAPONES_INLINE void pulse_write_reg1(pulse_state_t &s, uint8_t value) {
  s.sweep.period = (value >> 4) & 0x7;
  s.sweep.shift = value & 0x7;
  s.sweep.flags = 0;
  if (value & 0x80) s.sweep.flags |= SWP_FLAG_ENABLE;
  if (value & 0x08) s.sweep.flags |= SWP_FLAG_NEGATE;
  s.sweep.divider = 0;
}

static SHAPONES_INLINE void pulse_write_reg2(pulse_state_t &s, uint8_t value) {
  s.timer_period &= ~(0xff << TIMER_PREC);
  s.timer_period |= (uint32_t)value << TIMER_PREC;
}

static SHAPONES_INLINE void pulse_write_reg3(pulse_state_t &s, uint8_t value) {
  s.timer_period &= ~(0x700 << TIMER_PREC);
  s.timer_period |= (uint32_t)(value & 0x7) << (TIMER_PREC + 8);
  s.timer = 0;
  s.length = LENGTH_TABLE[(value >> 3) & 0x1f];
  s.phase = 0;
}

static void flush_write_queue() {
  reg_write_t req;
  while (write_queue.try_peek(&req)) {
    addr_t addr = req.addr;
    uint8_t data = req.data;
    switch (addr) {
      case REG_PULSE1_REG0: pulse_write_reg0(pulse[0], data); break;
      case REG_PULSE1_REG1: pulse_write_reg1(pulse[0], data); break;
      case REG_PULSE1_REG2: pulse_write_reg2(pulse[0], data); break;
      case REG_PULSE1_REG3: pulse_write_reg3(pulse[0], data); break;

      case REG_PULSE2_REG0: pulse_write_reg0(pulse[1], data); break;
      case REG_PULSE2_REG1: pulse_write_reg1(pulse[1], data); break;
      case REG_PULSE2_REG2: pulse_write_reg2(pulse[1], data); break;
      case REG_PULSE2_REG3: pulse_write_reg3(pulse[1], data); break;

      case REG_TRIANGLE_REG0:
        triangle.linear.reload_value = data & 0x7f;
        if (data & 0x80) {
          triangle.linear.flags |= LIN_FLAG_CONTROL;
        } else {
          triangle.linear.flags &= ~LIN_FLAG_CONTROL;
        }
        break;

      case REG_TRIANGLE_REG2:
        triangle.timer_period &= ~(0xff << TIMER_PREC);
        triangle.timer_period |= (uint32_t)data << TIMER_PREC;
        break;

      case REG_TRIANGLE_REG3:
        triangle.timer_period &= ~(0x700 << TIMER_PREC);
        triangle.timer_period |= (uint32_t)(data & 0x7) << (TIMER_PREC + 8);
        triangle.timer = 0;
        triangle.length = LENGTH_TABLE[(data >> 3) & 0x1f];
        triangle.phase = 0;
        triangle.linear.flags |= LIN_FLAG_RELOAD;
        break;

      case REG_NOISE_REG0:
        noise.envelope.flags = 0;
        if (data & 0x10) noise.envelope.flags |= ENV_FLAG_CONSTANT;
        if (data & 0x20) noise.envelope.flags |= ENV_FLAG_HALT_LOOP;
        noise.envelope.volume = data & 0xf;
        if (!(noise.envelope.flags & ENV_FLAG_CONSTANT)) {
          noise.envelope.flags |= ENV_FLAG_START;
          noise.envelope.divider = noise.envelope.volume;
        }
        break;

      case REG_NOISE_REG2:
        if (data & 0x80) {
          noise.flags |= NOISE_FLAG_FB_MODE;
        } else {
          noise.flags &= ~NOISE_FLAG_FB_MODE;
        }
        noise.timer_period = NOISE_PERIOD_TABLE[data & 0xF];
        break;

      case REG_NOISE_REG3:
        noise.length = LENGTH_TABLE[(data >> 3) & 0x1f];
        break;

      case REG_DMC_REG0: {
        bool irq_ena_old = !!(dmc.flags & DMC_FLAG_IRQ_ENABLE);
        bool irq_ena_new = !!(data & 0x80);
        if (irq_ena_new && !irq_ena_old) {
          interrupt::deassert_irq(interrupt::source_t::APU_DMC);
        }
        dmc.flags = 0;
        if (irq_ena_new) dmc.flags |= DMC_FLAG_IRQ_ENABLE;
        if (data & 0x40) dmc.flags |= DMC_FLAG_LOOP;
        dmc.timer_step = dmc_step_coeff / DMC_RATE_TABLE[data & 0x0f];
      } break;

      case REG_DMC_REG1: dmc.out_level = data & 0x7f; break;

      case REG_DMC_REG2: dmc.sample_addr = 0xc000 + ((addr_t)data << 6); break;
      case REG_DMC_REG3: dmc.sample_length = ((uint16_t)data << 4) + 1; break;

      case REG_STATUS:
        status.raw = data;
        if (!status.pulse0_enable) pulse[0].length = 0;
        if (!status.pulse1_enable) pulse[1].length = 0;
        if (!status.triangle_enable) triangle.length = 0;
        if (!status.noise_enable) noise.length = 0;
        if (status.dmc_enable) {
          if (dmc.bytes_remaining == 0) {
            dmc.addr_counter = dmc.sample_addr;
            dmc.bytes_remaining = dmc.sample_length;
          }
        } else {
          dmc.bytes_remaining = 0;
        }
        // see: https://www.nesdev.org/wiki/IRQ
        interrupt::deassert_irq(interrupt::source_t::APU_DMC);
        break;

      case REG_FRAME_COUNTER:
        if (data & 0x80) {
          // todo: implement
        } else {
          // todo: implement
        }
        if (data & 0x40) {
          quarter_frame_count = 0;
          quarter_frame_phase = 0;
          frame_step_flags =
              STEP_FLAG_FRAME | STEP_FLAG_HALF | STEP_FLAG_QUARTER;
        }
        break;
    }

    write_queue.try_pop(&req);
  }
}

result_t set_sampling_rate(uint32_t rate_hz) {
  sampling_rate = rate_hz;
  pulse_timer_step =
      (1ULL << TIMER_PREC) * cpu::CLOCK_FREQ_NTSC / sampling_rate / 2;
  triangle_timer_step =
      (1ULL << TIMER_PREC) * cpu::CLOCK_FREQ_NTSC / sampling_rate;
  noise_timer_step = cpu::CLOCK_FREQ_NTSC / sampling_rate;
  quarter_frame_phase_step =
      (QUARTER_FRAME_FREQUENCY * QUARTER_FRAME_PHASE_PERIOD) / rate_hz;
  dmc_step_coeff = ((uint64_t)cpu::CLOCK_FREQ_NTSC << TIMER_PREC) / rate_hz;
  return result_t::SUCCESS;
}

// see: https://www.nesdev.org/wiki/APU_Envelope
static SHAPONES_INLINE uint8_t step_envelope(envelope_t &e) {
  if (frame_step_flags & STEP_FLAG_QUARTER) {
    if (e.flags & ENV_FLAG_START) {
      e.flags &= ~ENV_FLAG_START;  // clear start flag
      e.decay = 15;
      e.divider = e.volume;
    } else if (e.divider > 0) {
      e.divider--;
    } else {
      e.divider = e.volume;
      if (e.decay > 0) {
        e.decay--;
      } else if (e.flags & ENV_FLAG_HALT_LOOP) {
        e.decay = 15;
      }
    }
  }

  if (e.flags & ENV_FLAG_CONSTANT) {
    return e.volume;
  } else {
    return e.decay;
  }
}

// see: https://www.nesdev.org/wiki/APU_Pulse
// see: https://www.nesdev.org/wiki/APU_Sweep
static SHAPONES_INLINE int sample_pulse(pulse_state_t &s) {
  // envelope unit
  uint_fast8_t vol = step_envelope(s.envelope);

  bool half_frame = !!(frame_step_flags & STEP_FLAG_HALF);

  // sweep unit
  if (half_frame && !!(s.sweep.flags & SWP_FLAG_ENABLE)) {
    if (s.sweep.divider < s.sweep.period) {
      s.sweep.divider++;
    } else {
      s.sweep.divider = 0;
      int32_t change = s.timer_period;
      int32_t target = s.timer_period;
      change >>= s.sweep.shift;
      if (s.sweep.flags & SWP_FLAG_NEGATE) {
        change = -change;
      }
      target += change;
      if (target < 0) {
        target = 0;
      } else if (target >= (0x7ff << TIMER_PREC)) {
        target = (0x7ff << TIMER_PREC);
        vol = 0;
      }
      s.timer_period = target;
    }
  }

  // mute
  if (s.timer_period < 8 || s.length <= 0 || vol == 0) {
    return 0;
  }

  // length counter
  if (half_frame && !(s.envelope.flags & ENV_FLAG_HALT_LOOP)) {
    if (s.length > 0) {
      s.length--;
    }
  }

  // sequencer
  s.timer += pulse_timer_step;
  int step = 0;
  if (s.timer_period != 0) {
    step = s.timer / s.timer_period;
  }
  s.timer -= step * s.timer_period;
  s.phase = (s.phase + step) & 0x7;

  if ((s.waveform >> s.phase) & 1) {
    return vol;
  } else {
    return 0;
  }
}

// see: https://www.nesdev.org/wiki/APU_Triangle
static SHAPONES_INLINE int sample_triangle(triangle_state_t &s) {
  // mute
  if (s.length <= 0) {
    return 0;
  }

  // length counter
  if ((frame_step_flags & STEP_FLAG_HALF) &&
      !(s.linear.flags & LIN_FLAG_CONTROL)) {
    if (s.length > 0) {
      s.length--;
    }
  }

  // linear counter
  if (frame_step_flags & STEP_FLAG_QUARTER) {
    if (s.linear.flags & LIN_FLAG_RELOAD) {
      s.linear.counter = s.linear.reload_value;
    } else if (s.linear.counter > 0) {
      s.linear.counter--;
    }
    if (!(s.linear.flags & LIN_FLAG_CONTROL)) {
      s.linear.flags &= ~LIN_FLAG_RELOAD;  // clear reload flag
    }
  }

  // phase counter
  s.timer += triangle_timer_step;
  int step = 0;
  if (s.timer_period != 0) {
    step = s.timer / s.timer_period;
  }
  s.timer -= step * s.timer_period;
  if (s.length > 0 && s.linear.counter > 0) {
    s.phase = (s.phase + step) & 0x1f;
  }
  uint32_t phase = s.phase;

  // sequencer
  if (phase <= 15) {
    return (15 - phase);
  } else {
    return (phase - 16);
  }
}

// see: https://www.nesdev.org/wiki/APU_Noise
static SHAPONES_INLINE int sample_noise(noise_state_t &s) {
  // envelope unit
  uint_fast8_t vol = step_envelope(s.envelope);

  // length counter
  if ((frame_step_flags & STEP_FLAG_HALF) &&
      !(s.envelope.flags & ENV_FLAG_HALT_LOOP)) {
    if (s.length > 0) {
      s.length--;
    }
  }

  // mute
  if (s.length <= 0 || vol == 0) {
    return 0;
  }

  // timer
  s.timer += noise_timer_step;
  int step = 0;
  if (s.timer_period != 0) {
    step = s.timer / s.timer_period;
  }
  s.timer -= step * s.timer_period;

  // LFSR
  for (int i = 0; i < step; i++) {
    uint32_t fb;
    if (s.flags & NOISE_FLAG_FB_MODE) {
      fb = ((s.lfsr >> 6) ^ (s.lfsr >> 0)) & 1;
    } else {
      fb = ((s.lfsr >> 1) ^ (s.lfsr >> 0)) & 1;
    }
    s.lfsr = (s.lfsr >> 1) | (fb << 14);
  }
  return ((s.lfsr & 1) == 0) ? (vol >> 1) : 0;
}

// see: https://www.nesdev.org/wiki/APU_DMC
static SHAPONES_INLINE int sample_dmc(dmc_state_t &s) {
  s.timer += s.timer_step;
  uint32_t step = s.timer >> TIMER_PREC;
  s.timer &= (1 << TIMER_PREC) - 1;

  for (uint32_t i = 0; i < step; i++) {
    if (s.bits_remaining == 0) {
      if (s.bytes_remaining > 0) {
        s.shift_reg = cpu::bus_read(s.addr_counter | 0x8000);
        s.addr_counter++;
        s.bytes_remaining--;
        if (s.bytes_remaining == 0) {
          if (s.flags & DMC_FLAG_LOOP) {
            s.addr_counter = s.sample_addr;
            s.bytes_remaining = s.sample_length;
          } else {
            status.dmc_enable = 0;
            if (s.flags & DMC_FLAG_IRQ_ENABLE) {
              interrupt::assert_irq(interrupt::source_t::APU_DMC);
            }
          }
        }
      }
      s.bits_remaining = 8;
    }
    if (s.bits_remaining > 0) {
      if (status.dmc_enable) {
        if (s.shift_reg & 1) {
          if (s.out_level <= 125) {
            s.out_level += 2;
          }
        } else {
          if (s.out_level >= 2) {
            s.out_level -= 2;
          }
        }
        s.shift_reg >>= 1;
      }
      s.bits_remaining--;
    }
  }

  return s.out_level;
}

result_t service(uint8_t *buff, int len) {
  if (!semaphore_try_take(SEMAPHORE_APU)) {
    for (int i = 0; i < len; i++) {
      buff[i] = 0;
    }
    return result_t::SUCCESS;
  }

  flush_write_queue();

  for (int i = 0; i < len; i++) {
    quarter_frame_phase += quarter_frame_phase_step;
    if (quarter_frame_phase >= QUARTER_FRAME_PHASE_PERIOD) {
      quarter_frame_phase -= QUARTER_FRAME_PHASE_PERIOD;

      frame_step_flags = 0;
      if ((quarter_frame_count & 3) == 3) {
        frame_step_flags |= STEP_FLAG_FRAME;
      }
      if ((quarter_frame_count & 1) == 1) {
        frame_step_flags |= STEP_FLAG_HALF;
      }
      frame_step_flags |= STEP_FLAG_QUARTER;
      quarter_frame_count = (quarter_frame_count + 1) & 0x3;
    } else {
      frame_step_flags = 0;
    }

    uint_fast8_t sample = 0;
    sample += sample_pulse(pulse[0]);
    sample += sample_pulse(pulse[1]);
    sample += sample_triangle(triangle);
    sample += sample_noise(noise);
    sample *= 2;
    sample += sample_dmc(dmc);
    buff[i] = sample;
  }

  semaphore_give(SEMAPHORE_APU);

  return result_t::SUCCESS;
}

uint32_t get_state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  flush_write_queue();

  uint8_t buff[STATE_SIZE];
  memset(buff, 0, sizeof(buff));
  uint8_t *p = buff;
  for (int i = 0; i < 2; i++) {
    pulse[i].store(p);
    p += pulse_state_t::STATE_SIZE;
  }
  triangle.store(p);
  p += triangle_state_t::STATE_SIZE;
  noise.store(p);
  p += noise_state_t::STATE_SIZE;
  dmc.store(p);
  p += dmc_state_t::STATE_SIZE;
  BufferWriter writer(p);
  writer.u32(quarter_frame_phase);
  writer.u8(quarter_frame_count);
  writer.u8(frame_step_flags);
  writer.u8(status.raw);
  return fs_write(file_handle, buff, sizeof(buff));
}

result_t load_state(void *file_handle) {
  write_queue.clear();

  uint8_t buff[STATE_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buff, sizeof(buff)));
  uint8_t *p = buff;
  for (int i = 0; i < 2; i++) {
    pulse[i].load(p);
    p += pulse_state_t::STATE_SIZE;
  }
  triangle.load(p);
  p += triangle_state_t::STATE_SIZE;
  noise.load(p);
  p += noise_state_t::STATE_SIZE;
  dmc.load(p);
  p += dmc_state_t::STATE_SIZE;
  BufferReader reader(p);
  quarter_frame_phase = reader.u32();
  quarter_frame_count = reader.u8();
  frame_step_flags = reader.u8();
  status.raw = reader.u8();

  return result_t::SUCCESS;
}

}  // namespace nes::apu
// #include "shapones/cpu.hpp"

// #include "shapones/apu.hpp"

// #include "shapones/common.hpp"

// #include "shapones/host_intf.hpp"

// #include "shapones/input.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"

// #include "shapones/menu.hpp"

// #include "shapones/ppu.hpp"

// #include "shapones/state.hpp"

#ifndef SHAPONES_STATE_HPP
#define SHAPONES_STATE_HPP

// #include "shapones/common.hpp"


namespace nes::state {

static const char *STATE_FILE_EXT = "spn";

static constexpr uint32_t MAX_SLOTS = 16;
static constexpr uint64_t MARKER = 0x74617453736E7053ULL;  // "SpnsStat"
static constexpr uint64_t VERSION = 1;

static constexpr uint32_t SLOT_FLAG_USED = 0x00000001;

static constexpr int SS_WIDTH = 60;
static constexpr int SS_HEIGHT = 56;
static constexpr int SS_SIZE_BYTES = state::SS_WIDTH * state::SS_HEIGHT;

struct state_file_header_t {
  static constexpr uint32_t SIZE = 32;

  uint64_t marker;
  uint64_t version;
  uint32_t slot_size;

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u64(marker);
    writer.u64(version);
    writer.u32(slot_size);
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    marker = reader.u64();
    version = reader.u64();
    slot_size = reader.u32();
  }
};

struct state_slot_entry_t {
  static constexpr uint32_t SIZE = 32;
  static constexpr uint32_t NAME_LENGTH = 16;

  int index;
  uint32_t flags;
  uint32_t frame_count;
  char name[NAME_LENGTH];

  bool is_used() const { return (flags & SLOT_FLAG_USED) != 0; }

  void store(uint8_t *buff) const {
    BufferWriter writer(buff);
    writer.u32(flags);
    writer.u32(frame_count);
    for (int i = 0; i < NAME_LENGTH; i++) {
      writer.u8(name[i]);
    }
  }

  void load(const uint8_t *buff) {
    BufferReader reader(buff);
    flags = reader.u32();
    frame_count = reader.u32();
    for (int i = 0; i < NAME_LENGTH; i++) {
      name[i] = (char)reader.u8();
    }
  }
};

using enum_slot_cb_t = bool (*)(const state_slot_entry_t &entry);

void reset();
void hsync(int focus_y, const uint8_t *line_buff, bool skip_render);

result_t get_state_path(char *out_path, size_t max_len);

uint32_t get_slot_size();

result_t enum_slots(const char *path, enum_slot_cb_t callback);

result_t save(const char *path, int slot);
result_t load(const char *path, int slot);

result_t read_screenshot(const char *path, int slot, uint8_t *out_buff);

}  // namespace nes::state

#endif

namespace nes::cpu {

static constexpr int MAX_BATCH_SIZE = 32;

static registers_t reg;
static bool stopped = false;
#if SHAPONES_IRQ_PENDING_SUPPORT
static uint8_t irq_pending = 0;
#else
static constexpr uint8_t irq_pending = 0;
#endif
volatile cycle_t ppu_cycle_count;

static addr_t dma_addr = 0;
static uint16_t dma_cycle = DMA_TRANSFER_SIZE;

static SHAPONES_INLINE bool dma_is_running() {
  return dma_cycle < DMA_TRANSFER_SIZE;
}

static SHAPONES_INLINE void dma_start(uint8_t src_page) {
  dma_cycle = 0;
  dma_addr = (addr_t)src_page * 0x100;
}

static SHAPONES_INLINE uint_fast8_t dma_service() {
  if (dma_cycle >= DMA_TRANSFER_SIZE) return 0;
  ppu::oam_dma_write(dma_cycle++, cpu::bus_read(dma_addr++));
  return 2;
}

uint8_t bus_read(addr_t addr) {
  uint8_t retval;
  if (memory::PRGROM_BASE <= addr &&
      addr < memory::PRGROM_BASE + PRGROM_RANGE) {
    retval = memory::prgrom_read(addr - memory::PRGROM_BASE);
  } else if (WRAM_BASE <= addr && addr < WRAM_BASE + WRAM_SIZE) {
    retval = memory::wram[addr - WRAM_BASE];
  } else if (memory::PRGRAM_BASE <= addr &&
             addr < memory::PRGRAM_BASE + PRGRAM_RANGE) {
    retval = memory::prgram_read(addr - memory::PRGRAM_BASE);
  } else if (PPUREG_BASE <= addr && addr < PPUREG_BASE + ppu::REG_SIZE) {
    retval = ppu::reg_read(addr);
  } else if (apu::REG_PULSE1_REG0 <= addr && addr <= apu::REG_DMC_REG3 ||
             addr == apu::REG_STATUS) {
    retval = apu::reg_read(addr);
  } else if (INPUT_REG_0 <= addr && addr <= INPUT_REG_1) {
    retval = input::read_latched(addr - INPUT_REG_0);
  } else if (WRAM_MIRROR_BASE <= addr && addr < WRAM_MIRROR_BASE + WRAM_SIZE) {
    retval = memory::wram[addr - WRAM_MIRROR_BASE];
  } else {
    retval = 0;
  }
  return retval;
}

void bus_write(addr_t addr, uint8_t data) {
  if (WRAM_BASE <= addr && addr < WRAM_BASE + WRAM_SIZE) {
    memory::wram[addr - WRAM_BASE] = data;
  } else if (memory::PRGRAM_BASE <= addr &&
             addr < memory::PRGRAM_BASE + PRGRAM_RANGE) {
    memory::prgram_write(addr - memory::PRGRAM_BASE, data);
  } else if (PPUREG_BASE <= addr && addr < PPUREG_BASE + ppu::REG_SIZE) {
    ppu::reg_write(addr, data);
  } else if (addr == OAM_DMA_REG) {
    dma_start(data);
  } else if (apu::REG_PULSE1_REG0 <= addr && addr <= apu::REG_DMC_REG3 ||
             addr == apu::REG_STATUS) {
    apu::reg_write(addr, data);
  } else if (addr == INPUT_REG_0) {
    input::write_control(data);
  } else if (WRAM_MIRROR_BASE <= addr && addr < WRAM_MIRROR_BASE + WRAM_SIZE) {
    memory::wram[addr - WRAM_MIRROR_BASE] = data;
  } else {
    mapper::instance->write(addr, data);
  }
}

static SHAPONES_INLINE uint16_t bus_read_w(addr_t addr) {
  return (uint16_t)bus_read(addr) | ((uint16_t)bus_read(addr + 1) << 8);
}

result_t init() {
  deinit();
  return result_t::SUCCESS;
}

void deinit() {}

result_t reset() {
  reg.A = 0;
  reg.X = 0;
  reg.Y = 0;
  reg.status.negative = 0;
  reg.status.overflow = 0;
  reg.status.reserved = 1;
  reg.status.breakmode = 0;
  reg.status.decimalmode = 0;
  reg.status.interrupt = 1;
  reg.status.zero = 0;
  reg.status.carry = 0;
  reg.SP = 0xfd;
  reg.PC = bus_read_w(VEC_RESET) | 0x8000;
  dma_cycle = DMA_TRANSFER_SIZE;
  dma_addr = 0;
  ppu_cycle_count = 0;
  stopped = false;
  SHAPONES_PRINTF("Entry point: 0x%x\n", (int)reg.PC);
  return result_t::SUCCESS;
}

void stop() {
  stopped = true;
  SHAPONES_PRINTF("CPU stopped.\n");
  SHAPONES_PRINTF("  PC: 0x%02x\n", (int)reg.PC);
  SHAPONES_PRINTF("  A : 0x%02x\n", (int)reg.A);
  SHAPONES_PRINTF("  X : 0x%02x\n", (int)reg.X);
  SHAPONES_PRINTF("  Y : 0x%02x\n", (int)reg.Y);
  SHAPONES_PRINTF("  SP: 0x%02x\n", (int)reg.SP);
  SHAPONES_PRINTF("  STATUS: 0x%02x\n", (int)reg.status.raw);
  SHAPONES_PRINTF("    carry:       %d\n", (int)reg.status.carry);
  SHAPONES_PRINTF("    zero:        %d\n", (int)reg.status.zero);
  SHAPONES_PRINTF("    interrupt:   %d\n", (int)reg.status.interrupt);
  SHAPONES_PRINTF("    decimalmode: %d\n", (int)reg.status.decimalmode);
  SHAPONES_PRINTF("    breakmode:   %d\n", (int)reg.status.breakmode);
  SHAPONES_PRINTF("    reserved:    %d\n", (int)reg.status.reserved);
  SHAPONES_PRINTF("    overflow:    %d\n", (int)reg.status.overflow);
  SHAPONES_PRINTF("    negative:    %d\n", (int)reg.status.negative);
}

bool is_stopped() { return stopped; }

static SHAPONES_INLINE uint8_t fetch() {
  uint8_t retval = bus_read(reg.PC);
  reg.PC += 1;
  if (reg.PC == 0) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0x0000\n");
  }
  return retval;
}

static SHAPONES_INLINE uint16_t fetch_w() {
  uint16_t retval = bus_read_w(reg.PC);
  reg.PC += 2;
  if (reg.PC == 0 || reg.PC == 1) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0x0000\n");
  }
  return retval;
}

static SHAPONES_INLINE uint8_t set_nz(uint8_t value) {
  reg.status.negative = (value >> 7) & 1;
  reg.status.zero = (value == 0) ? 1 : 0;
  return value;
}

static SHAPONES_INLINE void push(uint8_t value) {
  if (reg.SP == 0) {
    SHAPONES_ERRORF("Stack Overflow at push()\n");
  }
  bus_write(0x100 | reg.SP--, value);
}

static SHAPONES_INLINE uint8_t pop() {
  if (reg.SP >= 255) {
    SHAPONES_ERRORF("Stack Underflow at pop()\n");
  }
  return bus_read(0x100 | ++reg.SP);
}

static SHAPONES_INLINE addr_t fetch_zpg() { return fetch(); }
static SHAPONES_INLINE addr_t fetch_zpg_x() { return (fetch() + reg.X) & 0xff; }
static SHAPONES_INLINE addr_t fetch_zpg_y() { return (fetch() + reg.Y) & 0xff; }

static SHAPONES_INLINE addr_t fetch_imm() { return fetch(); }

static SHAPONES_INLINE addr_t fetch_pre_idx_ind(cycle_t *cycle) {
  addr_t base = (fetch() + reg.X) & 0xff;
  addr_t addr = bus_read(base) | ((uint16_t)bus_read((base + 1) & 0xffu) << 8);
  if ((addr & 0xff00u) != (base & 0xff00u)) *cycle += 1;
  return addr;
}

static SHAPONES_INLINE addr_t fetch_post_idx_ind(cycle_t *cycle) {
  addr_t addrOrData = fetch();
  addr_t baseAddr = bus_read(addrOrData) |
                    ((uint16_t)bus_read((addrOrData + 1) & 0xffu) << 8);
  addr_t addr = (baseAddr + reg.Y) & 0xffff;
  if ((addr & 0xff00u) != (baseAddr & 0xff00u)) *cycle += 1;
  return addr;
}

static SHAPONES_INLINE addr_t fetch_abs() { return fetch_w(); }

static SHAPONES_INLINE addr_t fetch_abs_x(cycle_t *cycle) {
  uint16_t base = fetch_w();
  uint16_t retval = base + reg.X;
  if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
  return retval;
}

static SHAPONES_INLINE addr_t fetch_abs_y(cycle_t *cycle) {
  uint16_t base = fetch_w();
  uint16_t retval = base + reg.Y;
  if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
  return retval;
}

static SHAPONES_INLINE addr_t fetch_ind_abs() {
  addr_t addr_or_data = fetch_w();
  addr_t next_addr =
      (addr_or_data & 0xFF00) | (((addr_or_data & 0xFF) + 1) & 0xFF);
  addr_t addr = bus_read(addr_or_data) | ((uint16_t)bus_read(next_addr) << 8);
  return addr;
}

static SHAPONES_INLINE addr_t fetch_rel(cycle_t *cycle) {
  int distance = fetch();
  if (distance >= 0x80) distance -= 256;
  addr_t retval = reg.PC + distance;
  if ((retval & 0xff00u) != (reg.PC & 0xff00u)) *cycle += 1;
  return retval;
}

static SHAPONES_INLINE void opBRK() {
  fetch();  // padding
  push(reg.PC >> 8);
  push(reg.PC & 0xff);
  auto s = reg.status;
  s.breakmode = true;
  push(s.raw);
  reg.status.interrupt = true;
  reg.PC = bus_read_w(VEC_IRQ);
}

static SHAPONES_INLINE void opJMP(addr_t addr) { reg.PC = addr; }

static SHAPONES_INLINE void opJSR(addr_t addr) {
  addr_t pc = reg.PC - 1;
  if (pc == 0xFFFF) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0xFFFF after JSR\n");
  }
  push(pc >> 8);
  push(pc & 0xff);
  reg.PC = addr;
}

static SHAPONES_INLINE void opRTI() {
  reg.status.raw = pop();
  reg.status.reserved = 1;
  reg.status.breakmode = 0;
  reg.PC = (addr_t)pop();
  reg.PC |= ((addr_t)pop() << 8);
}

static SHAPONES_INLINE void opRTS() {
  reg.PC = (addr_t)pop();
  reg.PC |= ((addr_t)pop() << 8);
  reg.PC += 1;
  if (reg.PC == 0) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0x0000 after RTS\n");
  }
}

static SHAPONES_INLINE void opBIT(addr_t addr) {
  uint8_t data = bus_read(addr);
  reg.status.negative = (data >> 7) & 1;
  reg.status.overflow = (data >> 6) & 1;
  reg.status.zero = (reg.A & data) ? 0 : 1;
}

static SHAPONES_INLINE void opPHP() {
  auto s = reg.status;
  s.breakmode = 1;
  push(s.raw);
}

static SHAPONES_INLINE void opPLP() {
  reg.status.raw = pop();
  reg.status.reserved = 1;
  reg.status.breakmode = 0;
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = 2;
#endif
}

static SHAPONES_INLINE void opPHA() { push(reg.A); }

static SHAPONES_INLINE void opPLA() { reg.A = set_nz(pop()); }

static SHAPONES_INLINE void cond_jump(bool cond, addr_t addr, cycle_t *cycle) {
  if (cond) {
    reg.PC = addr;
    (*cycle)++;
  }
}

static SHAPONES_INLINE void opBPL(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.negative, addr, cycle);
}
static SHAPONES_INLINE void opBMI(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.negative, addr, cycle);
}
static SHAPONES_INLINE void opBVC(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.overflow, addr, cycle);
}
static SHAPONES_INLINE void opBVS(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.overflow, addr, cycle);
}
static SHAPONES_INLINE void opBCC(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.carry, addr, cycle);
}
static SHAPONES_INLINE void opBCS(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.carry, addr, cycle);
}
static SHAPONES_INLINE void opBNE(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.zero, addr, cycle);
}
static SHAPONES_INLINE void opBEQ(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.zero, addr, cycle);
}

static SHAPONES_INLINE void opCLC() { reg.status.carry = 0; }
static SHAPONES_INLINE void opSEC() { reg.status.carry = 1; }
static SHAPONES_INLINE void opCLI() {
  reg.status.interrupt = 0;
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = 2;
#endif
}
static SHAPONES_INLINE void opSEI() {
  reg.status.interrupt = 1;
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = 2;
#endif
}
static SHAPONES_INLINE void opCLV() { reg.status.overflow = 0; }
static SHAPONES_INLINE void opCLD() { reg.status.decimalmode = 0; }
static SHAPONES_INLINE void opSED() { reg.status.decimalmode = 1; }

static SHAPONES_INLINE void opTXA() { reg.A = set_nz(reg.X); }
static SHAPONES_INLINE void opTYA() { reg.A = set_nz(reg.Y); }
static SHAPONES_INLINE void opTXS() { reg.SP = reg.X; }
static SHAPONES_INLINE void opTAY() { reg.Y = set_nz(reg.A); }
static SHAPONES_INLINE void opTAX() { reg.X = set_nz(reg.A); }
static SHAPONES_INLINE void opTSX() { reg.X = set_nz(reg.SP); }

static SHAPONES_INLINE void opLDA(uint8_t data) { reg.A = set_nz(data); }
static SHAPONES_INLINE void opLDX(uint8_t data) { reg.X = set_nz(data); }
static SHAPONES_INLINE void opLDY(uint8_t data) { reg.Y = set_nz(data); }

static SHAPONES_INLINE void opSTA(addr_t addr) { bus_write(addr, reg.A); }
static SHAPONES_INLINE void opSTX(addr_t addr) { bus_write(addr, reg.X); }
static SHAPONES_INLINE void opSTY(addr_t addr) { bus_write(addr, reg.Y); }

static SHAPONES_INLINE void compare(uint8_t a, uint8_t b) {
  int16_t compared = (int16_t)a - (int16_t)b;
  reg.status.carry = compared >= 0;
  set_nz(compared);
}
static SHAPONES_INLINE void opCMP(uint8_t data) { compare(reg.A, data); }
static SHAPONES_INLINE void opCPX(uint8_t data) { compare(reg.X, data); }
static SHAPONES_INLINE void opCPY(uint8_t data) { compare(reg.Y, data); }

static SHAPONES_INLINE void opINX() { set_nz(++reg.X); }
static SHAPONES_INLINE void opINY() { set_nz(++reg.Y); }
static SHAPONES_INLINE void opDEX() { set_nz(--reg.X); }
static SHAPONES_INLINE void opDEY() { set_nz(--reg.Y); }

static SHAPONES_INLINE void opINC(addr_t addr) {
  bus_write(addr, set_nz(bus_read(addr) + 1));
}
static SHAPONES_INLINE void opDEC(addr_t addr) {
  bus_write(addr, set_nz(bus_read(addr) - 1));
}

static SHAPONES_INLINE void opAND(uint8_t data) {
  reg.A = set_nz(data & reg.A);
}
static SHAPONES_INLINE void opORA(uint8_t data) {
  reg.A = set_nz(data | reg.A);
}
static SHAPONES_INLINE void opEOR(uint8_t data) {
  reg.A = set_nz(data ^ reg.A);
}

static SHAPONES_INLINE void opADC(uint8_t data) {
  uint_fast16_t operated = (uint_fast16_t)reg.A + data + reg.status.carry;
  reg.status.overflow =
      (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
  reg.status.carry = (operated >= 0x100) ? 1 : 0;
  reg.A = set_nz(operated);
}

static SHAPONES_INLINE void opSBC(uint8_t data) {
  int_fast16_t operated =
      (int_fast16_t)reg.A - data - (reg.status.carry ? 0 : 1);
  reg.status.overflow =
      (((reg.A ^ operated) & 0x80) != 0 && ((reg.A ^ data) & 0x80) != 0);
  reg.status.carry = (operated >= 0) ? 1 : 0;
  reg.A = set_nz(operated);
}

static SHAPONES_INLINE uint8_t opASL(uint8_t data) {
  reg.status.carry = (data >> 7) & 1;
  return set_nz(data << 1);
}
static SHAPONES_INLINE void opASL_a() { reg.A = opASL(reg.A); }
static SHAPONES_INLINE void opASL_m(addr_t addr) {
  bus_write(addr, opASL(bus_read(addr)));
}

static SHAPONES_INLINE uint8_t opLSR(uint8_t data) {
  reg.status.carry = data & 1;
  return set_nz((data >> 1) & 0x7f);
}
static SHAPONES_INLINE void opLSR_a() { reg.A = opLSR(reg.A); }
static SHAPONES_INLINE void opLSR_m(addr_t addr) {
  bus_write(addr, opLSR(bus_read(addr)));
}

static SHAPONES_INLINE uint8_t opROL(uint8_t data) {
  uint_fast8_t carry = (data >> 7) & 1;
  data = (data << 1) | reg.status.carry;
  reg.status.carry = carry;
  return set_nz(data);
}
static SHAPONES_INLINE void opROL_a() { reg.A = opROL(reg.A); }
static SHAPONES_INLINE void opROL_m(addr_t addr) {
  bus_write(addr, opROL(bus_read(addr)));
}

static SHAPONES_INLINE uint8_t opROR(uint8_t data) {
  uint_fast8_t carry = data & 1;
  data = ((data >> 1) & 0x7f) | (reg.status.carry << 7);
  reg.status.carry = carry;
  return set_nz(data);
}
static SHAPONES_INLINE void opROR_a() { reg.A = opROR(reg.A); }
static SHAPONES_INLINE void opROR_m(addr_t addr) {
  bus_write(addr, opROR(bus_read(addr)));
}

static SHAPONES_INLINE void opNOP() {}

static SHAPONES_INLINE void opSLO(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  reg.status.carry = (data & 0x80) >> 7;
  data <<= 1;
  reg.A = set_nz(reg.A | data);
  bus_write(addr, data);
}
static SHAPONES_INLINE void opRLA(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  uint_fast8_t carry = (data & 0x80) >> 7;
  data = (data << 1) | reg.status.carry;
  reg.status.carry = carry;
  reg.A = set_nz(reg.A & data);
  bus_write(addr, data);
}
static SHAPONES_INLINE void opSRE(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  reg.status.carry = data & 0x1;
  data >>= 1;
  reg.A = set_nz(reg.A ^ data);
  bus_write(addr, data);
}
static SHAPONES_INLINE void opRRA(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  uint_fast8_t carry = data & 0x1;
  data = ((data >> 1) & 0x7f);
  data |= reg.status.carry << 7;
  uint_fast16_t operated = (uint_fast16_t)data + reg.A + carry;
  reg.status.overflow =
      (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
  reg.A = set_nz(operated);
  reg.status.carry = (operated >> 8) & 1;
  bus_write(addr, data);
}

static SHAPONES_INLINE void opSAX(addr_t addr) {
  bus_write(addr, reg.A & reg.X);
}
static SHAPONES_INLINE void opLAX(uint8_t data) {
  reg.A = reg.X = set_nz(data);
}

static SHAPONES_INLINE void opDCP(addr_t addr) {
  uint_fast8_t operated = bus_read(addr) - 1;
  set_nz(reg.A - operated);
  bus_write(addr, operated);
}

static SHAPONES_INLINE void opISB(addr_t addr) {
  uint_fast8_t data = bus_read(addr) + 1;
  uint_fast16_t operated =
      (uint_fast16_t)(~data & 0xff) + reg.A + reg.status.carry;
  reg.status.overflow =
      (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
  reg.status.carry = (operated >> 8) & 1;
  reg.A = set_nz(operated);
  bus_write(addr, data);
}

result_t service() {
  menu::service();
  input::update();

  int n = MAX_BATCH_SIZE;
  while (n-- > 0) {
    cycle_t cycle = 0;

    cycle_t ppu_cycle_diff = ppu_cycle_count - ppu::cycle_following();
    if (ppu_cycle_diff >= ppu::MAX_DELAY_CYCLES) {
      break;
    }

    if (stopped) {
      cycle += 1;  // nop
    } else if (dma_is_running()) {
      // DMA is running
      cycle += dma_service();
    } else if (irq_pending == 0 && interrupt::is_nmi_asserted()) {
      // NMI
      interrupt::deassert_nmi();
      auto s = reg.status;
      s.breakmode = 0;
      push(reg.PC >> 8);
      push(reg.PC & 0xff);
      push(s.raw);
      reg.status.interrupt = true;
      reg.PC = bus_read_w(VEC_NMI);
      cycle += 7;  // ?
    } else if (irq_pending == 0 && !!interrupt::get_irq() &&
               !reg.status.interrupt) {
      // IRQ
      auto s = reg.status;
      s.breakmode = 0;
      push(reg.PC >> 8);
      push(reg.PC & 0xff);
      push(s.raw);
      reg.status.interrupt = true;
      reg.PC = bus_read_w(VEC_IRQ);
      cycle += 7;  // ?
    } else {
      uint8_t op_code = fetch();

      // clang-format off
      switch(op_code) {

      case 0x00: opBRK();                                     cycle += 7; break;
      case 0x20: opJSR(fetch_abs());                          cycle += 6; break;
      case 0x40: opRTI();                                     cycle += 6; break;
      case 0x60: opRTS();                                     cycle += 6; break;
      case 0x4c: opJMP(fetch_abs());                          cycle += 3; break;
      case 0x6c: opJMP(fetch_ind_abs());                      cycle += 5; break;

      case 0x24: opBIT(fetch_zpg());                          cycle += 3; break;
      case 0x2c: opBIT(fetch_abs());                          cycle += 4; break;

      case 0x08: opPHP();                                     cycle += 3; break;
      case 0x28: opPLP();                                     cycle += 4; break;
      case 0x48: opPHA();                                     cycle += 3; break;
      case 0x68: opPLA();                                     cycle += 4; break;

      case 0x10: opBPL(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x30: opBMI(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x50: opBVC(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x70: opBVS(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x90: opBCC(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0xb0: opBCS(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0xd0: opBNE(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0xf0: opBEQ(fetch_rel(&cycle), &cycle);            cycle += 2; break;

      case 0x18: opCLC();                                     cycle += 2; break;
      case 0x38: opSEC();                                     cycle += 2; break;
      case 0x58: opCLI();                                     cycle += 2; break;
      case 0x78: opSEI();                                     cycle += 2; break;
      case 0xb8: opCLV();                                     cycle += 2; break;
      case 0xd8: opCLD();                                     cycle += 2; break;
      case 0xf8: opSED();                                     cycle += 2; break;

      case 0x8a: opTXA();                                     cycle += 2; break;
      case 0x98: opTYA();                                     cycle += 2; break;
      case 0x9a: opTXS();                                     cycle += 2; break;
      case 0xa8: opTAY();                                     cycle += 2; break;
      case 0xaa: opTAX();                                     cycle += 2; break;
      case 0xba: opTSX();                                     cycle += 2; break;

      case 0x81: opSTA(fetch_pre_idx_ind(&cycle));            cycle += 6; break;
      case 0x85: opSTA(fetch_zpg());                          cycle += 3; break;
      case 0x8d: opSTA(fetch_abs());                          cycle += 4; break;
      case 0x91: opSTA(fetch_post_idx_ind(&cycle));           cycle += 6; break;
      case 0x95: opSTA(fetch_zpg_x());                        cycle += 4; break;
      case 0x99: opSTA(fetch_abs_y(&cycle));                  cycle += 4; break;
      case 0x9d: opSTA(fetch_abs_x(&cycle));                  cycle += 4; break;

      case 0x86: opSTX(fetch_zpg());                          cycle += 3; break;
      case 0x8e: opSTX(fetch_abs());                          cycle += 4; break;
      case 0x96: opSTX(fetch_zpg_y());                        cycle += 4; break;

      case 0x84: opSTY(fetch_zpg());                          cycle += 3; break;
      case 0x8c: opSTY(fetch_abs());                          cycle += 4; break;
      case 0x94: opSTY(fetch_zpg_x());                        cycle += 4; break;
  
      case 0xa1: opLDA(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xa5: opLDA(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xa9: opLDA(fetch_imm());                          cycle += 2; break;
      case 0xad: opLDA(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb1: opLDA(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xb5: opLDA(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xb9: opLDA(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0xbd: opLDA(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xa2: opLDX(fetch_imm());                          cycle += 2; break;
      case 0xa6: opLDX(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xae: opLDX(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb6: opLDX(bus_read(fetch_zpg_y()));              cycle += 4; break;
      case 0xbe: opLDX(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;

      case 0xa0: opLDY(fetch_imm());                          cycle += 2; break;
      case 0xa4: opLDY(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xac: opLDY(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb4: opLDY(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xbc: opLDY(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xc1: opCMP(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xc5: opCMP(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xc9: opCMP(fetch_imm());                          cycle += 2; break;
      case 0xcd: opCMP(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xd1: opCMP(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xd5: opCMP(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xd9: opCMP(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0xdd: opCMP(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xe0: opCPX(fetch_imm());                          cycle += 2; break;
      case 0xe4: opCPX(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xec: opCPX(bus_read(fetch_abs()));                cycle += 4; break;

      case 0xc0: opCPY(fetch_imm());                          cycle += 2; break;
      case 0xc4: opCPY(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xcc: opCPY(bus_read(fetch_abs()));                cycle += 4; break;

      case 0xca: opDEX();                                     cycle += 2; break;
      case 0x88: opDEY();                                     cycle += 2; break;
      
      case 0xe8: opINX();                                     cycle += 2; break;
      case 0xc8: opINY();                                     cycle += 2; break;

      case 0xc6: opDEC(fetch_zpg());                          cycle += 5; break;
      case 0xce: opDEC(fetch_abs());                          cycle += 6; break;
      case 0xd6: opDEC(fetch_zpg_x());                        cycle += 6; break;
      case 0xde: opDEC(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0xe6: opINC(fetch_zpg());                          cycle += 5; break;
      case 0xee: opINC(fetch_abs());                          cycle += 6; break;
      case 0xf6: opINC(fetch_zpg_x());                        cycle += 6; break;
      case 0xfe: opINC(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x01: opORA(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x05: opORA(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x09: opORA(fetch_imm());                          cycle += 2; break;
      case 0x0d: opORA(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x11: opORA(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x15: opORA(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x19: opORA(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x1d: opORA(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x21: opAND(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x25: opAND(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x29: opAND(fetch_imm());                          cycle += 2; break;
      case 0x2d: opAND(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x31: opAND(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x35: opAND(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x39: opAND(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x3d: opAND(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x41: opEOR(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x45: opEOR(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x49: opEOR(fetch_imm());                          cycle += 2; break;
      case 0x4d: opEOR(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x51: opEOR(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x55: opEOR(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x59: opEOR(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x5d: opEOR(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x61: opADC(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x65: opADC(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x69: opADC(fetch_imm());                          cycle += 2; break;
      case 0x6d: opADC(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x71: opADC(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x75: opADC(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x79: opADC(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x7d: opADC(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xe1: opSBC(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xe5: opSBC(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xe9: opSBC(fetch_imm());                          cycle += 2; break;
      case 0xed: opSBC(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xf1: opSBC(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xf5: opSBC(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xf9: opSBC(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0xfd: opSBC(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x06: opASL_m(fetch_zpg());                        cycle += 5; break;
      case 0x0a: opASL_a();                                   cycle += 2; break;
      case 0x0e: opASL_m(fetch_abs());                        cycle += 6; break;
      case 0x16: opASL_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x1e: opASL_m(fetch_abs_x(&cycle));                cycle += 6; break;

      case 0x26: opROL_m(fetch_zpg());                        cycle += 5; break;
      case 0x2a: opROL_a();                                   cycle += 2; break;
      case 0x2e: opROL_m(fetch_abs());                        cycle += 6; break;
      case 0x36: opROL_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x3e: opROL_m(fetch_abs_x(&cycle));                cycle += 6; break;

      case 0x46: opLSR_m(fetch_zpg());                        cycle += 5; break;
      case 0x4a: opLSR_a();                                   cycle += 2; break;
      case 0x4e: opLSR_m(fetch_abs());                        cycle += 6; break;
      case 0x56: opLSR_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x5e: opLSR_m(fetch_abs_x(&cycle));                cycle += 6; break;

      case 0x66: opROR_m(fetch_zpg());                        cycle += 5; break;
      case 0x6a: opROR_a();                                   cycle += 2; break;
      case 0x6e: opROR_m(fetch_abs());                        cycle += 6; break;
      case 0x76: opROR_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x7e: opROR_m(fetch_abs_x(&cycle));                cycle += 6; break;
      
      case 0xea: opNOP();                                     cycle += 2; break;
#if 0
      // unofficial opcodes
      case 0x03: opSLO(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x07: opSLO(fetch_zpg());                          cycle += 5; break;
      case 0x0f: opSLO(fetch_abs());                          cycle += 6; break;
      case 0x13: opSLO(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x17: opSLO(fetch_zpg_x());                        cycle += 6; break;
      case 0x1b: opSLO(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x1f: opSLO(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x23: opRLA(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x27: opRLA(fetch_zpg());                          cycle += 5; break;
      case 0x2f: opRLA(fetch_abs());                          cycle += 6; break;
      case 0x33: opRLA(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x37: opRLA(fetch_zpg_x());                        cycle += 6; break;
      case 0x3b: opRLA(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x3f: opRLA(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x43: opSRE(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x47: opSRE(fetch_zpg());                          cycle += 5; break;
      case 0x4f: opSRE(fetch_abs());                          cycle += 6; break;
      case 0x53: opSRE(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x57: opSRE(fetch_zpg_x());                        cycle += 6; break;
      case 0x5b: opSRE(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x5f: opSRE(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x63: opRRA(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x67: opRRA(fetch_zpg());                          cycle += 5; break;
      case 0x6f: opRRA(fetch_abs());                          cycle += 6; break;
      case 0x73: opRRA(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x77: opRRA(fetch_zpg_x());                        cycle += 6; break;
      case 0x7b: opRRA(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x7f: opRRA(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x83: opSAX(fetch_pre_idx_ind(&cycle));            cycle += 6; break;
      case 0x87: opSAX(fetch_zpg());                          cycle += 3; break;
      case 0x8f: opSAX(fetch_abs());                          cycle += 4; break;
      case 0x97: opSAX(fetch_zpg_y());                        cycle += 4; break;

      case 0xa3: opLAX(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xa7: opLAX(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xab: opLAX(fetch_imm());                          cycle += 2; break;
      case 0xaf: opLAX(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb3: opLAX(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xb7: opLAX(bus_read(fetch_zpg_y()));              cycle += 4; break;
      case 0xbf: opLAX(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;

      case 0xc3: opDCP(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0xc7: opDCP(fetch_zpg());                          cycle += 5; break;
      case 0xcf: opDCP(fetch_abs());                          cycle += 6; break;
      case 0xd3: opDCP(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0xd7: opDCP(fetch_zpg_x());                        cycle += 6; break;
      case 0xdb: opDCP(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0xdf: opDCP(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0xe3: opISB(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0xe7: opISB(fetch_zpg());                          cycle += 5; break;
      case 0xef: opISB(fetch_abs());                          cycle += 6; break;
      case 0xf3: opISB(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0xf7: opISB(fetch_zpg_x());                        cycle += 6; break;
      case 0xfb: opISB(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0xff: opISB(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0xeb: opSBC(fetch_imm());                          cycle += 2; break;

      case 0x1a: opNOP();                                     cycle += 2; break;
      case 0x3a: opNOP();                                     cycle += 2; break;
      case 0x5a: opNOP();                                     cycle += 2; break;
      case 0x7a: opNOP();                                     cycle += 2; break;
      case 0xda: opNOP();                                     cycle += 2; break;
      case 0xfa: opNOP();                                     cycle += 2; break;

      case 0x02: opNOP();                                     cycle += 2; break; // STP
      case 0x12: opNOP();                                     cycle += 2; break; // STP
      case 0x22: opNOP();                                     cycle += 2; break; // STP
      case 0x32: opNOP();                                     cycle += 2; break; // STP
      case 0x42: opNOP();                                     cycle += 2; break; // STP
      case 0x52: opNOP();                                     cycle += 2; break; // STP
      case 0x62: opNOP();                                     cycle += 2; break; // STP
      case 0x72: opNOP();                                     cycle += 2; break; // STP
      case 0x92: opNOP();                                     cycle += 2; break; // STP
      case 0xb2: opNOP();                                     cycle += 2; break; // STP
      case 0xd2: opNOP();                                     cycle += 2; break; // STP
      case 0xf2: opNOP();                                     cycle += 2; break; // STP

      case 0x9c: fetch_abs_x(&cycle); opNOP();                cycle += 5; break; // SHY
      case 0x9e: fetch_abs_y(&cycle); opNOP();                cycle += 5; break; // SHX
      case 0x0b: fetch_imm(); opNOP();                        cycle += 2; break; // ANC
      case 0x2b: fetch_imm(); opNOP();                        cycle += 2; break; // ANC
      case 0x4b: fetch_imm(); opNOP();                        cycle += 2; break; // ALR
      case 0x6b: fetch_imm(); opNOP();                        cycle += 2; break; // ARR
      case 0x8b: fetch_imm(); opNOP();                        cycle += 2; break; // XAA
      case 0xcb: fetch_imm(); opNOP();                        cycle += 2; break; // AXS

      case 0x93: fetch_post_idx_ind(&cycle); opNOP();         cycle += 6; break; // AHX
      case 0x9f: fetch_abs_y(&cycle); opNOP();                cycle += 5; break; // AHX

      case 0x9b: fetch_abs_y(&cycle); opNOP();                cycle += 5; break; // TAS
      case 0xbb: fetch_abs_y(&cycle); opNOP();                cycle += 4; break; // LAS

      case 0x80: fetch_imm(); opNOP();                        cycle += 2; break; 
      case 0x82: fetch_imm(); opNOP();                        cycle += 2; break; 
      case 0x89: fetch_imm(); opNOP();                        cycle += 2; break; 
      case 0xc2: fetch_imm(); opNOP();                        cycle += 2; break;
      case 0xe2: fetch_imm(); opNOP();                        cycle += 3; break;

      case 0x04: fetch_zpg(); opNOP();                        cycle += 3; break; 
      case 0x44: fetch_zpg(); opNOP();                        cycle += 3; break; 
      case 0x64: fetch_zpg(); opNOP();                        cycle += 3; break;

      case 0x0c: fetch_abs(); opNOP();                        cycle += 4; break;

      case 0x14: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0x34: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0x54: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0x74: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0xd4: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0xf4: fetch_zpg_x(); opNOP();                      cycle += 4; break;

      case 0x1c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0x3c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0x5c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0x7c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0xdc: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0xfc: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
#else
      default:
          SHAPONES_ERRORF("UNKNOWN INSTRUCTION: 0x%02x (PC=0x%04x)\n", (int)op_code, (int)reg.PC);
          break;
#endif
      }
      // clang-format on
    }  // if

#if SHAPONES_IRQ_PENDING_SUPPORT
    if (irq_pending > 0) {
      irq_pending--;
    }
#endif
    ppu_cycle_count += cycle * 3;

  }  // while

  return result_t::SUCCESS;
}

uint32_t get_state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  memset(buffer, 0, sizeof(buffer));
  uint8_t *p = buffer;
  reg.store(p);
  p += registers_t::STATE_SIZE;
  BufferWriter writer(p);
  writer.u64(ppu_cycle_count);
  writer.u16(dma_addr);
  writer.u16(dma_cycle);
  writer.b(stopped);
  writer.u8(irq_pending);
  return fs_write(file_handle, buffer, sizeof(buffer));
}

result_t load_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buffer, sizeof(buffer)));
  const uint8_t *p = buffer;
  reg.load(p);
  p += registers_t::STATE_SIZE;
  BufferReader reader(p);
  ppu_cycle_count = reader.u64();
  dma_addr = reader.u16();
  dma_cycle = reader.u16();
  stopped = reader.b();
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = reader.u8();
#else
  reader.u8();  // discard
#endif
  return result_t::SUCCESS;
}

}  // namespace nes::cpu
// #include "shapones/fs.hpp"

#ifndef SHAPONES_FS_HPP
#define SHAPONES_FS_HPP

// #include "shapones/common.hpp"


namespace nes::fs {


 bool is_root_dir(const char *path);
 int find_parent_separator(const char *path);
 int find_char_rev(const char *path, char c, int start_idx = -1);
 result_t append_separator(char *path);
 result_t append_path(char *path, const char *name);
 result_t replace_ext(char *path, const char *new_ext);

}  // namespace nes::fs

#endif

namespace nes::fs {

bool is_root_dir(const char *path) { return find_parent_separator(path) < 0; }

int find_parent_separator(const char *path) {
  int n = strnlen(path, nes::MAX_PATH_LENGTH);
  if (n == 0) {
    return -1;
  }
  if (path[n - 1] == '/') {
    n--;
  }
  return find_char_rev(path, '/', n);
}

int find_char_rev(const char *path, char c, int start_idx) {
  if (start_idx < 0) {
    start_idx = strnlen(path, nes::MAX_PATH_LENGTH);
  } else if (start_idx == 0) {
    return -1;
  }
  for (int i = start_idx - 1; i >= 0; i--) {
    if (path[i] == c) {
      return i;
    }
  }
  return -1;
}

result_t append_separator(char *path) {
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  if (path[len - 1] == '/') {
    return result_t::SUCCESS;
  }
  if (len + 1 >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_FS_PATH_TOO_LONG;
  }
  path[len++] = '/';
  path[len] = '\0';
  return result_t::SUCCESS;
}

result_t append_path(char *path, const char *name) {
  SHAPONES_TRY(append_separator(path));
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  int name_len = strnlen(name, nes::MAX_PATH_LENGTH);
  if (len + name_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_FS_PATH_TOO_LONG;
  }

  strcat(path, name);
  return result_t::SUCCESS;
}

result_t replace_ext(char *path, const char *new_ext) {
  int old_path_len = strnlen(path, nes::MAX_PATH_LENGTH);

  int sep_idx = find_char_rev(path, '/');
  int old_name_len = old_path_len;
  if (sep_idx >= 0) {
    old_name_len -= (sep_idx + 1);
  }

  int dot_idx = find_char_rev(path, '.');
  if (dot_idx < 0 || dot_idx < sep_idx) {
    dot_idx = old_path_len;
  }

  int old_ext_len = old_path_len - dot_idx - 1;
  int new_ext_len = strnlen(new_ext, nes::MAX_FILENAME_LENGTH);

  int new_path_len = old_path_len - old_ext_len + new_ext_len;
  if (new_path_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_FS_PATH_TOO_LONG;
  }

  int new_name_len = old_name_len - old_ext_len + new_ext_len;
  if (new_name_len >= nes::MAX_FILENAME_LENGTH) {
    return result_t::ERR_FS_PATH_TOO_LONG;
  }

  path[dot_idx] = '.';
  strcpy(&path[dot_idx + 1], new_ext);
  return result_t::SUCCESS;
}

}  // namespace nes::fs// #include "shapones/input.hpp"

// #include "shapones/host_intf.hpp"

// #include "shapones/menu.hpp"


namespace nes::input {

static constexpr uint32_t STATE_SIZE = 16;

static control_t reg;
static status_t raw[2];
static uint8_t shift_reg[2];

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  reg.raw = 0;
  raw[0].raw = 0;
  raw[1].raw = 0;
  shift_reg[0] = 0;
  shift_reg[1] = 0;
  return result_t::SUCCESS;
}

status_t get_status(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  return raw[player];
}

void set_status(int player, status_t s) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  raw[player] = s;
}

void update() {
  if (reg.strobe) {
    if (menu::is_shown()) {
      shift_reg[0] = 0;
      shift_reg[1] = 0;
    } else {
      shift_reg[0] = raw[0].raw;
      shift_reg[1] = raw[1].raw;
    }
  }
}

uint8_t read_latched(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  uint8_t retval = shift_reg[player] & 1;
  shift_reg[player] >>= 1;
  return retval;
}

void write_control(uint8_t data) {
  reg.raw = data;
  update();
}

uint32_t state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  memset(buffer, 0, STATE_SIZE);
  uint8_t *p = buffer;
  BufferWriter writer(p);
  writer.u8(reg.raw);
  writer.u8(raw[0].raw);
  writer.u8(raw[1].raw);
  writer.u8(shift_reg[0]);
  writer.u8(shift_reg[1]);
  return fs_write(file_handle, buffer, STATE_SIZE);
}

result_t load_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buffer, STATE_SIZE));
  const uint8_t *p = buffer;
  BufferReader reader(p);
  reg.raw = reader.u8();
  raw[0].raw = reader.u8();
  raw[1].raw = reader.u8();
  shift_reg[0] = reader.u8();
  shift_reg[1] = reader.u8();
  return result_t::SUCCESS;
}

}  // namespace nes::input
// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"


namespace nes::interrupt {

static constexpr uint32_t STATE_SIZE = 16;

static volatile source_t irq;
static volatile bool nmi;

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  SpinLockBlock lock(SPINLOCK_IRQ);
  irq = static_cast<source_t>(0);
  nmi = false;
  return result_t::SUCCESS;
}

void assert_irq(source_t src) {
  SpinLockBlock lock(SPINLOCK_IRQ);
  irq = irq | src;
}
void deassert_irq(source_t src) {
  SpinLockBlock lock(SPINLOCK_IRQ);
  irq = irq & ~src;
}
source_t get_irq() { return irq; }

void assert_nmi() { nmi = true; }
void deassert_nmi() { nmi = false; }
bool is_nmi_asserted() { return nmi; }

uint32_t get_state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  memset(buffer, 0, STATE_SIZE);
  uint8_t *p = buffer;
  BufferWriter writer(p);
  writer.u32(static_cast<uint32_t>(irq));
  writer.b(nmi);
  return fs_write(file_handle, buffer, STATE_SIZE);
}

result_t load_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buffer, STATE_SIZE));
  const uint8_t *p = buffer;
  BufferReader reader(p);
  irq = static_cast<source_t>(reader.u32());
  nmi = reader.b();
  return result_t::SUCCESS;
}

}  // namespace nes::interrupt
// #include "shapones/mapper.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

// #include "shapones/memory.hpp"


// #include "shapones/mappers/map000.hpp"

#ifndef SHAPONES_MAP000_HPP
#define SHAPONES_MAP000_HPP

// #include "shapones/mapper.hpp"


namespace nes::mapper {

class Map000 : public Mapper {
 public:
  Map000() : Mapper(0, "NROM") {}
};

}  // namespace nes::mapper

#endif
// #include "shapones/mappers/map001.hpp"

#ifndef SHAPONES_MAP001_HPP
#define SHAPONES_MAP001_HPP

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"


namespace nes::mapper {

using namespace nes::memory;

class Map001 : public Mapper {
 private:
  uint8_t shift_reg = 0b10000;
  uint8_t ctrl_reg = 0;
  uint8_t chr_bank0 = 0;
  uint8_t chr_bank1 = 0;
  uint8_t prg_bank = 0;

 public:
  static constexpr uint32_t STATE_SIZE = 8;

  Map001() : Mapper(1, "MMC1") {}

  result_t reset() override {
    shift_reg = 0b10000;
    ctrl_reg = 0x0C;
    chr_bank0 = 0;
    chr_bank1 = 0;
    prg_bank = 0x00;
    perform_remap();
    return result_t::SUCCESS;
  }

  void write(addr_t addr, uint8_t value) override {
    bool remap = false;
    if (value & 0x80) {
      shift_reg = 0b10000;
      ctrl_reg |= 0x0C;
      remap = true;
    } else {
      bool shift = !(shift_reg & 0x01);
      uint8_t val = (shift_reg >> 1) | ((value & 0x01) << 4);
      if (shift) {
        shift_reg = val;
      } else {
        shift_reg = 0b10000;
        switch (addr & 0x6000) {
          default:
          case 0x0000: ctrl_reg = val; break;
          case 0x2000: chr_bank0 = val; break;
          case 0x4000: chr_bank1 = val; break;
          case 0x6000: prg_bank = val; break;
        }
        remap = true;
      }
    }

    if (remap) {
      perform_remap();
    }
  }

  void perform_remap() {
    switch (ctrl_reg & 0x03) {
      default:
      case 0:
        set_nametable_arrangement(nametable_arrangement_t::SINGLE_LOWER);
        break;
      case 1:
        set_nametable_arrangement(nametable_arrangement_t::SINGLE_UPPER);
        break;
      case 2:
        set_nametable_arrangement(nametable_arrangement_t::HORIZONTAL);
        break;
      case 3:
        set_nametable_arrangement(nametable_arrangement_t::VERTICAL);
        break;
    }

    switch (ctrl_reg & 0x0C) {
      default:
      case 0x00:
      case 0x04:
        prgrom_remap(0x8000, (prg_bank & 0xFE) * 0x4000, 0x8000);
        break;
      case 0x08:
        prgrom_remap(0x8000, 0 * 0x4000, 0x4000);
        prgrom_remap(0xC000, (prg_bank & 0x0F) * 0x4000, 0x4000);
        break;
      case 0x0C:
        prgrom_remap(0x8000, (prg_bank & 0x0F) * 0x4000, 0x4000);
        prgrom_remap(0xC000, prgrom_phys_size - 0x4000, 0x4000);
        break;
    }

    if ((ctrl_reg & 0x10) == 0) {
      chrrom_remap(0x0000, (chr_bank0 & 0x1E) << 12, 0x2000);
    } else {
      chrrom_remap(0x0000, (chr_bank0 & 0x1F) << 12, 0x1000);
      chrrom_remap(0x1000, (chr_bank1 & 0x1F) << 12, 0x1000);
    }
  }

  uint32_t get_state_size() override { return STATE_SIZE; }

  result_t save_state(void *file_handle) override {
    uint8_t buffer[STATE_SIZE];
    memset(buffer, 0, sizeof(buffer));
    uint32_t offset = 0;
    buffer[offset++] = shift_reg;
    buffer[offset++] = ctrl_reg;
    buffer[offset++] = chr_bank0;
    buffer[offset++] = chr_bank1;
    buffer[offset++] = prg_bank;
    return nes::fs_write(file_handle, buffer, STATE_SIZE);
  }

  result_t load_state(void *file_handle) override {
    uint8_t buffer[STATE_SIZE];
    SHAPONES_TRY(nes::fs_read(file_handle, buffer, STATE_SIZE));
    uint32_t offset = 0;
    shift_reg = buffer[offset++];
    ctrl_reg = buffer[offset++];
    chr_bank0 = buffer[offset++];
    chr_bank1 = buffer[offset++];
    prg_bank = buffer[offset++];
    perform_remap();
    return result_t::SUCCESS;
  }
};

}  // namespace nes::mapper

#endif
// #include "shapones/mappers/map002.hpp"

#ifndef SHAPONES_MAP002_HPP
#define SHAPONES_MAP002_HPP

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"


namespace nes::mapper {

using namespace nes::memory;

// see: https://www.nesdev.org/wiki/UxROM
class Map002 : public Mapper {
 private:
  uint8_t bank = 0;

 public:
  static constexpr uint32_t STATE_SIZE = 8;

  Map002() : Mapper(2, "UxROM") {}

  result_t init() override { return result_t::SUCCESS; }

  result_t reset() override {
    bank = 0;
    prgrom_remap(0xC000, prgrom_phys_size - 0x4000, 0x4000);
    return result_t::SUCCESS;
  }

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0xffff) {
      bank = value & 0x0F;
      perform_remap();
    }
  }

  void perform_remap() { prgrom_remap(0x8000, bank * 0x4000, 0x4000); }

  uint32_t get_state_size() override { return STATE_SIZE; }

  result_t save_state(void *file_handle) override {
    uint8_t buff[STATE_SIZE];
    memset(buff, 0, sizeof(buff));
    buff[0] = bank;
    return nes::fs_write(file_handle, buff, STATE_SIZE);
  }

  result_t load_state(void *file_handle) override {
    uint8_t buff[STATE_SIZE];
    SHAPONES_TRY(nes::fs_read(file_handle, buff, STATE_SIZE));
    bank = buff[0];
    perform_remap();
    return result_t::SUCCESS;
  }
};

}  // namespace nes::mapper

#endif
// #include "shapones/mappers/map003.hpp"

#ifndef SHAPONES_MAP003_HPP
#define SHAPONES_MAP003_HPP

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"


namespace nes::mapper {

using namespace nes::memory;

class Map003 : public Mapper {
 private:
  uint8_t bank = 0;

 public:
  static constexpr uint32_t STATE_SIZE = 8;

  Map003() : Mapper(3, "CNROM") {}

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0xffff) {
      bank = value & 0x3;
      perform_remap();
    }
  }

  void perform_remap() { chrrom_remap(0x0000, bank * 0x2000, 0x2000); }

  uint32_t get_state_size() override { return STATE_SIZE; }

  result_t save_state(void *file_handle) override {
    uint8_t buff[STATE_SIZE];
    memset(buff, 0, sizeof(buff));
    buff[0] = bank;
    return nes::fs_write(file_handle, buff, STATE_SIZE);
  }

  result_t load_state(void *file_handle) override {
    uint8_t buff[STATE_SIZE];
    SHAPONES_TRY(nes::fs_read(file_handle, buff, STATE_SIZE));
    bank = buff[0];
    perform_remap();
    return result_t::SUCCESS;
  }
};

}  // namespace nes::mapper

#endif
// #include "shapones/mappers/map004.hpp"

#ifndef SHAPONES_MAP004_HPP
#define SHAPONES_MAP004_HPP

// #include "shapones/interrupt.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"


namespace nes::mapper {

using namespace nes::memory;

class Map004 : public Mapper {
 private:
  uint8_t reg_sel;
  uint8_t map_reg[8];
  uint8_t nametable_arrangement;
  uint8_t ram_protection;

  volatile bool irq_enable = false;
  volatile bool irq_reloading = false;
  volatile uint8_t irq_latch = 255;

  uint8_t irq_counter = 0;

 public:
  static constexpr uint32_t STATE_SIZE = 32;

  Map004() : Mapper(4, "MMC3") {}

  result_t init() override {
    prgrom_remap(0xE000, prgrom_phys_size - 8192, 8192);
    return result_t::SUCCESS;
  }

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0x9FFF) {
      if ((addr & 0x0001) == 0) {
        reg_sel = value;
      } else {
        int ireg = reg_sel & 0x07;
        if (ireg <= 1) {
          value &= 0xfe;  // ignore LSB for 2KB bank
        }
        map_reg[ireg] = value;
      }
      perform_remap();
    } else if (0xA000 <= addr && addr <= 0xBFFF) {
      if ((addr & 0x0001) == 0) {
        nametable_arrangement = value;
        perform_nametable_arrangement();
      } else {
        // PRG RAM protect
        ram_protection = value;
      }
    } else if (0xC000 <= addr && addr <= 0xDFFF) {
      if ((addr & 0x0001) == 0) {
        // IRQ latch
        irq_latch = value;
      } else {
        // IRQ reload
        irq_reloading = true;
      }
    } else if (0xE000 <= addr && addr <= 0xFFFF) {
      bool enable_old = irq_enable;
      bool enable_new = !!(addr & 0x0001);
      if (!enable_new && enable_old) {
        interrupt::deassert_irq(interrupt::source_t::MAPPER);
      }
      irq_enable = enable_new;
    }
  }

  bool hblank(const nes::ppu::registers_t &reg, int y) override {
    bool irq = false;
    if (reg.mask.bg_enable && reg.mask.sprite_enable) {
      if (irq_counter == 0 || irq_reloading) {
        irq_reloading = false;
        irq_counter = irq_latch;
      } else {
        irq_counter--;
      }
      if (irq_enable && irq_counter == 0) {
        interrupt::assert_irq(interrupt::source_t::MAPPER);
        irq = true;
      }
    }
    return irq;
  }

  void perform_remap() {
    constexpr int pbs = 8192;
    prgrom_remap(0xA000, map_reg[7] * pbs, pbs);
    if ((reg_sel & 0x40) == 0) {
      prgrom_remap(0x8000, map_reg[6] * pbs, pbs);
      prgrom_remap(0xC000, prgrom_phys_size - pbs * 2, pbs);
    } else {
      prgrom_remap(0x8000, prgrom_phys_size - pbs * 2, pbs);
      prgrom_remap(0xC000, map_reg[6] * pbs, pbs);
    }

    constexpr int cbs = 1024;
    if ((reg_sel & 0x80) == 0) {
      chrrom_remap(0x0000, map_reg[0] * cbs, cbs * 2);
      chrrom_remap(0x0800, map_reg[1] * cbs, cbs * 2);
      chrrom_remap(0x1000, map_reg[2] * cbs, cbs);
      chrrom_remap(0x1400, map_reg[3] * cbs, cbs);
      chrrom_remap(0x1800, map_reg[4] * cbs, cbs);
      chrrom_remap(0x1C00, map_reg[5] * cbs, cbs);
    } else {
      chrrom_remap(0x0000, map_reg[2] * cbs, cbs);
      chrrom_remap(0x0400, map_reg[3] * cbs, cbs);
      chrrom_remap(0x0800, map_reg[4] * cbs, cbs);
      chrrom_remap(0x0C00, map_reg[5] * cbs, cbs);
      chrrom_remap(0x1000, map_reg[0] * cbs, cbs * 2);
      chrrom_remap(0x1800, map_reg[1] * cbs, cbs * 2);
    }
  }

  void perform_nametable_arrangement() {
    if (nametable_arrangement & 0x01) {
      set_nametable_arrangement(nametable_arrangement_t::VERTICAL);
    } else {
      set_nametable_arrangement(nametable_arrangement_t::HORIZONTAL);
    }
  }

  uint32_t get_state_size() override { return STATE_SIZE; }

  result_t save_state(void *file_handle) override {
    uint8_t buffer[STATE_SIZE];
    memset(buffer, 0, sizeof(buffer));
    uint32_t offset = 0;
    buffer[offset++] = reg_sel;
    for (int i = 0; i < 8; i++) {
      buffer[offset++] = map_reg[i];
    }
    buffer[offset++] = nametable_arrangement;
    buffer[offset++] = ram_protection;
    buffer[offset++] = irq_enable ? 1 : 0;
    buffer[offset++] = irq_reloading ? 1 : 0;
    buffer[offset++] = irq_latch;
    buffer[offset++] = irq_counter;
    return nes::fs_write(file_handle, buffer, STATE_SIZE);
  }

  result_t load_state(void *file_handle) override {
    uint8_t buffer[STATE_SIZE];
    SHAPONES_TRY(nes::fs_read(file_handle, buffer, STATE_SIZE));
    uint32_t offset = 0;
    reg_sel = buffer[offset++];
    for (int i = 0; i < 8; i++) {
      map_reg[i] = buffer[offset++];
    }
    nametable_arrangement = buffer[offset++];
    ram_protection = buffer[offset++];
    irq_enable = buffer[offset++] != 0;
    irq_reloading = buffer[offset++] != 0;
    irq_latch = buffer[offset++];
    irq_counter = buffer[offset++];
    perform_remap();
    perform_nametable_arrangement();
    return result_t::SUCCESS;
  }
};

}  // namespace nes::mapper

#endif

namespace nes::mapper {

Mapper *instance = nullptr;

result_t init() {
  deinit();
  instance = new Map000();
  return result_t::SUCCESS;
}

void deinit() {
  if (instance) {
    delete instance;
    instance = nullptr;
  }
}

result_t map_ines(const uint8_t *ines) {
  uint8_t flags6 = ines[6];
  uint8_t flags7 = ines[7];

  int id = (flags7 & 0xf0) | ((flags6 >> 4) & 0xf);

  Mapper *old = instance;
  switch (id) {
    case 0: instance = new Map000(); break;
    case 1: instance = new Map001(); break;
    case 2: instance = new Map002(); break;
    case 3: instance = new Map003(); break;
    case 4: instance = new Map004(); break;
    default:
      instance = new Map000();
      SHAPONES_ERRORF("Unsupported Mapper Number\n");
      break;
  }

  SHAPONES_TRY(instance->init());

  SHAPONES_PRINTF("Mapper No.%d (%s) initialized.\n", id, instance->name);

  if (old) {
    delete old;
  }

  return result_t::SUCCESS;
}

}  // namespace nes::mapper
// #include "shapones/memory.hpp"

// #include "shapones/host_intf.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/ppu.hpp"


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
  SHAPONES_TRY(nes::ram_alloc(1, (void **)&prgram));
  SHAPONES_TRY(nes::ram_alloc(1, (void **)&chrram));
  return result_t::SUCCESS;
}

void deinit() {
  if (prgram) {
    nes::ram_free(prgram);
    prgram = nullptr;
  }
  if (chrram) {
    nes::ram_free(chrram);
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
    return result_t::ERR_INES_INVALID_FORMAT;
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
    nes::ram_free(prgram);
  }
  SHAPONES_TRY(nes::ram_alloc(prgram_size, (void **)&prgram));
  prgram_addr_mask = prgram_size - 1;

  // 512-byte trainer at $7000-$71FF (stored before PRG data)
  bool has_trainer = (flags6 & 0x4) != 0;

  int start_of_prg_rom = 0x10;
  if (has_trainer) start_of_prg_rom += 0x200;
  prgrom = ines + start_of_prg_rom;
  
  if (chrram) {
    nes::ram_free(chrram);
  }
  if (num_chr_rom_pages == 0) {
    chrram_size = CHRROM_RANGE;
    SHAPONES_TRY(nes::ram_alloc(CHRROM_RANGE, (void **)&chrram));
    chrrom = chrram;
  } else {
    chrram_size = 0;
    SHAPONES_TRY(nes::ram_alloc(1, (void **)&chrram));
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
// #include "shapones/menu.hpp"

// #include "shapones/font8x16.hpp"

#ifndef SHAPONES_FONT8X16_HPP
#define SHAPONES_FONT8X16_HPP

#if !(SHAPONES_NO_STDLIB)
// #include "shapones/common.hpp"

#endif

namespace nes::menu {

const uint16_t FONT8X16_CODE_FIRST = 0x20;
const uint16_t FONT8X16_CODE_LAST = 0xCF;
const uint16_t FONT8X16_DATA[] = {
  // 0x20 ' '
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x21 '!'
  0x0000, 0x0000, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0,
  0x0000, 0x0000, 0x0FC0, 0x0FC0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x22 '"'
  0x0000, 0xB8B8, 0xFCFC, 0xF8F8, 0xB0B0, 0x2C2C, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x23 '#'
  0x0000, 0x0000, 0x79E0, 0x79E0, 0xFFFC, 0xFFFC, 0x3CF0, 0x3CF0,
  0xFFFC, 0xFFFC, 0x2DB4, 0x2DB4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x24 '$'
  0x0000, 0x0740, 0x2FE0, 0xBFF8, 0xF77C, 0x03BC, 0x1BF4, 0x7F90,
  0xFB00, 0xF77C, 0xBFF8, 0x2FE0, 0x0740, 0x0000, 0x0000, 0x0000,
  // 0x25 '%'
  0x0000, 0x0000, 0xF6E0, 0x7FF8, 0x3F3C, 0x1FF8, 0x0FE0, 0x2FC0,
  0xBFD0, 0xF3F0, 0xBFF4, 0x2E7C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x26 '&'
  0x0000, 0x0000, 0x02E0, 0x0BF8, 0x0F3C, 0xFBF8, 0xF7F0, 0x7FB4,
  0x3D3C, 0x7C7C, 0xFFF4, 0xD7D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x27 '''
  0x0000, 0x0B80, 0x0FC0, 0x0F80, 0x0B00, 0x02C0, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x28 '('
  0x0000, 0x3F00, 0x1F80, 0x0FD0, 0x0BE0, 0x07F0, 0x03F0, 0x03F0,
  0x07F0, 0x0BE0, 0x0FD0, 0x1F80, 0x3F00, 0x0000, 0x0000, 0x0000,
  // 0x29 ')'
  0x0000, 0x03F0, 0x0BD0, 0x1FC0, 0x2F80, 0x3F40, 0x3F00, 0x3F00,
  0x3F40, 0x2F80, 0x1FC0, 0x0BD0, 0x03F0, 0x0000, 0x0000, 0x0000,
  // 0x2A '*'
  0x0000, 0x0000, 0x0000, 0x0F00, 0x9F6C, 0xFFFC, 0x6F90, 0x6F90,
  0xFFFC, 0x9F6C, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2B '+'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0F03, 0x0F03, 0xFFF0, 0xFFF0,
  0x0F03, 0x0F03, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2C ','
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0B80, 0x0FC0, 0x0F80, 0x0B00, 0x02C0, 0x0000,
  // 0x2D '-'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFF0, 0xFFF0,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2E '.'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0B80, 0x0FC0, 0x0B80, 0x0000, 0x0000, 0x0000,
  // 0x2F '/'
  0x0000, 0xFC00, 0xFC00, 0x7E00, 0x3F00, 0x2F40, 0x0FC0, 0x0FC0,
  0x07E0, 0x03F0, 0x02F4, 0x00FC, 0x00FC, 0x0000, 0x0000, 0x0000,
  // 0x30 '0'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF03C, 0xFE7C, 0xF6FC,
  0xF03C, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x31 '1'
  0x0000, 0x0000, 0x0F00, 0x0FD0, 0x0FFC, 0x0F1C, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x32 '2'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF030, 0xFC00, 0x3F00,
  0x0FC0, 0x03F0, 0xFFFC, 0xFFFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x33 '3'
  0x0000, 0x0000, 0x3FF8, 0x3FF8, 0x1F00, 0x07C0, 0x1FF0, 0x7FF0,
  0xF400, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x34 '4'
  0x0000, 0x0000, 0x03C0, 0x02D0, 0x01E0, 0x3CF0, 0x3CB4, 0x3C78,
  0xFFFC, 0xFFFC, 0x3C00, 0x3C00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x35 '5'
  0x0000, 0x0000, 0x3FF0, 0x3FF0, 0x00F4, 0x00F8, 0x1FFC, 0x7FFC,
  0xF400, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x36 '6'
  0x0000, 0x0000, 0x07C0, 0x03D0, 0x01F0, 0x1FF4, 0x7FFC, 0xF8BC,
  0xF03C, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x37 '7'
  0x0000, 0x0000, 0xFFFC, 0xFFFC, 0xF03C, 0xF43C, 0xB800, 0x7C00,
  0x3D00, 0x2E00, 0x1F00, 0x0F40, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x38 '8'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF47C, 0xB478, 0x2FE0, 0x7FF4,
  0xF47C, 0xF47C, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x39 '9'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF03C, 0xF8BC, 0xFFF4,
  0x7FD0, 0x3D00, 0x1F00, 0x0F40, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3A ':'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0B80, 0x0FC0, 0x0B80, 0x0000,
  0x0000, 0x0B80, 0x0FC0, 0x0B80, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3B ';'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0B80, 0x0FC0, 0x0B80, 0x0000,
  0x0000, 0x0B80, 0x0FC0, 0x0F80, 0x0B00, 0x02C0, 0x0000, 0x0000,
  // 0x3C '<'
  0x0000, 0x0000, 0x0C00, 0x3F00, 0x0FC0, 0x03F0, 0x00FC, 0x00FC,
  0x03F0, 0x0FC0, 0x3F00, 0x0C00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3D '='
  0x0000, 0x0000, 0x0000, 0x0000, 0x3FF0, 0x3FF0, 0x0000, 0x0000,
  0x3FF0, 0x3FF0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3E '>'
  0x0000, 0x0000, 0x00C0, 0x03F0, 0x0FC0, 0x3F00, 0xFC00, 0xFC00,
  0x3F00, 0x0FC0, 0x03F0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3F '?'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF47C, 0xF03C, 0xFC00, 0x3F00,
  0x0F00, 0x0000, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x40 '@'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xFE3C, 0xF33C, 0xF33C,
  0xDE3C, 0x00BC, 0x3FF4, 0x3FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x41 'A'
  0x0000, 0x0000, 0x0FC0, 0x1FD0, 0x2EE0, 0x3DF0, 0x7CF4, 0xB8B8,
  0xFFFC, 0xFFFC, 0xF03C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x42 'B'
  0x0000, 0x0000, 0x2FFC, 0xBFFC, 0xF83C, 0xF43C, 0x7FFC, 0x7FFC,
  0xF43C, 0xF83C, 0xBFFC, 0x2FFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x43 'C'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xB8BC, 0x003C, 0x003C, 0x003C,
  0x003C, 0xB8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x44 'D'
  0x0000, 0x0000, 0x1FFC, 0x7FFC, 0xF83C, 0xF03C, 0xF03C, 0xF03C,
  0xF03C, 0xF83C, 0x7FFC, 0x1FFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x45 'E'
  0x0000, 0x0000, 0xFFFC, 0xFFFC, 0x003C, 0x003C, 0x3FFC, 0x3FFC,
  0x003C, 0x003C, 0xFFFC, 0xFFFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x46 'F'
  0x0000, 0x0000, 0xFFFC, 0xFFFC, 0x003C, 0x003C, 0x3FFC, 0x3FFC,
  0x003C, 0x003C, 0x003C, 0x003C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x47 'G'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xB8BC, 0x003C, 0xFF3C, 0xFF3C,
  0xF03C, 0xF4BC, 0xFFF4, 0xF7D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x48 'H'
  0x0000, 0x0000, 0xF03C, 0xF03C, 0xF03C, 0xF03C, 0xFFFC, 0xFFFC,
  0xF03C, 0xF03C, 0xF03C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x49 'I'
  0x0000, 0x0000, 0x0FF0, 0x0FF0, 0x03C0, 0x03C0, 0x03C0, 0x03C0,
  0x03C0, 0x03C0, 0x0FF0, 0x0FF0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4A 'J'
  0x0000, 0x0000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000,
  0xF000, 0xF8B8, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4B 'K'
  0x0000, 0x0000, 0x303C, 0xFC3C, 0x3F3C, 0x0FFC, 0x03FC, 0x03FC,
  0x0FFC, 0x3F3C, 0xFC3C, 0x303C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4C 'L'
  0x0000, 0x0000, 0x003C, 0x003C, 0x003C, 0x003C, 0x003C, 0x003C,
  0x003C, 0x003C, 0xFFFC, 0xFFFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4D 'M'
  0x0000, 0x0000, 0xF03C, 0xF47C, 0xF8BC, 0xFCFC, 0xFDFC, 0xFEFC,
  0xFBBC, 0xF77C, 0xF33C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4E 'N'
  0x0000, 0x0000, 0xF03C, 0xF07C, 0xF0FC, 0xF2FC, 0xF3FC, 0xFFFC,
  0xFE3C, 0xFC3C, 0xF43C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4F 'O'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF03C, 0xF03C, 0xF03C,
  0xF03C, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x50 'P'
  0x0000, 0x0000, 0x1FFC, 0x7FFC, 0xF83C, 0xF03C, 0xF83C, 0x7FFC,
  0x1FFC, 0x003C, 0x003C, 0x003C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x51 'Q'
  0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF03C, 0xF03C, 0xFB7C,
  0xFE3C, 0xFCBC, 0xBFF4, 0xEFD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x52 'R'
  0x0000, 0x0000, 0x1FFC, 0x7FFC, 0xF83C, 0xF03C, 0xF83C, 0x7FFC,
  0x1FFC, 0x3C3C, 0xB83C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x53 'S'
  0x0000, 0x0000, 0x2FE0, 0xBFF8, 0xF47C, 0x00BC, 0x1BF4, 0x7F90,
  0xF800, 0xF47C, 0xBFF8, 0x2FE0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x54 'T'
  0x0000, 0x0000, 0xFFF8, 0xFFF8, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x55 'U'
  0x0000, 0x0000, 0xF03E, 0xF03E, 0xF03C, 0xF03C, 0xF03C, 0xF03C,
  0xF03C, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x56 'V'
  0x0000, 0x0000, 0xF03C, 0xF03C, 0xF47C, 0xB8B8, 0x7CF4, 0x3DF0,
  0x2EE0, 0x1FD0, 0x0FC0, 0x0B80, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x57 'W'
  0x0000, 0x0000, 0xF03C, 0xF03C, 0xF33C, 0xF77C, 0xBBB8, 0xBFF8,
  0x7EF4, 0x7DF4, 0x3CF0, 0x3CF0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x58 'X'
  0x0000, 0x0000, 0xF03C, 0xF47C, 0x7CF4, 0x3DF0, 0x1FD0, 0x1FD0,
  0x3DF0, 0x7CF4, 0xF47C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x59 'Y'
  0x0000, 0x0000, 0xC03C, 0xD07C, 0xF0F4, 0xF5F0, 0x7FD0, 0x3FC0,
  0x1F40, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5A 'Z'
  0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x3C01, 0x2F00, 0x0F40, 0x07C0,
  0x03E0, 0x00F0, 0xFFFC, 0xFFFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5B '['
  0x0000, 0x3FF0, 0x3FF0, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0,
  0x03F0, 0x03F0, 0x03F0, 0x3FF0, 0x3FF0, 0x0000, 0x0000, 0x0000,
  // 0x5C '\'
  0x0000, 0x00FC, 0x00FC, 0x02F4, 0x03F0, 0x07E0, 0x0FC0, 0x0FC0,
  0x2F40, 0x3F00, 0x7E00, 0xFC00, 0xFC00, 0x0000, 0x0000, 0x0000,
  // 0x5D ']'
  0x0000, 0xFFF0, 0xFFF0, 0xFC00, 0xFC00, 0xFC00, 0xFC00, 0xFC00,
  0xFC00, 0xFC00, 0xFC00, 0xFFF0, 0xFFF0, 0x0000, 0x0000, 0x0000,
  // 0x5E '^'
  0x0000, 0x0300, 0x0FC0, 0x3FF0, 0xFCFC, 0x3030, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5F '_'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xFFFC, 0xFFFC, 0x0000, 0x0000, 0x0000,
  // 0x60 '`'
  0x0000, 0x0FC0, 0x1F00, 0x2C00, 0x3000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x61 'a'
  0x0000, 0x0000, 0x0000, 0x0000, 0x2FE0, 0xBFF8, 0xF010, 0xFFE0,
  0xFFF8, 0xF83C, 0xFFF8, 0xF7E0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x62 'b'
  0x0000, 0x0000, 0x003C, 0x003C, 0x1F7C, 0x7FFC, 0xF8BC, 0xF03C,
  0xF03C, 0xF8BC, 0x7FFC, 0x1F7C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x63 'c'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1FD0, 0x7FF4, 0x34BC, 0x003C,
  0x003C, 0x34BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x64 'd'
  0x0000, 0x0000, 0xF000, 0xF000, 0xF7D0, 0xFFF4, 0xF8BC, 0xF03C,
  0xF03C, 0xF8BC, 0xFFF4, 0xF7D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x65 'e'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF03C,
  0xFFFC, 0x00BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x66 'f'
  0x0000, 0x0000, 0x2F80, 0xBFE0, 0x35F0, 0x00F0, 0x0FFC, 0x0FFC,
  0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x67 'g'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF7D0, 0xFFF4, 0xF8BC, 0xF03C,
  0xF8BC, 0xFFF4, 0xF7D0, 0xF870, 0x7FF4, 0x1FD0, 0x0000, 0x0000,
  // 0x68 'h'
  0x0000, 0x0000, 0x003C, 0x003C, 0x1F7C, 0x7FFC, 0xF8BC, 0xF03C,
  0xF03C, 0xF03C, 0xF03C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x69 'i'
  0x0000, 0x0000, 0x03C0, 0x03C0, 0x0000, 0x03C0, 0x03C0, 0x03C0,
  0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6A 'j'
  0x0000, 0x0000, 0x0F00, 0x0F00, 0x0000, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F80, 0x07FC, 0x01FC, 0x0000, 0x0000,
  // 0x6B 'k'
  0x0000, 0x0000, 0x003C, 0x003C, 0x303C, 0xFC3C, 0x3F3C, 0x0FFC,
  0x0FFC, 0x3F3C, 0xFC3C, 0x303C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6C 'l'
  0x0000, 0x0000, 0x03F0, 0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x03C0,
  0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6D 'm'
  0x0000, 0x0000, 0x0000, 0x0000, 0x26DC, 0xBFFC, 0xFFFC, 0xF77C,
  0xF33C, 0xF33C, 0xF33C, 0xF33C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6E 'n'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1F7C, 0x7FFC, 0xF8BC, 0xF03C,
  0xF03C, 0xF03C, 0xF03C, 0xF03C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6F 'o'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1FD0, 0x7FF4, 0xF8BC, 0xF03C,
  0xF03C, 0xF8BC, 0x7FF4, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x70 'p'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1F7C, 0x7FFC, 0xF8BC, 0xF03C,
  0xF03C, 0xF8BC, 0x7FFC, 0x1F7C, 0x003C, 0x003C, 0x0000, 0x0000,
  // 0x71 'q'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF7D0, 0xFFF4, 0xF8BC, 0xF03C,
  0xF03C, 0xF8BC, 0xFFF4, 0xF7D0, 0xF000, 0xF000, 0x0000, 0x0000,
  // 0x72 'r'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1F7C, 0x7FFC, 0x34BC, 0x003C,
  0x003C, 0x003C, 0x003C, 0x003C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x73 's'
  0x0000, 0x0000, 0x0000, 0x0000, 0x2FE0, 0xBFF8, 0xF47C, 0x1FF4,
  0x7FD0, 0xF47C, 0xBFF8, 0x2FE0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x74 't'
  0x0000, 0x0000, 0x00F0, 0x00F0, 0x3FFC, 0x3FFC, 0x00F0, 0x00F0,
  0x00F0, 0x35F0, 0xBFE0, 0x2F80, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x75 'u'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF03C, 0xF03C, 0xF03C, 0xF03C,
  0xF03C, 0xF8BC, 0xFFF4, 0xF7D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x76 'v'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF03C, 0xF47C, 0xF8BC, 0xBCF8,
  0x7DF4, 0x3EF0, 0x2FE0, 0x1FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x77 'w'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF03C, 0xF33C, 0xF77C, 0xBBB8,
  0xBFF8, 0x7EF4, 0x7DF4, 0x3CF0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x78 'x'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3030, 0xF8BC, 0x3FF0, 0x1FD0,
  0x1FD0, 0x3FF0, 0xF8BC, 0x3030, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x79 'y'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF03C, 0xF03C, 0xF47C, 0x7CF4,
  0x3FF0, 0x1FD0, 0x0FC0, 0x03F0, 0x00FC, 0x0030, 0x0000, 0x0000,
  // 0x7A 'z'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFC, 0xFFFC, 0xFC00, 0x3F00,
  0x0FC0, 0x03F0, 0xFFFC, 0xFFFC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x7B '{'
  0x0000, 0x0000, 0x3F80, 0x3FE0, 0x07F0, 0x03F0, 0x03F0, 0x01FC,
  0x01FC, 0x03F0, 0x03F0, 0x07F0, 0x3FE0, 0x3F80, 0x0000, 0x0000,
  // 0x7C '|'
  0x0000, 0x0000, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0,
  0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0, 0x0000, 0x0000,
  // 0x7D '}'
  0x0000, 0x0000, 0x0BF0, 0x2FF0, 0x3F40, 0x3F00, 0x3F00, 0xFD00,
  0xFD00, 0x3F00, 0x3F00, 0x3F40, 0x2FF0, 0x0BF0, 0x0000, 0x0000,
  // 0x7E '~'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF1F4, 0xF3FC,
  0xFF3C, 0x7D3C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x7F ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x80 ''
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0xA400, 0xA900, 0xAA00, 0x6A00,
  // 0x81 ''
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0x82 ''
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x001A, 0x006A, 0x00AA, 0x00A9,
  // 0x83 ''
  0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00,
  0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00, 0x2A00,
  // 0x84 ''
  0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8,
  0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8, 0x00A8,
  // 0x85 ''
  0x6A00, 0xAA00, 0xA900, 0xA400, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x86 ''
  0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x87 ''
  0x00A9, 0x00AA, 0x006A, 0x001A, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x88 ''
  0x0000, 0x0000, 0xA2A4, 0xA028, 0xA028, 0xA1A4, 0xA280, 0xA280,
  0xA1A8, 0x0000, 0x0000, 0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0x89 ''
  0x0000, 0x0000, 0x028A, 0x0280, 0x0280, 0x028A, 0x0280, 0x0280,
  0x2A8A, 0x0000, 0x0000, 0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0x8A ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8B ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8C ''
  0x6AA9, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xA96A, 0xA82A, 0xA41A,
  0xA00A, 0x9006, 0x8002, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x6AA9,
  // 0x8D ''
  0x6AA9, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x8002, 0x9006, 0xA00A,
  0xA41A, 0xA82A, 0xA96A, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x6AA9,
  // 0x8E ''
  0x4444, 0x1111, 0x4444, 0x1111, 0x4444, 0x1111, 0x4444, 0x1111,
  0x4444, 0x1111, 0x4444, 0x1111, 0x4444, 0x1111, 0x4444, 0x1111,
  // 0x8F ''
  0x5555, 0x6AA9, 0xAAAA, 0xAAAA, 0xAAAA, 0xAFFA, 0xA55A, 0xAFFA,
  0xA55A, 0xAFFA, 0xA55A, 0xAAAA, 0xAAAA, 0xAAAA, 0x6AA9, 0x5555,
  // 0x90 ''
  0x0000, 0xAA90, 0xAAA4, 0xAAA8, 0xAAA8, 0xAAA8, 0xAAA8, 0xAAA8,
  0xAAA8, 0xAAA8, 0xAAA8, 0x0068, 0x0028, 0x0028, 0x0028, 0x0028,
  // 0x91 ''
  0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA,
  0xAAAA, 0xAAAA, 0xAAAA, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x92 ''
  0x0000, 0x06AA, 0x1AAA, 0x2AAA, 0x2AAA, 0x2AAA, 0x2AAA, 0x2AAA,
  0x2AAA, 0x2AAA, 0x2AAA, 0x2900, 0x2800, 0x2800, 0x2800, 0x2800,
  // 0x93 ''
  0x0028, 0x0028, 0x0028, 0x0028, 0x0028, 0x0028, 0x0028, 0x0028,
  0x0028, 0x0028, 0x0028, 0x0028, 0x0028, 0x0028, 0x0028, 0x0028,
  // 0x94 ''
  0x2800, 0x2800, 0x2800, 0x2800, 0x2800, 0x2800, 0x2800, 0x2800,
  0x2800, 0x2800, 0x2800, 0x2800, 0x2800, 0x2800, 0x2800, 0x2800,
  // 0x95 ''
  0x0028, 0x0028, 0x0028, 0x0028, 0x0068, 0xAAA4, 0xAA90, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x96 ''
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xAAAA, 0xAAAA, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x97 ''
  0x2800, 0x2800, 0x2800, 0x2800, 0x2900, 0x1AAA, 0x06AA, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x98 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x99 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x9A ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x9B ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x9C ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x9D ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x9E ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x9F ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xA0 ' '
  0x0000, 0x5540, 0x5550, 0x5554, 0x0154, 0x4054, 0x4054, 0x5454,
  0x5454, 0x4054, 0x4054, 0x0154, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA1 '¡'
  0x0000, 0x5555, 0x5555, 0x5555, 0x0000, 0x0001, 0x0001, 0x0015,
  0x1015, 0x5401, 0x1001, 0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA2 '¢'
  0x0000, 0x0555, 0x1555, 0x5555, 0x5500, 0x5400, 0x5400, 0x5400,
  0x5410, 0x5454, 0x5410, 0x5500, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA3 '£'
  0x0000, 0x5540, 0x1550, 0x1554, 0x1554, 0x1554, 0x1554, 0x1554,
  0x1554, 0x1554, 0x1554, 0x1554, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA4 '¤'
  0x0000, 0x5555, 0x0000, 0x1550, 0x1150, 0x1150, 0x0000, 0x5554,
  0x5004, 0x5554, 0x5404, 0x5554, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA5 '¥'
  0x0000, 0x0555, 0x1555, 0x5554, 0x5550, 0x5550, 0x5550, 0x5550,
  0x5550, 0x5550, 0x5550, 0x5550, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA6 '¦'
  0x0000, 0x5540, 0x5550, 0x5554, 0x5554, 0x0554, 0x0154, 0x0054,
  0x0154, 0x0554, 0x1554, 0x5554, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA7 '§'
  0x0000, 0x5555, 0x5555, 0x4005, 0x0000, 0x0000, 0x0000, 0x4000,
  0x4000, 0x5000, 0x1540, 0x5014, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA8 '¨'
  0x0000, 0x0555, 0x1555, 0x5555, 0x5554, 0x5545, 0x5501, 0x5401,
  0x5500, 0x5540, 0x5550, 0x5550, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xA9 '©'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xAA 'ª'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xAB '«'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xAC '¬'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xAD '­'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xAE '®'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xAF '¯'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xB0 '°'
  0xAA90, 0xAAA4, 0xAAA8, 0x01A8, 0x80A8, 0x80A8, 0xA8A8, 0xA8A8,
  0x80A8, 0x80A8, 0x01A8, 0xAAA8, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xB1 '±'
  0xAAAA, 0xAAAA, 0xAAAA, 0x0000, 0x0002, 0x0002, 0x002A, 0x642A,
  0xA802, 0x6402, 0x0000, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xB2 '²'
  0x1AAA, 0x6AAA, 0xAAAA, 0xA900, 0xA800, 0xA800, 0xA800, 0xA864,
  0xA8A8, 0xA864, 0xA900, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xB3 '³'
  0xAA90, 0x6AA4, 0x2AA8, 0x2AA8, 0x2AA8, 0x2AA8, 0x2AA8, 0x2AA8,
  0x2AA8, 0x2AA8, 0x2AA8, 0x6AA8, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xB4 '´'
  0xAAAA, 0x1550, 0x22A0, 0x22A0, 0x26A0, 0x0000, 0x6AA4, 0xA558,
  0xAAA8, 0xA958, 0xAAA8, 0x5555, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xB5 'µ'
  0x1AAA, 0x6AAA, 0xAAA8, 0xAAA0, 0xAAA0, 0xAAA0, 0xAAA0, 0xAAA0,
  0xAAA0, 0xAAA0, 0xAAA0, 0xAAA5, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000,
  // 0xB6 '¶'
  0xAA90, 0xAAA4, 0xAAA8, 0x6AA8, 0x0AA8, 0x02A8, 0x00A8, 0x02A8,
  0x0AA8, 0x2AA8, 0xAAA8, 0xAAA8, 0xAAAA, 0xAAAA, 0xAAAA, 0x5555,
  // 0xB7 '·'
  0xAAAA, 0xAAAA, 0x5016, 0x0000, 0x0000, 0x4000, 0x8000, 0x9000,
  0x6000, 0x2A90, 0xA468, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x5555,
  // 0xB8 '¸'
  0x1AAA, 0x6AAA, 0xAAAA, 0xAAA4, 0xAA86, 0xAA02, 0xA801, 0xAA00,
  0xAA80, 0xAAA0, 0xAAA8, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x5555,
  // 0xB9 '¹'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xBA 'º'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xBB '»'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xBC '¼'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xBD '½'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xBE '¾'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xBF '¿'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xC0 'À'
  0x0000, 0x0000, 0x2000, 0xA800, 0xAA00, 0xAA80, 0xAAA0, 0xA800,
  0xA800, 0xA800, 0xA800, 0xA800, 0xA400, 0x9000, 0x0000, 0x0000,
  // 0xC1 'Á'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0002, 0x000A, 0x002A, 0x0000,
  0x0000, 0x0000, 0x0001, 0x0AAA, 0x0AAA, 0x0AAA, 0x0000, 0x0000,
  // 0xC2 'Â'
  0x0000, 0x0000, 0x1F80, 0x55E0, 0x8070, 0x0030, 0xFE30, 0x0330,
  0x0330, 0x0330, 0x0330, 0x0330, 0xAA90, 0x0000, 0x0000, 0x0000,
  // 0xC3 'Ã'
  0x0000, 0x0000, 0x0000, 0x0000, 0x007F, 0x0055, 0x07FF, 0x0840,
  0x0840, 0x0840, 0x0840, 0x0840, 0x06AA, 0x0000, 0x0000, 0x0000,
  // 0xC4 'Ä'
  0x0000, 0x0000, 0xFF80, 0xFFC0, 0x03C0, 0xFFC0, 0x03C0, 0xFFC0,
  0x03C0, 0xFFC0, 0x03C0, 0xFFC0, 0xFFC0, 0xFF80, 0x0000, 0x0000,
  // 0xC5 'Å'
  0x0000, 0x0000, 0x02FF, 0x03FF, 0x03F0, 0x03FF, 0x03FC, 0x03FF,
  0x03C0, 0x03FF, 0x0003, 0x01A3, 0x0063, 0x0013, 0x0000, 0x0000,
  // 0xC6 'Æ'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xC7 'Ç'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xC8 'È'
  0x0000, 0x0000, 0xE000, 0xF000, 0xF000, 0xF000, 0xFFE0, 0xFFF0,
  0xFFF0, 0xFFE0, 0xF000, 0xF000, 0xF000, 0xE000, 0x0000, 0x0000,
  // 0xC9 'É'
  0x0000, 0x0000, 0x000B, 0x000F, 0x000F, 0x000F, 0x0BFF, 0x0FFF,
  0x0FFF, 0x0BFF, 0x000F, 0x000F, 0x000F, 0x000B, 0x0000, 0x0000,
  // 0xCA 'Ê'
  0x0000, 0x0000, 0xFFE0, 0xFC30, 0xFC30, 0xFC30, 0xF830, 0x0030,
  0xFD30, 0x0330, 0xFF30, 0x0330, 0xFF30, 0xFFE0, 0x0000, 0x0000,
  // 0xCB 'Ë'
  0x0000, 0x0000, 0x01FF, 0x0733, 0x0C33, 0x0C33, 0x0C2F, 0x0C00,
  0x0C7F, 0x0CF0, 0x0CFF, 0x0CFF, 0x0CFF, 0x0BFF, 0x0000, 0x0000,
  // 0xCC 'Ì'
  0x0000, 0x0000, 0x01D0, 0x1FF0, 0xFFF0, 0xFFF0, 0xFFF0, 0xFFF0,
  0xFFF0, 0xFFF0, 0xFFF0, 0xFFF0, 0x1FF0, 0x01D0, 0x0000, 0x0000,
  // 0xCD 'Í'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x001F, 0x01FF, 0x1FFF,
  0x1FFF, 0x01FF, 0x001F, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0xCE 'Î'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0xCF 'Ï'
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
};

}  // namespace nes

#endif // SHAPONES_FONT8X16_HPP
// #include "shapones/fs.hpp"

// #include "shapones/host_intf.hpp"

// #include "shapones/input.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/state.hpp"


namespace nes::menu {

static constexpr int CHAR_WIDTH = 8;
static constexpr int CHAR_HEIGHT = 16;
#if SHAPONES_MENU_LARGE_FONT
static constexpr int BUFF_WIDTH = SCREEN_WIDTH / (CHAR_WIDTH * 2);
static constexpr int BUFF_HEIGHT = SCREEN_HEIGHT / (CHAR_HEIGHT * 2);
static constexpr int CLIENT_X = 0;
static constexpr int CLIENT_Y = 1;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH - 0;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT - 1;
#else
static constexpr int BUFF_WIDTH = SCREEN_WIDTH / CHAR_WIDTH;
static constexpr int BUFF_HEIGHT = SCREEN_HEIGHT / CHAR_HEIGHT;
static constexpr int CLIENT_X = 1;
static constexpr int CLIENT_Y = 2;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH - 2;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT - 3;
#endif

static constexpr int MENU_MAX_ITEMS = 256;

static constexpr int POPUP_MAX_ITEMS = 4;
static constexpr int POPUP_MAX_MESSAGE_LENGTH = 64;
static constexpr int POPUP_WIDTH = BUFF_WIDTH - 8;
static constexpr int POPUP_X = (BUFF_WIDTH - POPUP_WIDTH) / 2;
static constexpr int POPUP_Y = (BUFF_HEIGHT - POPUP_MAX_ITEMS - 3) / 2;

enum class tab_t {
  NES_LIST,
  SAVE_LIST,
  MONITOR,
  COUNT,
};

static constexpr int NUM_TABS = (int)tab_t::COUNT;

enum class icon_t {
  NONE,
  PARENT,
  FOLDER,
  FILE,
  ADD,
  WARNING,
  ERROR,
  PLAY,
  SAVE,
};

enum class action_t {
  NONE,
  OPEN_DIR,
  LOAD_ROM,
  ADD_STATE_SLOT,
  STATE_SELECT,
  LOAD_STATE,
  SAVE_STATE,
  DELETE_STATE,
  FORCE_NMI_CONFIRM,
  FORCE_NMI_TRIGGER,
  CLOSE_POPUP,
};

// Workaround for the issue where repeated use of strdup in PicoLibSDK causes a
// panic:
static char *strdup_safe(const char *src) {
  if (src == nullptr) return nullptr;
  size_t len = strlen(src);
  char *dst = new char[len + 1];
  strcpy(dst, src);
  return dst;
}

static void request_redraw();

class ListItem {
 public:
  const icon_t icon;
  const action_t action;
  const char *label;
  const int32_t tag;
  ListItem(icon_t icon, action_t action, const char *label, int32_t tag = 0)
      : icon(icon), action(action), label(strdup_safe(label)), tag(tag) {}
  ~ListItem() {
    if (label != nullptr) {
      delete[] label;
      label = nullptr;
    }
  }
};

class ListBox {
 public:
  const int capacity;
  ListItem **items;
  int num_items;
  int sel_index;
  int scroll_pos;
  int x, y, width, height;

  ListBox(int capacity)
      : capacity(capacity),
        items(new ListItem *[capacity]),
        num_items(0),
        sel_index(0),
        scroll_pos(0) {}

  ~ListBox() {
    for (int i = 0; i < num_items; i++) {
      delete items[i];
    }
    delete[] items;
  }

  void set_bounds(int x, int y, int w, int h) {
    this->x = x;
    this->y = y;
    this->width = w;
    this->height = h;
  }

  ListItem *get_selected_item() const {
    if (0 <= sel_index && sel_index < num_items) {
      return items[sel_index];
    } else {
      return nullptr;
    }
  }

  void clear() {
    for (int i = 0; i < num_items; i++) {
      delete items[i];
    }
    num_items = 0;
    sel_index = 0;
    scroll_pos = 0;
    request_redraw();
  }

  void add_item(icon_t icon, action_t action, const char *label,
                int32_t tag = 0) {
    if (num_items >= capacity) return;
    items[num_items++] = new ListItem(icon, action, label, tag);
    request_redraw();
  }

  void on_key_down(input::status_t key) {
    if (key.up) {
      if (num_items > 0) {
        sel_index = (sel_index + num_items - 1) % num_items;
        scroll_to(sel_index);
        request_redraw();
      }
    } else if (key.down) {
      if (num_items > 0) {
        sel_index = (sel_index + 1) % num_items;
        scroll_to(sel_index);
        request_redraw();
      }
    }
  }

  void scroll_to(int index) {
    if (index < scroll_pos) {
      scroll_pos = index;
      request_redraw();
    } else if (index >= scroll_pos + height) {
      scroll_pos = index - height + 1;
      request_redraw();
    }
  }
};

bool shown = false;
tab_t tab = tab_t::NES_LIST;
int last_menu_index[NUM_TABS] = {0};
volatile bool vsync = false;

const uint8_t PALETTE_FILE[] = {0x1D, 0x00, 0x10, 0x20, 0x21, 0x11, 0x01, 0x3F,
                                0x2D, 0x00, 0x10, 0x3D, 0x3F, 0x0C, 0x1C, 0x2C};
static constexpr int NUM_PALETTES = sizeof(PALETTE_FILE) / 4;

uint8_t text_buff[BUFF_WIDTH * BUFF_HEIGHT];
uint8_t palette_buff[BUFF_WIDTH * BUFF_HEIGHT];

bool key_release_waiting = false;
input::status_t key_pressed;
input::status_t key_down;
input::status_t key_up;

bool disk_mounted = false;
char current_dir[nes::MAX_PATH_LENGTH + 1] = "";

bool redraw_requested = false;

ListBox menu(MENU_MAX_ITEMS);

icon_t popup_icon = icon_t::NONE;
char popup_message[POPUP_MAX_MESSAGE_LENGTH + 1] = "";
ListBox popup_list(POPUP_MAX_ITEMS);
bool popup_shown = false;

uint8_t ss_buff[state::SS_SIZE_BYTES];
bool ss_enable = false;

static result_t load_tab(tab_t tab, bool force = false);
static result_t load_file_list_tab();
static result_t load_state_list_tab();
static result_t load_monitor_tab();
static result_t load_state_screenshot();

static int find_empty_slot();

static void read_input();
static result_t process_input();
static result_t on_menu_selected(ListItem *item);

static result_t on_open_dir(ListItem *mi);
static result_t on_load_rom(ListItem *mi);
static result_t on_add_state_slot();
static result_t on_state_select(ListItem *mi);
static result_t on_save_state(ListItem *mi);
static result_t on_load_state(ListItem *mi);
static result_t on_delete_state();
static result_t on_force_nmi_trigger();

static result_t show_message(icon_t icon, const char *msg);
static result_t show_confirm(icon_t icon, const char *msg,
                             action_t confirm_action);

static void popup_show(icon_t icon, const char *msg);
static void popup_close();

static void perform_redraw();
static int draw_text(int x, int y, const char *str, int max_len = 999999);
static void draw_char(int x, int y, char c);
static void draw_icon(int x, int y, icon_t icon);
static void draw_frame(int x, int y, int w, int h, char offset);
static void draw_list(const ListBox &list);
static void fill_char(int x, int y, int w, int h, char c = ' ');
static void fill_palette(int x, int y, int w, int h, uint8_t p);
static void clip_rect(int *x, int *y, int *w, int *h);

result_t init() {
  deinit();
  key_pressed.raw = 0;
  key_down.raw = 0;
  key_up.raw = 0;

  shown = false;
  tab = tab_t::NES_LIST;
  for (int i = 0; i < BUFF_WIDTH * BUFF_HEIGHT; i++) {
    text_buff[i] = '\0';
    palette_buff[i] = 0x00;
  }
  return result_t::SUCCESS;
}
void deinit() { menu.clear(); }

void show() {
  if (shown) return;
  shown = true;
  key_release_waiting = true;
  read_input();
  read_input();
  load_tab(tab, true);
}

void hide() {
  if (!shown) return;
  popup_close();
  menu.clear();
  shown = false;
}

static result_t load_tab(tab_t t, bool force) {
  if (tab == t && !force) return result_t::SUCCESS;

  tab = t;

  switch (tab) {
    case tab_t::NES_LIST: SHAPONES_TRY(load_file_list_tab()); break;
    case tab_t::SAVE_LIST: SHAPONES_TRY(load_state_list_tab()); break;
    case tab_t::MONITOR: SHAPONES_TRY(load_monitor_tab()); break;
  }

  int i = last_menu_index[static_cast<int>(tab)];
  if (0 <= i && i < menu.num_items) {
    menu.sel_index = i;
    menu.scroll_to(i);
  }

  request_redraw();

  return result_t::SUCCESS;
}

static result_t load_file_list_tab() {
  menu.set_bounds(CLIENT_X, CLIENT_Y + 1, CLIENT_WIDTH, CLIENT_HEIGHT - 1);

  menu.clear();

  if (!disk_mounted) {
    if (fs_mount() == result_t::SUCCESS) {
      disk_mounted = true;
    } else {
      return result_t::SUCCESS;
    }
  }

  if (current_dir[0] == '\0') {
    SHAPONES_TRY(fs_get_current_dir(current_dir));
    SHAPONES_TRY(fs::append_separator(current_dir));
  }

  if (!fs::is_root_dir(current_dir)) {
    // add parent directory entry
    menu.add_item(icon_t::PARENT, action_t::OPEN_DIR, "../");
  }

  result_t res = fs_enum_files(current_dir, [](const file_info_t &info) {
    char *name = (char *)info.name;
    if (info.is_dir) {
      // append '/' to directory names
      size_t len = strnlen(name, nes::MAX_FILENAME_LENGTH + 1);
      char name[nes::MAX_FILENAME_LENGTH + 2];
      strncpy(name, info.name, nes::MAX_FILENAME_LENGTH);
      name[len++] = '/';
      name[len] = '\0';
    }
    icon_t icon = info.is_dir ? icon_t::FOLDER : icon_t::FILE;
    action_t action = info.is_dir ? action_t::OPEN_DIR : action_t::LOAD_ROM;
    menu.add_item(icon, action, name);
    return (menu.num_items < menu.capacity);
  });
  if (res != result_t::SUCCESS) {
    menu.clear();
    return res;
  }

  // sort by name
  for (int i = 0; i < menu.num_items - 1; i++) {
    for (int j = i + 1; j < menu.num_items; j++) {
      const ListItem *mi = menu.items[i];
      const ListItem *mj = menu.items[j];
      bool swap = false;
      if (mi->icon != mj->icon) {
        swap = mi->icon > mj->icon;
      } else {
        swap = strcmp(mi->label, mj->label) > 0;
      }
      if (swap) {
        ListItem *temp = menu.items[i];
        menu.items[i] = menu.items[j];
        menu.items[j] = temp;
      }
    }
  }

  ss_enable = false;
  request_redraw();
  return res;
}

static result_t load_state_list_tab() {
  result_t res = result_t::SUCCESS;

  menu.set_bounds(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, CLIENT_HEIGHT);

  menu.clear();
  char state_path[nes::MAX_PATH_LENGTH + 1];
  strncpy(state_path, get_ines_path(), nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(fs::replace_ext(state_path, state::STATE_FILE_EXT));

  do {
    if (nes::fs_exists(state_path)) {
      res = state::enum_slots(
          state_path, [](const state::state_slot_entry_t &entry) {
            if (!entry.is_used()) {
              // In PicoLibSDK, true/false are defined as int,
              // so a cast to bool is required.
              return (bool)true;
            }

            char label[nes::MAX_FILENAME_LENGTH + 1];
            float t = (float)entry.frame_count / 60;
            if (t < 60) {
              snprintf(label, sizeof(label), "#%02d %.2fs", entry.index, t);
            } else if (t < 3600) {
              snprintf(label, sizeof(label), "#%02d %.2fm", entry.index,
                       t / 60);
            } else {
              snprintf(label, sizeof(label), "#%02d %.2fh", entry.index,
                       t / 3600);
            }
            menu.add_item(icon_t::FILE, action_t::STATE_SELECT, label,
                          entry.index);
            return (menu.num_items < menu.capacity);
          });
      if (res != result_t::SUCCESS) {
        break;
      }
    }

    if (menu.num_items < state::MAX_SLOTS) {
      menu.add_item(icon_t::ADD, action_t::ADD_STATE_SLOT, "New Slot", -1);
    }

    load_state_screenshot();
  } while (0);

  if (res != result_t::SUCCESS) {
    menu.clear();
    popup_close();
    popup_list.add_item(icon_t::NONE, action_t::DELETE_STATE, "DELETE FILE");
    popup_list.add_item(icon_t::NONE, action_t::CLOSE_POPUP, "Cancel");
    popup_list.sel_index = 1;
    popup_show(icon_t::ERROR, "Data Broken.");
  }

  request_redraw();

  return res;
}

static result_t load_monitor_tab() {
  menu.clear();
  menu.add_item(icon_t::NONE, action_t::FORCE_NMI_CONFIRM, "Force NMI");
  request_redraw();
  return result_t::SUCCESS;
}

static result_t load_state_screenshot() {
  char state_path[nes::MAX_PATH_LENGTH + 1];
  strncpy(state_path, get_ines_path(), nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(fs::replace_ext(state_path, state::STATE_FILE_EXT));

  ss_enable = false;
  auto *mi = menu.get_selected_item();
  if (mi && mi->tag >= 0) {
    result_t res = state::read_screenshot(state_path, mi->tag, ss_buff);
    ss_enable = (res == result_t::SUCCESS);
  }
  return result_t::SUCCESS;
}

result_t service() {
  if (!vsync) return result_t::SUCCESS;
  vsync = false;

  if (!shown) return result_t::SUCCESS;

  read_input();
  process_input();

  if (redraw_requested) {
    redraw_requested = false;
    perform_redraw();
  }

  return result_t::SUCCESS;
}

static int find_empty_slot() {
  uint64_t used_flags = 0;
  for (int i = 0; i < menu.num_items; i++) {
    if (menu.items[i]->tag >= 0) {
      used_flags |= (1 << menu.items[i]->tag);
    }
  }
  for (int i = 0; i < state::MAX_SLOTS; i++) {
    if ((used_flags & (1 << i)) == 0) {
      return i;
    }
  }
  return -1;
}

static void read_input() {
  input::status_t prev_pressed = key_pressed;
  key_pressed = input::get_status(0);
  key_down.raw = key_pressed.raw & ~prev_pressed.raw;
  key_up.raw = ~key_pressed.raw & prev_pressed.raw;
}

static result_t process_input() {
  if (key_release_waiting) {
    if (key_pressed.raw != 0 || key_up.raw != 0) {
      return result_t::SUCCESS;
    } else {
      key_release_waiting = false;
    }
  }

  if (key_up.A) {
    ListItem *mi = nullptr;
    if (popup_shown) {
      mi = popup_list.get_selected_item();
    } else {
      mi = menu.get_selected_item();
    }
    if (mi) {
      on_menu_selected(mi);
    }
  } else if (key_down.select) {
    if (!popup_shown) {
      tab_t new_tab =
          static_cast<tab_t>((static_cast<int>(tab) + 1) % NUM_TABS);
      load_tab(new_tab);
    }
  } else if (popup_shown) {
    popup_list.on_key_down(key_down);
  } else {
    int prev_sel = menu.sel_index;
    menu.on_key_down(key_down);
    int new_sel = menu.sel_index;
    if (tab == tab_t::SAVE_LIST && prev_sel != new_sel) {
      load_state_screenshot();
    }
    last_menu_index[static_cast<int>(tab)] = menu.sel_index;
  }
  return result_t::SUCCESS;
}

static result_t on_menu_selected(ListItem *mi) {
  result_t res = result_t::SUCCESS;
  switch (mi->action) {
    case action_t::OPEN_DIR: res = on_open_dir(mi); break;
    case action_t::LOAD_ROM: res = on_load_rom(mi); break;
    case action_t::ADD_STATE_SLOT: res = on_add_state_slot(); break;
    case action_t::STATE_SELECT: res = on_state_select(mi); break;
    case action_t::LOAD_STATE: res = on_load_state(mi); break;
    case action_t::SAVE_STATE: res = on_save_state(mi); break;
    case action_t::DELETE_STATE: res = on_delete_state(); break;
    case action_t::FORCE_NMI_CONFIRM:
      show_confirm(icon_t::WARNING, "Force NMI?", action_t::FORCE_NMI_TRIGGER);
      break;
    case action_t::FORCE_NMI_TRIGGER: res = on_force_nmi_trigger(); break;
    case action_t::CLOSE_POPUP: popup_close(); break;
  }

  if (res != result_t::SUCCESS) {
    show_message(icon_t::ERROR, result_to_string(res));
  }

  return res;
}

static result_t on_open_dir(ListItem *mi) {
  if (strcmp(mi->label, "../") == 0) {
    // parent directory
    int sep_idx = fs::find_parent_separator(current_dir);
    if (sep_idx >= 0) {
      current_dir[sep_idx + 1] = '\0';
    }
  } else {
    // sub directory
    SHAPONES_TRY(fs::append_path(current_dir, mi->label));
  }
  SHAPONES_TRY(fs::append_separator(current_dir));
  load_file_list_tab();
  return result_t::SUCCESS;
}

static result_t on_load_rom(ListItem *mi) {
  char path[nes::MAX_PATH_LENGTH + 1];
  strncpy(path, current_dir, nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(fs::append_path(path, mi->label));
  unmap_ines();
  const uint8_t *ines = nullptr;
  size_t ines_size = 0;
  SHAPONES_TRY(load_ines(path, &ines, &ines_size));
  SHAPONES_TRY(map_ines(ines, path));
  hide();
  return result_t::SUCCESS;
}

static result_t on_add_state_slot() {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  int slot = find_empty_slot();
  if (slot < 0) return result_t::ERR_STATE_SLOT_FULL;
  state::save(path, slot);
  load_state_list_tab();
  return result_t::SUCCESS;
}

static result_t on_state_select(ListItem *mi) {
  popup_list.clear();
  popup_message[0] = '\0';
  popup_list.add_item(icon_t::PLAY, action_t::LOAD_STATE, "Load", mi->tag);
  popup_list.add_item(icon_t::SAVE, action_t::SAVE_STATE, "Save", mi->tag);
  popup_list.add_item(icon_t::NONE, action_t::CLOSE_POPUP, "Cancel");
  popup_show(icon_t::NONE, "Action?");
  return result_t::SUCCESS;
}

static result_t on_save_state(ListItem *mi) {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  SHAPONES_TRY(state::save(path, mi->tag));
  popup_close();
  load_state_list_tab();
  return result_t::SUCCESS;
}

static result_t on_load_state(ListItem *mi) {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  SHAPONES_TRY(state::load(path, mi->tag));
  popup_close();
  hide();
  return result_t::SUCCESS;
}

static result_t on_delete_state() {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  SHAPONES_TRY(nes::fs_delete(path));
  popup_close();
  load_state_list_tab();
  return result_t::SUCCESS;
}

static result_t on_force_nmi_trigger() {
  interrupt::assert_nmi();
  popup_close();
  hide();
  return result_t::SUCCESS;
}

static result_t show_message(icon_t icon, const char *msg) {
  popup_list.clear();
  popup_list.add_item(icon_t::NONE, action_t::CLOSE_POPUP, "Close");
  popup_show(icon, msg);
  return result_t::SUCCESS;
}

static result_t show_confirm(icon_t icon, const char *msg,
                             action_t confirm_action) {
  popup_list.clear();
  popup_list.add_item(icon_t::NONE, confirm_action, "OK");
  popup_list.add_item(icon_t::NONE, action_t::CLOSE_POPUP, "Cancel");
  popup_show(icon, msg);
  return result_t::SUCCESS;
}

result_t overlay(int y, uint8_t *line_buff) {
  if (y == SCREEN_HEIGHT - 1) {
    vsync = true;
  }

  if (!shown) return result_t::SUCCESS;

#if SHAPONES_MENU_LARGE_FONT
  int iy = (y / 2) / CHAR_HEIGHT;
  int sy = (y / 2) % CHAR_HEIGHT;
#else
  int iy = y / CHAR_HEIGHT;
  int sy = y % CHAR_HEIGHT;
#endif
  if (iy < 0 || iy >= BUFF_HEIGHT) {
    return result_t::SUCCESS;
  }

  for (int ix = 0; ix < BUFF_WIDTH; ix++) {
    int x = ix * CHAR_WIDTH;
    uint8_t c = text_buff[iy * BUFF_WIDTH + ix];
    uint8_t p = palette_buff[iy * BUFF_WIDTH + ix] % NUM_PALETTES;
    if (c < FONT8X16_CODE_FIRST || FONT8X16_CODE_LAST < c) {
      continue;
    }
    uint16_t word =
        FONT8X16_DATA[((int)c - FONT8X16_CODE_FIRST) * CHAR_HEIGHT + sy];
    for (int ipix = 0; ipix < CHAR_WIDTH; ipix++) {
      int color_index = word & 0x03;
      word >>= 2;
#if SHAPONES_MENU_LARGE_FONT
      int dx = (x + ipix) * 2;
      line_buff[dx + 0] = PALETTE_FILE[p * 4 + color_index];
      line_buff[dx + 1] = PALETTE_FILE[p * 4 + color_index];
#else
      line_buff[x + ipix] = PALETTE_FILE[p * 4 + color_index];
#endif
    }
  }

  if (ss_enable && !popup_shown) {
    // scren shot
    int ss_x = SCREEN_WIDTH - CHAR_WIDTH * 2 - state::SS_WIDTH - 2;
    int ss_y = SCREEN_HEIGHT - CHAR_HEIGHT * 2 - state::SS_HEIGHT - 2;
    if (ss_y <= y && y < ss_y + state::SS_HEIGHT) {
      int sy = y - ss_y;
      memcpy(&line_buff[ss_x], &ss_buff[sy * state::SS_WIDTH], state::SS_WIDTH);
    }
  }

  return result_t::SUCCESS;
}

static void popup_show(icon_t icon, const char *msg) {
  popup_icon = icon;
  strncpy(popup_message, msg, POPUP_MAX_MESSAGE_LENGTH);

  popup_shown = true;
  int msg_w = strnlen(msg, POPUP_MAX_MESSAGE_LENGTH) + 4;
  int list_w = msg_w;
  for (int i = 0; i < popup_list.num_items; i++) {
    auto *mi = popup_list.items[i];
    int item_w = strnlen(mi->label, POPUP_MAX_MESSAGE_LENGTH) + 4;
    if (item_w > list_w) {
      list_w = item_w;
    }
  }
  int list_x = (BUFF_WIDTH - list_w) / 2;

  int frame_h = popup_list.num_items + 2;
  int message_h = 0;
  if (msg_w > 0) {
    message_h = 1;
  }
  frame_h += message_h;
  int frame_y = (BUFF_HEIGHT - frame_h) / 2;
  int list_y = frame_y + 1 + message_h;
  popup_list.set_bounds(list_x, list_y, list_w, popup_list.num_items);
  request_redraw();
}

static void popup_close() {
  popup_list.clear();
  popup_shown = false;
  request_redraw();
}

static void request_redraw() { redraw_requested = true; }

static void perform_redraw() {
  fill_char(0, 0, BUFF_WIDTH, 1, ' ');
  draw_frame(CLIENT_X - 1, CLIENT_Y - 1, CLIENT_WIDTH + 2, CLIENT_HEIGHT + 2,
             '\x80');

  // tab bar
  draw_text(CLIENT_X, CLIENT_Y - 1, "\x88\x89");
  for (int itab = 0; itab < NUM_TABS; itab++) {
    for (int i = 0; i < 3; i++) {
      char c = '\xA0' + itab * 3 + i;
      if (itab == (int)tab) {
        c += 0x10;
      }
      draw_char(CLIENT_X + 2 + itab * 3 + i, CLIENT_Y - 1, c);
    }
  }

  if (tab == tab_t::NES_LIST) {
    // current directory
    fill_char(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1);
    draw_text(CLIENT_X, CLIENT_Y, current_dir, CLIENT_WIDTH);
    fill_palette(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1, 2);
  }

  // file list area
  draw_list(menu);

  // popup
  if (popup_shown) {
    int message_h = 0;
    if (popup_message[0] != '\0') {
      message_h = 1;
    }
    draw_frame(popup_list.x - 1, popup_list.y - 1 - message_h,
               popup_list.width + 2, popup_list.height + 2 + message_h, '\x90');
    if (message_h > 0) {
      fill_char(popup_list.x, popup_list.y - 1, popup_list.width, 1);
      int icon_w = 0;
      if (popup_icon != icon_t::NONE) {
        icon_w = 3;
        draw_icon(popup_list.x, popup_list.y - 1, popup_icon);
      }
      draw_text(popup_list.x + icon_w, popup_list.y - 1, popup_message,
                popup_list.width - icon_w);
      fill_palette(popup_list.x, popup_list.y - 1, popup_list.width, 1, 0);
    }
    draw_list(popup_list);
  }
}

static int draw_text(int x, int y, const char *str, int max_len) {
  int n = 0;
  char c;
  while (n < max_len && (c = str[n++]) != '\0') {
    draw_char(x++, y, c);
  }
  return n - 1;
}

static void draw_char(int x, int y, char c) {
  if (0 <= y && y < BUFF_HEIGHT && 0 <= x && x < BUFF_WIDTH) {
    text_buff[y * BUFF_WIDTH + x] = c;
  }
}

static void draw_icon(int x, int y, icon_t icon) {
  const char *text = nullptr;
  switch (icon) {
    case icon_t::PARENT: text = "\xC0\xC1"; break;
    case icon_t::FOLDER: text = "\xC2\xC3"; break;
    case icon_t::FILE: text = "\xC4\xC5"; break;
    case icon_t::ADD: text = "\xC8\xC9"; break;
    case icon_t::SAVE: text = "\xCA\xCB"; break;
    case icon_t::PLAY: text = "\xCC\xCD"; break;
  }
  if (text) draw_text(x, y, text);
}

static void draw_frame(int x, int y, int w, int h, char offset) {
  // top border
  draw_char(x, y, offset + 0);
  fill_char(x + 1, y, w - 2, 1, offset + 1);
  draw_char(x + w - 1, y, offset + 2);
  fill_palette(x, y, w, 2, 0);

  // side borders
  fill_char(x, y + 1, 1, h - 2, offset + 3);
  fill_palette(x, y + 1, 1, h - 2, 0);
  fill_char(x + w - 1, y + 1, 1, h - 2, offset + 4);
  fill_palette(x + w - 1, y + 1, 1, h - 2, 0);

  // bottom border
  draw_char(x, y + h - 1, offset + 5);
  fill_char(x + 1, y + h - 1, w - 2, 1, offset + 6);
  draw_char(x + w - 1, y + h - 1, offset + 7);
  fill_palette(x, y + h - 1, w, 1, 0);
}

static void draw_list(const ListBox &list) {
  int x = list.x;
  int y = list.y;
  int w = list.width;
  int h = list.height;
  int item_w = list.width;

  if (list.num_items > list.height) {
    // scroll bar
    item_w = list.width - 1;
    int list_right = CLIENT_X + CLIENT_WIDTH - 1;
    int button_pos = 0;
    if (list.num_items > h) {
      int n = (list.num_items - h);
      button_pos = (list.scroll_pos * (h - 3) + (n - 1)) / n;
    }
    fill_palette(list_right, y, 1, h, 0);
    draw_char(list_right, y, '\x8C');          // up arrow
    draw_char(list_right, y + h - 1, '\x8D');  // down arrow
    fill_char(list_right, y + 1, 1, h - 2, '\x8E');
    draw_char(list_right, y + 1 + button_pos, '\x8F');  // scroll bar
  }

  for (int iy = 0; iy < h; iy++) {
    int i = iy + list.scroll_pos;
    fill_char(x, y + iy, item_w, 1, ' ');
    if (0 <= i && i < list.num_items) {
      auto *mi = list.items[i];
      if (mi->icon != icon_t::NONE) {
        draw_icon(x, y + iy, mi->icon);
      }
      draw_text(x + 3, y + iy, mi->label, item_w - 3);
    }
    if (i == list.sel_index) {
      fill_palette(x, y + iy, item_w, 1, 1);  // highlight color
    } else {
      fill_palette(x, y + iy, item_w, 1, 0);  // normal color
    }
  }
}

static void fill_char(int x, int y, int w, int h, char c) {
  clip_rect(&x, &y, &w, &h);
  for (int iy = 0; iy < h; iy++) {
    for (int ix = 0; ix < w; ix++) {
      text_buff[(y + iy) * BUFF_WIDTH + (x + ix)] = c;
    }
  }
}

static void fill_palette(int x, int y, int w, int h, uint8_t p) {
  clip_rect(&x, &y, &w, &h);
  for (int iy = 0; iy < h; iy++) {
    int offset = (y + iy) * BUFF_WIDTH + x;
    for (int ix = 0; ix < w; ix++) {
      palette_buff[offset + ix] = p;
    }
  }
}

static void clip_rect(int *x, int *y, int *w, int *h) {
  if (*x < 0) {
    *w += *x;
    *x = 0;
  }
  if (*x + *w > BUFF_WIDTH) {
    *w = BUFF_WIDTH - *x;
  }
  if (*y < 0) {
    *h += *y;
    *y = 0;
  }
  if (*y + *h > BUFF_HEIGHT) {
    *h = BUFF_HEIGHT - *y;
  }
}

}  // namespace nes::menu
// #include "shapones/ppu.hpp"

// #include "shapones/cpu.hpp"

// #include "shapones/fifo.hpp"

// #include "shapones/host_intf.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"

// #include "shapones/menu.hpp"

// #include "shapones/state.hpp"


namespace nes::ppu {

static constexpr uint32_t STATE_HEADER_SIZE = registers_t::STATE_SIZE + 32;

static registers_t reg;

volatile cycle_t cycle_count;

static uint16_t focus_x;
static uint16_t focus_y;

static volatile uint16_t scroll_counter;
static uint8_t fine_x_counter;

static uint8_t bus_read_data_latest = 0;
static uint8_t bus_read_data_delayed = 0;

static bool scroll_ppuaddr_high_stored = false;

static bool nmi_level = false;

static uint8_t palette_file[PALETTE_NUM_BANK * PALETTE_SIZE];

static uint8_t oam[OAM_SIZE];
static sprite_line_t sprite_lines[MAX_VISIBLE_SPRITES];
static int num_visible_sprites;

static AsyncFifo<reg_write_t, 6> write_queue;
static volatile bool ppu_status_read = false;

static void flush_write_queue();

static uint8_t bus_read(addr_t addr);
static void bus_write(addr_t addr, uint8_t data);
static uint8_t oam_read(addr_t addr);
static void oam_write(addr_t addr, uint8_t data);
static uint8_t palette_read(addr_t addr);
static void palette_write(addr_t addr, uint8_t data);

static void render_bg(uint8_t *line_buff, int x0, int x1, bool skip_render);
static void enum_visible_sprites(bool skip_render);
static void render_sprite(uint8_t *line_buff, int x0, int x1, bool skip_render);

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  // https://www.nesdev.org/wiki/PPU_power_up_state
  memset(&reg, 0, sizeof(registers_t));
  memset(oam, 0, sizeof(oam));

  bus_read_data_latest = 0;
  bus_read_data_delayed = 0;

  focus_x = 0;
  focus_y = 0;

  num_visible_sprites = 0;

  scroll_ppuaddr_high_stored = false;

  nmi_level = false;

  cycle_count = 0;

  write_queue.clear();

  return result_t::SUCCESS;
}

bool is_in_hblank() { return focus_x >= SCREEN_WIDTH; }

int current_focus_y() { return focus_y; }

uint8_t reg_read(addr_t addr) {
  if (!write_queue.is_empty()) {
    SemaphoreBlock block(SEMAPHORE_PPU);
    flush_write_queue();
  }

  uint8_t retval;
  switch (addr) {
    case REG_PPUSTATUS: {
      retval = reg.status.raw;
      scroll_ppuaddr_high_stored = false;
      ppu_status_read = true;
    } break;

    case REG_OAMDATA: retval = oam_read(reg.oam_addr); break;

    case REG_PPUDATA: {
      addr_t addr = scroll_counter & SCROLL_MASK_PPU_ADDR;
      bus_read(addr);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scroll_counter &= ~SCROLL_MASK_PPU_ADDR;
      scroll_counter |= addr;
      retval = bus_read_data_delayed;
    } break;

    default: retval = 0; break;
  }
  return retval;
}

void reg_write(addr_t addr, uint8_t data) {
  reg_write_t req;
  req.addr = addr;
  req.data = data;
  if (!write_queue.try_push(req)) {
    SemaphoreBlock block(SEMAPHORE_PPU);
    flush_write_queue();
    write_queue.push_blocking(req);
  }
}

static void flush_write_queue() {
  reg_write_t req;

  if (ppu_status_read) {
    ppu_status_read = false;
    reg.status.vblank_flag = 0;
  }

  while (write_queue.try_peek(&req)) {
    switch (req.addr) {
      case REG_PPUCTRL:
        // name sel bits
        reg.scroll &= 0xf3ff;
        reg.scroll |= (uint16_t)(req.data & 0x3) << 10;
        // other bits
        reg.control.raw = req.data;
        break;

      case REG_PPUMASK: reg.mask.raw = req.data; break;

      case REG_OAMADDR: reg.oam_addr = req.data; break;

      case REG_OAMDATA: oam_write(reg.oam_addr, req.data); break;

      case REG_PPUSCROLL:
        if (!scroll_ppuaddr_high_stored) {
          reg.scroll &= ~SCROLL_MASK_COARSE_X;
          reg.scroll |= (req.data >> 3) & SCROLL_MASK_COARSE_X;
          reg.fine_x = req.data & 0x7;
          scroll_ppuaddr_high_stored = true;
        } else {
          reg.scroll &= ~(SCROLL_MASK_COARSE_Y | SCROLL_MASK_FINE_Y);
          reg.scroll |= ((uint16_t)req.data << 2) & SCROLL_MASK_COARSE_Y;
          reg.scroll |= ((uint16_t)req.data << 12) & SCROLL_MASK_FINE_Y;
          scroll_ppuaddr_high_stored = false;
        }
        break;

      case REG_PPUADDR:
        if (!scroll_ppuaddr_high_stored) {
          reg.scroll &= 0x00ffu;
          reg.scroll |= ((uint16_t)req.data << 8) & 0x3f00u;
          scroll_ppuaddr_high_stored = true;
        } else {
          reg.scroll &= 0xff00u;
          reg.scroll |= (uint16_t)req.data;
          scroll_counter = reg.scroll;
          scroll_ppuaddr_high_stored = false;
        }
        break;

      case REG_PPUDATA: {
        uint_fast16_t scr = scroll_counter;
        addr_t addr = scr & SCROLL_MASK_PPU_ADDR;
        bus_write(addr, req.data);
        addr += reg.control.incr_stride ? 32 : 1;
        addr &= SCROLL_MASK_PPU_ADDR;
        scr &= ~SCROLL_MASK_PPU_ADDR;
        scr |= addr;
        scroll_counter = scr;
      } break;
    }

    write_queue.try_pop(&req);
  }
}

void oam_dma_write(addr_t offset, uint8_t data) {
  oam_write((reg.oam_addr + offset) % OAM_SIZE, data);
}

static SHAPONES_INLINE uint8_t bus_read(addr_t addr) {
  if (memory::CHRROM_BASE <= addr &&
      addr < memory::CHRROM_BASE + CHRROM_RANGE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = memory::chrrom_read(addr - memory::CHRROM_BASE);
  } else if (VRAM_BASE <= addr && addr < VRAM_BASE + VRAM_SIZE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = memory::vram_read(addr - VRAM_BASE);
  } else if (PALETTE_FILE_BASE <= addr &&
             addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
    bus_read_data_delayed = palette_read(addr - PALETTE_FILE_BASE);
    bus_read_data_latest = bus_read_data_delayed;
  } else if (VRAM_MIRROR_BASE <= addr &&
             addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = memory::vram_read(addr - VRAM_MIRROR_BASE);
  } else {
    bus_read_data_delayed = 0;
    bus_read_data_latest = 0;
  }
  return bus_read_data_latest;
}

static SHAPONES_INLINE void bus_write(addr_t addr, uint8_t data) {
  if (memory::CHRROM_BASE <= addr &&
      addr < memory::CHRROM_BASE + CHRROM_RANGE) {
    memory::chrram_write(addr - memory::CHRROM_BASE, data);
  } else if (VRAM_BASE <= addr && addr < VRAM_BASE + VRAM_SIZE) {
    memory::vram_write(addr - VRAM_BASE, data);
  } else if (PALETTE_FILE_BASE <= addr &&
             addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
    palette_write(addr - PALETTE_FILE_BASE, data);
  } else if (VRAM_MIRROR_BASE <= addr &&
             addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
    memory::vram_write(addr - VRAM_MIRROR_BASE, data);
  }
}

static SHAPONES_INLINE uint8_t oam_read(addr_t addr) {
  return oam[addr % OAM_SIZE];
}

static SHAPONES_INLINE void oam_write(addr_t addr, uint8_t data) {
  oam[addr % OAM_SIZE] = data;
}

static SHAPONES_INLINE uint8_t palette_read(addr_t addr) {
  addr %= PALETTE_FILE_SIZE;
  switch (addr) {
    case 0x10: return palette_file[0 * PALETTE_SIZE];
    case 0x14: return palette_file[1 * PALETTE_SIZE];
    case 0x18: return palette_file[2 * PALETTE_SIZE];
    case 0x1c: return palette_file[3 * PALETTE_SIZE];
    default: return palette_file[addr];
  }
}

static SHAPONES_INLINE void palette_write(addr_t addr, uint8_t data) {
  addr %= PALETTE_FILE_SIZE;
  data &= 0x3f;
  switch (addr) {
    case 0x10: palette_file[0 * PALETTE_SIZE] = data; break;
    case 0x14: palette_file[1 * PALETTE_SIZE] = data; break;
    case 0x18: palette_file[2 * PALETTE_SIZE] = data; break;
    case 0x1c: palette_file[3 * PALETTE_SIZE] = data; break;
    default: palette_file[addr] = data; return;
  }
}

result_t service(uint8_t *line_buff, bool skip_render, status_t *status) {
  if (!semaphore_try_take(SEMAPHORE_PPU)) {
    status->timing = timing_t::NONE;
    status->focus_y = focus_y;
    return result_t::SUCCESS;
  }

  timing_t timing = timing_t::NONE;
  bool irq = false;

  while (true) {
    flush_write_queue();

    cycle_t cycle_diff = cpu::ppu_cycle_leading() - cycle_count;
    if (cycle_diff <= 0) {
      break;
    }

    // events
    if (focus_x == 0) {
      if (focus_y == SCREEN_HEIGHT + 1) {
        // vblank flag/interrupt
        reg.status.vblank_flag = 1;
      } else if (focus_y == SCAN_LINES - 1) {
        // clear flags
        reg.status.vblank_flag = 0;
        reg.status.sprite0_hit = 0;
      }
    }

    bool nmi_level_new =
        reg.status.vblank_flag && reg.control.vblank_nmi_enable;
    if (nmi_level_new && !nmi_level) {
      interrupt::assert_nmi();
      irq = true;
    }
    nmi_level = nmi_level_new;

    // determine step count
    uint_fast16_t step_count;
    uint_fast16_t dist_to_end;
    if (focus_y < SCREEN_HEIGHT && focus_x < SCREEN_WIDTH) {
      // visible area
      dist_to_end = SCREEN_WIDTH - focus_x;
    } else {
      // blank_area
      dist_to_end = LINE_CYCLES - focus_x;
    }
    step_count = dist_to_end < cycle_diff ? dist_to_end : cycle_diff;
    step_count =
        step_count < MAX_DELAY_CYCLES / 2 ? step_count : MAX_DELAY_CYCLES / 2;

    uint_fast16_t next_focus_x = focus_x + step_count;

    if (focus_x == 0 && focus_y < SCREEN_HEIGHT && reg.mask.sprite_enable) {
      // enumerate visible sprites in current line
      enum_visible_sprites(skip_render);
    }

    // render background
    render_bg(line_buff, focus_x, next_focus_x, skip_render);

    if (focus_x < SCREEN_WIDTH && focus_y < SCREEN_HEIGHT &&
        reg.mask.sprite_enable) {
      // render sprite
      render_sprite(line_buff, focus_x, next_focus_x, skip_render);
    }

    if (focus_y < SCREEN_HEIGHT) {
      if (focus_x <= SCREEN_WIDTH && SCREEN_WIDTH < next_focus_x) {
        mapper::instance->hblank(reg, focus_y);
      }
    } else {
      if (focus_x == 0) {
        mapper::instance->vblank(reg);
      }
    }

    // step focus
    focus_x = next_focus_x;
    if (focus_x >= LINE_CYCLES) {
      focus_x -= LINE_CYCLES;
      focus_y++;
      if (focus_y >= SCAN_LINES) {
        focus_y = 0;
        timing |= timing_t::END_OF_FRAME;
      }
    }

    // step cycle counter
    cycle_count += step_count;

    if (focus_y < SCREEN_HEIGHT) {
      if (focus_x == SCREEN_WIDTH) {
        timing |= timing_t::END_OF_VISIBLE_LINE;
        if (focus_y == SCREEN_HEIGHT - 1) {
          timing |= timing_t::END_OF_VISIBLE_AREA;
        }
      }
    } else {
      if (focus_x == 0) {
        timing |= timing_t::START_OF_VBLANK_LINE;
      }
    }

    if (timing != timing_t::NONE || irq) {
      break;
    }
  }

  if (!!(timing & timing_t::END_OF_VISIBLE_LINE)) {
    nes::state::hsync(focus_y, line_buff, skip_render);
    if (!skip_render) {
      nes::menu::overlay(focus_y, line_buff);
    }
  }

  if (status) {
    status->focus_y = focus_y;
    status->timing = timing;
  }

  semaphore_give(SEMAPHORE_PPU);

  return result_t::SUCCESS;
}

static void render_bg(uint8_t *line_buff, int x0_block, int x1_block,
                      bool skip_render) {
  bool visible_area = (x0_block < SCREEN_WIDTH && focus_y < SCREEN_HEIGHT);
  bool bg_enabled = reg.mask.bg_enable;

  if (skip_render && num_visible_sprites == 0) {
    // If skip_render is set, only render lines where sprite #0 exists.
    visible_area = false;
  }

  uint32_t bg_offset = (uint32_t)reg.control.bg_name_sel << 12;

  while (x0_block < x1_block) {
    int x0, x1;
    {
      uint_fast16_t scr = scroll_counter;

      // determine step count
      x0 = x0_block;
      if (visible_area && bg_enabled) {
        x1 = x0 + (BLOCK_SIZE - ((scr & 0x1) * TILE_SIZE + fine_x_counter));
      } else {
        x1 = x0 + 64;
      }
      x1 = x1 < x1_block ? x1 : x1_block;

      if (visible_area) {
        if (bg_enabled) {
          // read name table for two tiles
          addr_t name_addr0 = scr & 0xffeu;
          addr_t name_addr1 = name_addr0 + 1;
          uint32_t name0 = memory::vram_read(name_addr0);
          uint32_t name1 = memory::vram_read(name_addr1);

          uint32_t fine_y = (scr & SCROLL_MASK_FINE_Y) >> 12;

          uint32_t chr = 0xFFFFFFFF;
          const uint8_t *palette = nullptr;
          if (!skip_render) {
            // read CHRROM
            uint32_t chrrom_index0 = (name0 << 4) + fine_y;
            uint32_t chrrom_index1 = (name1 << 4) + fine_y;
            chrrom_index0 += bg_offset;
            chrrom_index1 += bg_offset;
            uint_fast16_t chr0 =
                memory::chrrom_read_double(chrrom_index0, false);
            uint_fast16_t chr1 =
                memory::chrrom_read_double(chrrom_index1, false);
            chr = ((uint32_t)chr1 << 16) | (uint32_t)chr0;

            // adjust CHR bit pos
            int chr_shift_size = ((scr << 4) & 0x10) | (fine_x_counter << 1);
            chr >>= chr_shift_size;

            // calc attr index
            addr_t attr_index = (scr & SCROLL_MASK_NAME_SEL) | 0x3c0 |
                                ((scr >> 2) & 0x7) | ((scr >> 4) & 0x38);
            int attr_shift_size = ((scr >> 4) & 0x4) | (scr & 0x2);

            // read attr table
            uint8_t attr = memory::vram_read(attr_index);
            attr = (attr >> attr_shift_size) & 0x3;
            palette = palette_file + attr * PALETTE_SIZE;
          }

          // render BG block
          uint8_t bg_color = palette_file[0];
          for (int x = x0; x < x1; x++) {
            uint32_t palette_index = chr & 0x3;
            chr >>= 2;
            if (palette_index == 0) {
              line_buff[x] = bg_color;
            } else {
              uint8_t col = OPAQUE_FLAG;
              if (!skip_render) {
                col |= palette[palette_index];
              }
              line_buff[x] = col;
            }
          }
        } else {
          // blank background
          uint8_t bg_color = palette_file[0];
          memset(&line_buff[x0], bg_color, x1 - x0);
        }
      }
    }

    // update scroll counter
    // see: https://www.nesdev.org/wiki/PPU_scrolling
    if (reg.mask.bg_enable || reg.mask.sprite_enable) {
      uint_fast16_t scr = scroll_counter;
      uint_fast8_t fx = fine_x_counter;
      if (focus_y < SCREEN_HEIGHT) {
        if (x0 < SCREEN_WIDTH) {
          // step scroll counter for x-axis
          fx += (x1 - x0);
          while (fx >= TILE_SIZE) {
            fx -= TILE_SIZE;
            // if coarse_x < 31
            if ((scr & SCROLL_MASK_COARSE_X) < SCROLL_MASK_COARSE_X) {
              scr++;  // coarse_x++
            } else {
              // right edge of name table
              scr &= ~SCROLL_MASK_COARSE_X;  // coarse_x = 0
              scr ^= 0x0400u;                // switch name table horizontally
            }
          }
        } else if (SCREEN_WIDTH < x1 && x0 <= SCREEN_WIDTH) {
          // step scroll counter for y-axis
          // if fine_y < 7
          if ((scr & SCROLL_MASK_FINE_Y) < SCROLL_MASK_FINE_Y) {
            scr += 0x1000u;  // fine_y++
          } else {
            // bottom edge of tile
            scr &= ~SCROLL_MASK_FINE_Y;  // fine_y = 0
            // if coarse_y == 29
            if ((scr & SCROLL_MASK_COARSE_Y) == ((NUM_TILE_Y - 1) << 5)) {
              // bottom edge of name table
              scr &= ~SCROLL_MASK_COARSE_Y;  // coarse_y = 0
              scr ^= 0x0800u;                // switch name table vertically
            }
            // else if coarse_y == 31
            else if ((scr & SCROLL_MASK_COARSE_Y) == SCROLL_MASK_COARSE_Y) {
              scr &= ~SCROLL_MASK_COARSE_Y;  // coarse_y = 0
            } else {
              scr += NUM_TILE_X;  // coarse_y++
            }
          }

          // horizontal recovery
          constexpr uint16_t MASK = 0x041fu;
          scr &= ~MASK;
          scr |= reg.scroll & MASK;
          fx = reg.fine_x;
        }
        fine_x_counter = fx;
      } else if (focus_y == SCAN_LINES - 1) {
        if (280 <= x1 && x0 <= 304) {
          // vertical recovery
          constexpr uint16_t copy_mask = 0x7be0u;
          scr &= ~copy_mask;
          scr |= reg.scroll & copy_mask;
        }
      }
      scroll_counter = scr;
    }  // if

    x0_block = x1;
  }  // while
}

static void enum_visible_sprites(bool skip_render) {
  int n = MAX_SPRITE_COUNT;
  if (skip_render) {
    // When skip_render is set, only sprite #0 is processed for IRQ.
    n = 1;
  }

  num_visible_sprites = 0;
  uint8_t h = reg.control.sprite_size ? 16 : 8;
  for (int i = 0; i < n; i++) {
    uint_fast8_t s_y = oam[i * 4 + OAM_ENTRY_OFFSET_Y];
    uint_fast8_t s_tile = oam[i * 4 + OAM_ENTRY_OFFSET_TILE];
    uint_fast8_t s_attr = oam[i * 4 + OAM_ENTRY_OFFSET_ATTR];
    uint_fast8_t s_x = oam[i * 4 + OAM_ENTRY_OFFSET_X];

    const auto &s = oam[i];

    // vertical hit test
    int src_y = focus_y - (s_y + SPRITE_Y_OFFSET);
    if (0 <= src_y && src_y < h) {
      int tile_index = 0;
      uint_fast16_t chr = 0xFFFF;

      if (!skip_render) {
        if (s_attr & OAM_ATTR_INVERT_V) {
          // vertical inversion
          if (reg.control.sprite_size) {
            src_y ^= 0xf;
          } else {
            src_y ^= 0x7;
          }
        }

        // tile index calculation
        if (reg.control.sprite_size) {
          // 8x16 sprite
          if (src_y < 8) {
            tile_index = s_tile & 0xfe;
          } else {
            tile_index = s_tile | 0x01;
          }

          if (s_tile & 0x1) {
            tile_index += 0x1000 / 16;
          }
        } else {
          // 8x8 sprite
          tile_index = s_tile;
          if (reg.control.sprite_name_sel) {
            tile_index += 0x1000 / 16;
          }
        }
        // read CHRROM
        int chrrom_index = (tile_index << 4) + (src_y & 0x7);
        chr = memory::chrrom_read_double(chrrom_index,
                                         s_attr & OAM_ATTR_INVERT_H);
      }

      // store sprite information
      sprite_line_t &sl = sprite_lines[num_visible_sprites++];
      sl.chr = chr;
      sl.x = s_x;
      sl.palette_offset = (4 + (s_attr & OAM_ATTR_PALETTE)) * PALETTE_SIZE;
      sl.attr = 0;
      if (s_attr & OAM_ATTR_PRIORITY) sl.attr |= SL_ATTR_BEHIND;
      if (i == 0) sl.attr |= SL_ATTR_ZERO;

      if (num_visible_sprites >= MAX_VISIBLE_SPRITES) {
        break;
      }
    }
  }
}

static void render_sprite(uint8_t *line_buff, int x0_block, int x1_block,
                          bool skip_render) {
  for (int i = 0; i < num_visible_sprites; i++) {
    const auto &sl = sprite_lines[i];
    int x0 = (x0_block > sl.x) ? x0_block : sl.x;
    int x1 = (x1_block < sl.x + TILE_SIZE) ? x1_block : (sl.x + TILE_SIZE);
    uint_fast16_t chr = sl.chr >> (2 * (x0 - sl.x));
    uint_fast8_t attr = sl.attr;
    uint8_t *palette = palette_file + sl.palette_offset;
    for (int x = x0; x < x1; x++) {
      uint_fast16_t palette_index = chr & 0x3;
      chr >>= 2;
      bool sprite_opaque = (palette_index != 0);
      uint_fast8_t col = line_buff[x];
      bool bg_opaque = (col & OPAQUE_FLAG) != 0;
      if (!skip_render && !(col & BEHIND_FLAG)) {
        bool sprite_front = !bg_opaque || !(attr & SL_ATTR_BEHIND);
        if (sprite_opaque) {
          if (sprite_front) {
            col = palette[palette_index];
          }
          col |= BEHIND_FLAG;
          line_buff[x] = col;
        }
      }
      if ((attr & SL_ATTR_ZERO) && sprite_opaque && bg_opaque) {
        reg.status.sprite0_hit = 1;
      }
    }
  }
}

uint32_t get_state_size() {
  return STATE_HEADER_SIZE + sizeof(palette_file) + sizeof(oam);
}

result_t save_state(void *file_handle) {
  flush_write_queue();

  uint8_t buff[STATE_HEADER_SIZE];
  memset(buff, 0, sizeof(buff));
  uint8_t *p = buff;
  reg.store(p);
  p += registers_t::STATE_SIZE;
  BufferWriter writer(p);
  writer.u64(cycle_count);
  writer.u16(focus_x);
  writer.u16(focus_y);
  writer.u16(scroll_counter);
  writer.u8(fine_x_counter);
  writer.u8(bus_read_data_latest);
  writer.u8(bus_read_data_delayed);
  writer.b(scroll_ppuaddr_high_stored);
  writer.b(nmi_level);
  SHAPONES_TRY(fs_write(file_handle, buff, sizeof(buff)));
  SHAPONES_TRY(fs_write(file_handle, palette_file, sizeof(palette_file)));
  SHAPONES_TRY(fs_write(file_handle, oam, sizeof(oam)));
  return result_t::SUCCESS;
}

result_t load_state(void *file_handle) {
  write_queue.clear();
  SHAPONES_THREAD_FENCE_SEQ_CST();

  uint8_t buff[STATE_HEADER_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buff, sizeof(buff)));
  const uint8_t *p = buff;
  reg.load(p);
  p += registers_t::STATE_SIZE;
  BufferReader reader(p);
  cycle_count = reader.u64();
  focus_x = reader.u16();
  focus_y = reader.u16();
  scroll_counter = reader.u16();
  fine_x_counter = reader.u8();
  bus_read_data_latest = reader.u8();
  bus_read_data_delayed = reader.u8();
  scroll_ppuaddr_high_stored = reader.b();
  nmi_level = reader.b();
  SHAPONES_TRY(fs_read(file_handle, palette_file, sizeof(palette_file)));
  SHAPONES_TRY(fs_read(file_handle, oam, sizeof(oam)));
  return result_t::SUCCESS;
}

}  // namespace nes::ppu
// #include "shapones/state.hpp"

// #include "shapones/apu.hpp"

// #include "shapones/cpu.hpp"

// #include "shapones/fs.hpp"

// #include "shapones/host_intf.hpp"

// #include "shapones/input.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"

// #include "shapones/menu.hpp"

// #include "shapones/ppu.hpp"


namespace nes::state {

static constexpr int SS_BUFF_DEPTH = 2;
static constexpr int SS_SCALING = nes::SCREEN_HEIGHT / state::SS_HEIGHT;
static constexpr int SS_CLIP_LEFT =
    (nes::SCREEN_WIDTH - state::SS_WIDTH * SS_SCALING) / 2;
static constexpr int SS_CLIP_TOP =
    (nes::SCREEN_HEIGHT - state::SS_HEIGHT * SS_SCALING) / 2;

uint8_t ss_buff[SS_SIZE_BYTES * SS_BUFF_DEPTH];
volatile int ss_wr_index = 0;
volatile int ss_num_stored = 0;
int ss_capture_counter = 0;

uint32_t frame_count = 0;

static uint32_t get_slot_offset(int slot, uint32_t slot_size) {
  return state_file_header_t::SIZE + state_slot_entry_t::SIZE * MAX_SLOTS +
         slot * slot_size;
}
static result_t write_screenshot(void *file_handle);
static result_t write_slot_data(void *file_handle);
static result_t read_slot_data(void *file_handle);

void reset() {
  ss_wr_index = 0;
  ss_num_stored = 0;
  ss_capture_counter = 3 * 60;  // wait 3 seconds before first shot
  memset(ss_buff, 0, sizeof(ss_buff));
}

void hsync(int focus_y, const uint8_t *line_buff, bool skip_render) {
  if (focus_y == SCREEN_HEIGHT - 1) {
    frame_count++;
  }

  if (menu::is_shown()) return;
  if (focus_y == SCREEN_HEIGHT - 1) {
    ss_capture_counter--;
  }
  if (ss_capture_counter > 0 || skip_render) {
    return;
  }

  int sy = focus_y - SS_CLIP_TOP;
  int dy = sy / SS_SCALING;
  if (0 <= dy && dy < SS_HEIGHT && sy % SS_SCALING == 0) {
    int wr_index = ss_wr_index;
    uint8_t *dst = &ss_buff[(wr_index * SS_SIZE_BYTES) + (dy * SS_WIDTH)];
    for (int dx = 0; dx < SS_WIDTH; dx++) {
      int sx = SS_CLIP_LEFT + dx * SS_SCALING;
      uint8_t c01 =
          blend_colors(line_buff[sx] & 0x3F, line_buff[sx + 1] & 0x3F);
      uint8_t c23 =
          blend_colors(line_buff[sx + 2] & 0x3F, line_buff[sx + 3] & 0x3F);
      dst[dx] = blend_colors(c01, c23);
    }
    if (dy == SS_HEIGHT - 1) {
      ss_wr_index = (wr_index + 1) % SS_BUFF_DEPTH;
      ss_num_stored++;
      if (ss_num_stored > SS_BUFF_DEPTH) {
        ss_num_stored = SS_BUFF_DEPTH;
      }
      ss_capture_counter = 3 * 60;  // wait 3 seconds before next shot
    }
  }
}

result_t get_state_path(char *out_path, size_t max_len) {
  const char *ines_path = get_ines_path();
  if (ines_path[0] == '\0') {
    return result_t::ERR_INES_NOT_LOADED;
  }
  strncpy(out_path, ines_path, max_len);
  SHAPONES_TRY(fs::replace_ext(out_path, STATE_FILE_EXT));
  return result_t::SUCCESS;
}

uint32_t get_slot_size() {
  uint32_t size = 0;
  size += SS_SIZE_BYTES;
  size += cpu::get_state_size();
  size += ppu::get_state_size();
  size += apu::get_state_size();
  size += interrupt::get_state_size();
  size += memory::get_state_size();
  size += mapper::instance->get_state_size();
  size += input::state_size();
  return size;
}

result_t save(const char *path, int slot) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, true, &f));

  size_t file_size = 0;
  SHAPONES_TRY(fs_size(f, &file_size));
  bool create = file_size < sizeof(state_file_header_t) +
                                sizeof(state_slot_entry_t) * MAX_SLOTS;

  do {
    if (create) {
      // write file header
      {
        uint8_t buff[state_file_header_t::SIZE];
        memset(buff, 0, sizeof(buff));
        state_file_header_t fh;
        fh.marker = MARKER;
        fh.version = VERSION;
        fh.slot_size = slot_size;
        fh.store(buff);

        res = fs_seek(f, 0);
        if (res != result_t::SUCCESS) break;

        res = fs_write(f, buff, sizeof(buff));
        if (res != result_t::SUCCESS) break;
      }

      // initialize index
      for (int i = 0; i < MAX_SLOTS; i++) {
        uint8_t buff[state_slot_entry_t::SIZE];
        memset(buff, 0, sizeof(buff));
        res = fs_write(f, buff, sizeof(buff));
        if (res != result_t::SUCCESS) break;
      }
    }

    // write slot data
    {
      uint32_t offset = get_slot_offset(slot, slot_size);
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = write_screenshot(f);
      if (res != result_t::SUCCESS) break;

      res = write_slot_data(f);
      if (res != result_t::SUCCESS) break;
    }

    // update index
    {
      uint8_t buff[state_slot_entry_t::SIZE];
      memset(buff, 0, sizeof(buff));
      state_slot_entry_t slot_header;
      slot_header.flags = SLOT_FLAG_USED;
      slot_header.frame_count = frame_count;
      strncpy(slot_header.name, "No Name", state_slot_entry_t::NAME_LENGTH);
      slot_header.store(buff);

      int offset = state_file_header_t::SIZE + state_slot_entry_t::SIZE * slot;

      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = fs_write(f, buff, sizeof(buff));
      if (res != result_t::SUCCESS) break;
    }

    // seek to end
    fs_seek(f, file_size);

  } while (0);

  fs_close(f);
  return res;
}

result_t load(const char *path, int slot) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    // header check
    {
      uint8_t buff[state_file_header_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_file_header_t fh;
      fh.load(buff);
      if (fh.marker != MARKER) {
        res = result_t::ERR_STATE_INVALID_FORMAT;
        break;
      }
      if (fh.slot_size != slot_size) {
        res = result_t::ERR_STATE_SIZE_MISMATCH;
        break;
      }
    }

    // read slot entry
    {
      SHAPONES_TRY(fs_seek(
          f, state_file_header_t::SIZE + state_slot_entry_t::SIZE * slot));
      uint8_t buff[state_slot_entry_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_slot_entry_t slot_entry;
      slot_entry.index = slot;
      slot_entry.load(buff);
      if (!slot_entry.is_used()) {
        res = result_t::ERR_STATE_NO_SLOT_DATA;
        break;
      }
      frame_count = slot_entry.frame_count;
    }

    // read slot data
    {
      uint32_t offset = get_slot_offset(slot, slot_size);
      res = fs_seek(f, offset);
      if (res != result_t::SUCCESS) break;

      res = fs_read(f, ss_buff, SS_SIZE_BYTES);
      if (res != result_t::SUCCESS) break;
      ss_wr_index = 1;
      ss_num_stored = 1;
      ss_capture_counter = 0;

      res = read_slot_data(f);
      if (res != result_t::SUCCESS) break;
    }
  } while (0);

  fs_close(f);

  return res;
}

result_t read_screenshot(const char *path, int slot, uint8_t *out_buff) {
  result_t res = result_t::SUCCESS;

  uint32_t slot_size = get_slot_size();

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    uint32_t offset = get_slot_offset(slot, slot_size);
    res = fs_seek(f, offset);
    if (res != result_t::SUCCESS) break;

    res = fs_read(f, out_buff, SS_SIZE_BYTES);
    if (res != result_t::SUCCESS) break;
  } while (0);

  fs_close(f);

  return res;
}

static result_t write_screenshot(void *f) {
  SemaphoreBlock lock(SEMAPHORE_PPU);
  int rd_index = (ss_wr_index + SS_BUFF_DEPTH - ss_num_stored) % SS_BUFF_DEPTH;
  result_t res = fs_write(f, &ss_buff[rd_index * SS_SIZE_BYTES], SS_SIZE_BYTES);
  return res;
}

static result_t write_slot_data(void *f) {
  SemaphoreBlock ppu_block(SEMAPHORE_PPU);
  SemaphoreBlock apu_block(SEMAPHORE_APU);
  SHAPONES_TRY(cpu::save_state(f));
  SHAPONES_TRY(ppu::save_state(f));
  SHAPONES_TRY(apu::save_state(f));
  SHAPONES_TRY(input::save_state(f));
  SHAPONES_TRY(interrupt::save_state(f));
  SHAPONES_TRY(memory::save_state(f));
  SHAPONES_TRY(mapper::instance->save_state(f));
  return result_t::SUCCESS;
}

static result_t read_slot_data(void *f) {
  SemaphoreBlock ppu_block(SEMAPHORE_PPU);
  SemaphoreBlock apu_block(SEMAPHORE_APU);
  SHAPONES_TRY(cpu::load_state(f));
  SHAPONES_TRY(ppu::load_state(f));
  SHAPONES_TRY(apu::load_state(f));
  SHAPONES_TRY(input::load_state(f));
  SHAPONES_TRY(interrupt::load_state(f));
  SHAPONES_TRY(memory::load_state(f));
  SHAPONES_TRY(mapper::instance->load_state(f));
  return result_t::SUCCESS;
}

result_t enum_slots(const char *path, enum_slot_cb_t callback) {
  result_t res = result_t::SUCCESS;

  void *f;
  SHAPONES_TRY(fs_open(path, false, &f));

  do {
    // header check
    {
      uint8_t buff[state_file_header_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_file_header_t fh;
      fh.load(buff);
      if (fh.marker != MARKER) {
        res = result_t::ERR_STATE_INVALID_FORMAT;
        break;
      }
    }

    // read slot entries
    for (int i = 0; i < MAX_SLOTS; i++) {
      uint8_t buff[state_slot_entry_t::SIZE];
      SHAPONES_TRY(fs_read(f, buff, sizeof(buff)));
      state_slot_entry_t slot_entry;
      slot_entry.index = i;
      slot_entry.load(buff);
      if (!callback(slot_entry)) {
        break;
      }
    }
  } while (0);
  fs_close(f);
  return res;
}

}  // namespace nes::state
// #include "shapones/shapones.hpp"

// #include "shapones/state.hpp"


namespace nes {

char ines_path[MAX_PATH_LENGTH + 1] = "";
bool ines_mapped = false;

static void build_blend_table();

config_t get_default_config() {
  config_t cfg;
  cfg.apu_sampling_rate = 44100;
  return cfg;
}

const char *get_ines_path() { return ines_path; }

result_t init(const config_t &cfg) {
  build_blend_table();

  for (int i = 0; i < NUM_SPINLOCKS; i++) {
    SHAPONES_TRY(nes::spinlock_init(i));
  }
  for (int i = 0; i < NUM_SEMAPHORES; i++) {
    SHAPONES_TRY(nes::semaphore_init(i));
  }
  SHAPONES_TRY(interrupt::init());
  SHAPONES_TRY(memory::init());
  SHAPONES_TRY(mapper::init());
  SHAPONES_TRY(cpu::init());
  SHAPONES_TRY(ppu::init());
  SHAPONES_TRY(apu::init());
  SHAPONES_TRY(menu::init());
  SHAPONES_TRY(input::init());
  apu::set_sampling_rate(cfg.apu_sampling_rate);
  cpu::stop();
  return result_t::SUCCESS;
}

void deinit() {
  unmap_ines();
  cpu::deinit();
  ppu::deinit();
  apu::deinit();
  mapper::deinit();
  memory::deinit();
  menu::deinit();
  input::deinit();
  interrupt::deinit();
  for (int i = 0; i < NUM_SPINLOCKS; i++) {
    nes::spinlock_deinit(i);
  }
  for (int i = 0; i < NUM_SEMAPHORES; i++) {
    nes::semaphore_deinit(i);
  }
}

result_t map_ines(const uint8_t *ines, const char *path) {
  unmap_ines();

  result_t res = result_t::SUCCESS;
  do {
    SemaphoreBlock ppu_block(SEMAPHORE_PPU);
    SemaphoreBlock apu_block(SEMAPHORE_APU);

    if (path && strnlen(path, MAX_PATH_LENGTH + 1) == 0) {
      res = result_t::ERR_FS_PATH_TOO_LONG;
      break;
    }

    res = memory::map_ines(ines);
    if (res != result_t::SUCCESS) break;

    if (path) {
      strncpy(ines_path, path, MAX_PATH_LENGTH);
      ines_path[MAX_PATH_LENGTH] = '\0';
    } else {
      ines_path[0] = '\0';
    }

    reset();
  } while (0);

  if (res == result_t::SUCCESS) {
    ines_mapped = true;
  } else {
    ines_mapped = false;
    ines_path[0] = '\0';
  }
  return res;
}

void unmap_ines() {
  if (!ines_mapped) return;

  {
    SemaphoreBlock ppu_block(SEMAPHORE_PPU);
    SemaphoreBlock apu_block(SEMAPHORE_APU);
    stop();
    memory::unmap_ines();
    ines_mapped = false;
  }

  if (ines_path[0] != '\0') {
    unload_ines();
  }

  ines_path[0] = '\0';
}

result_t reset() {
  nes::state::reset();
  SHAPONES_TRY(mapper::instance->reset());
  SHAPONES_TRY(ppu::reset());
  SHAPONES_TRY(apu::reset());
  SHAPONES_TRY(input::reset());
  SHAPONES_TRY(interrupt::reset());
  SHAPONES_TRY(cpu::reset());
  return result_t::SUCCESS;
}

void stop() { cpu::stop(); }

result_t render_next_line(uint8_t *line_buff, bool skip_render,
                          ppu::status_t *status) {
  ppu::status_t s;
  if (!status) status = &s;
  do {
    SHAPONES_TRY(cpu::service());
    SHAPONES_TRY(ppu::service(line_buff, skip_render, status));
  } while (!(status->timing & ppu::timing_t::END_OF_VISIBLE_LINE));
  return result_t::SUCCESS;
}

result_t vsync(uint8_t *line_buff, bool skip_render) {
  ppu::status_t status;
  do {
    SHAPONES_TRY(cpu::service());
    SHAPONES_TRY(ppu::service(line_buff, skip_render, &status));
  } while (!(status.timing & ppu::timing_t::END_OF_FRAME));
  return result_t::SUCCESS;
}

static void build_blend_table() {
  for (int i = 0; i < 64; i++) {
    uint32_t ci = NES_PALETTE_24BPP[i];
    uint8_t ri = (ci >> 16) & 0xff;
    uint8_t gi = (ci >> 8) & 0xff;
    uint8_t bi = ci & 0xff;
    for (int j = 0; j < 64; j++) {
      uint32_t cj = NES_PALETTE_24BPP[j];
      uint8_t rj = (cj >> 16) & 0xff;
      uint8_t gj = (cj >> 8) & 0xff;
      uint8_t bj = cj & 0xff;
      uint8_t r = (ri + rj) / 2;
      uint8_t g = (gi + gj) / 2;
      uint8_t b = (bi + bj) / 2;
      blend_table[(i << 6) | j] = nearest_rgb888(r, g, b);
    }
  }
}

}  // namespace nes
#endif

#endif
