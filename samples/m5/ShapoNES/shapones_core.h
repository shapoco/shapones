#ifndef SHAPONES_CORE_H
#define SHAPONES_CORE_H

#define SHAPONES_MENU_LARGE_FONT (1)

#ifndef SHAPONES_HPP
#define SHAPONES_HPP

// #include "shapones/apu.hpp"

#ifndef SHAPONES_APU_HPP
#define SHAPONES_APU_HPP

// #include "shapones/common.hpp"

#ifndef SHAPONES_COMMON_HPP
#define SHAPONES_COMMON_HPP

#if !(SHAPONES_NO_STDLIB)
#include <stdint.h>
#include <stdio.h>
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

#if SHAPONES_DEFINE_FAST_INT
using uint_fast8_t = uint8_t;
using uint_fast16_t = uint16_t;
using uint_fast32_t = uint32_t;
using int_fast8_t = int8_t;
using int_fast16_t = int16_t;
using int_fast32_t = int32_t;
#endif

#define SHAPONES_TRY(expr)               \
  do {                                   \
    ::nes::result_t res = (expr);        \
    if (res != nes::result_t::SUCCESS) { \
      return res;                        \
    }                                    \
  } while (false)

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

#define SHAPONES_INLINE inline __attribute__((always_inline))
#define SHAPONES_NOINLINE __attribute__((noinline))

#ifndef SHAPONES_MUTEX_FAST
#define SHAPONES_MUTEX_FAST (0)
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

static constexpr int NUM_LOCKS = 3;
static constexpr int LOCK_INTERRUPTS = 0;
static constexpr int LOCK_PPU = 1;
static constexpr int LOCK_APU = 2;

static constexpr int MAX_FILENAME_LENGTH = SHAPONES_MAX_FILENAME_LEN;
static constexpr int MAX_PATH_LENGTH = SHAPONES_MAX_PATH_LEN;

enum class result_t {
  SUCCESS = 0,
  ERR_INVALID_NES_FORMAT,
  ERR_NO_DISK,
  ERR_DIR_NOT_FOUND,
  ERR_PATH_TOO_LONG,
  ERR_FAILED_TO_READ_FILE,
};

struct Config {
  uint32_t apu_sampling_rate;
};

enum class NametableArrangement : uint8_t {
  HORIZONTAL = 0,
  VERTICAL = 1,
  SINGLE_LOWER = 2,
  SINGLE_UPPER = 3,
  FOUR_SCREEN = 4,
};

void stop();

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

static constexpr int ENV_FLAG_START = 0x1;
static constexpr int ENV_FLAG_CONSTANT = 0x2;
static constexpr int ENV_FLAG_HALT_LOOP = 0x4;

static constexpr int SWP_FLAG_ENABLED = 0x1;
static constexpr int SWP_FLAG_NEGATE = 0x2;

static constexpr int LIN_FLAG_RELOAD = 0x1;
static constexpr int LIN_FLAG_CONTROL = 0x2;

struct Envelope {
  int flags;
  int volume;
  int divider;
  int decay;
};

struct Sweep {
  int flags;
  int period;
  int divider;
  int shift;
};

struct LinearCounter {
  int flags;
  int counter;
  int reload_value;
};

struct PulseState {
  uint8_t waveform;
  uint32_t timer;
  uint32_t timer_period;
  uint32_t phase;
  int length;
  Envelope envelope;
  Sweep sweep;
};

struct TriangleState {
  uint32_t timer;
  uint32_t timer_period;
  uint32_t phase;
  int length;
  LinearCounter linear;
};

struct NoiseState {
  uint16_t lfsr;
  int length;
  Envelope envelope;
};

struct DmcState {
  bool silence;
  bool irq_enabled;
  bool loop;
  uint32_t timer_step;
  uint32_t timer;
  addr_t sample_addr;
  int sample_length;
  addr_t addr_counter;
  int bytes_remaining;
  uint8_t shift_reg;
  int bits_remaining;
  uint8_t out_level;
};

union Status {
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

struct Registers {
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
};

enum RegSel { A, X, Y };

extern volatile cycle_t ppu_cycle_count;

result_t init();
void deinit();

bool is_stopped();

result_t reset();
void stop();

result_t service();

uint8_t bus_read(addr_t addr);
void bus_write(addr_t addr, uint8_t data);

static SHAPONES_INLINE cycle_t ppu_cycle_leading() { return ppu_cycle_count; }

}  // namespace nes::cpu

#endif
// #include "shapones/dma.hpp"

#ifndef SHAPONES_DMA_HPP
#define SHAPONES_DMA_HPP

// #include "shapones/common.hpp"


namespace nes::dma {

static constexpr int TRANSFER_SIZE = 256;

bool is_running();
void start(int src_page);
int exec_next_cycle();

}  // namespace nes::dma

#endif
// #include "shapones/host_intf.hpp"

#ifndef SHAPONES_HOST_INTF_HPP
#define SHAPONES_HOST_INTF_HPP

// #include "shapones/common.hpp"


namespace nes {

struct file_info_t {
  bool is_dir = false;
  const char *name = nullptr;
};

using fs_enum_files_cb_t = void (*)(const file_info_t &info);

result_t lock_init(int id);
void lock_deinit(int id);
void lock_get(int id);
void lock_release(int id);

result_t fs_mount();
void fs_unmount();

result_t fs_get_current_dir(char *out_path);
result_t fs_enum_files(const char *path, fs_enum_files_cb_t callback);

result_t request_load_nes_file(const char *path);

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

union InputStatus {
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

union InputControl {
  uint8_t raw;
  struct {
    uint8_t strobe : 1;
    uint8_t reserved : 7;
  };
};

InputStatus get_status(int player);
void set_status(int player, InputStatus data);
void update();
uint8_t read_latched(int player);
void write_control(uint8_t data);

}  // namespace nes::input

#endif
// #include "shapones/interrupt.hpp"

#ifndef SHAPONES_INTERRUPT_HPP
#define SHAPONES_INTERRUPT_HPP

// #include "shapones/common.hpp"


namespace nes::interrupt {

enum class Source : uint32_t {
  APU_FRAME_COUNTER = (1 << 0),
  APU_DMC = (1 << 1),
  MMC3 = (1 << 2),
};

static SHAPONES_INLINE Source operator|(Source a, Source b) {
  return static_cast<Source>(static_cast<uint32_t>(a) |
                             static_cast<uint32_t>(b));
}
static SHAPONES_INLINE Source operator&(Source a, Source b) {
  return static_cast<Source>(static_cast<uint32_t>(a) &
                             static_cast<uint32_t>(b));
}
static SHAPONES_INLINE bool operator!(Source a) {
  return !static_cast<uint32_t>(a);
}
static SHAPONES_INLINE Source operator~(Source a) {
  return static_cast<Source>(~static_cast<uint32_t>(a));
}
static SHAPONES_INLINE Source& operator|=(Source& a, Source b) {
  a = a | b;
  return a;
}
static SHAPONES_INLINE Source& operator&=(Source& a, Source b) {
  a = a & b;
  return a;
}

void assert_irq(Source src);
void deassert_irq(Source src);
Source get_irq();

void assert_nmi();
void deassert_nmi();
bool is_nmi_asserted();

}  // namespace nes::interrupt

#endif
// #include "shapones/mapper.hpp"

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

struct Registers {
 public:
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
};

static constexpr uint8_t OAM_ATTR_PALETTE = 0x3;
static constexpr uint8_t OAM_ATTR_PRIORITY = 0x20;
static constexpr uint8_t OAM_ATTR_INVERT_H = 0x40;
static constexpr uint8_t OAM_ATTR_INVERT_V = 0x80;

struct OamEntry {
  uint8_t y;
  uint8_t tile;
  uint8_t attr;
  uint8_t x;
};

static constexpr uint8_t SL_ATTR_BEHIND = 0x1;
static constexpr uint8_t SL_ATTR_ZERO = 0x2;

struct SpriteLine {
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

cycle_t cycle_following();

// todo: delete
#if 0
void mmc3_irq_set_enable(bool enable);
void mmc3_irq_set_reload();
void mmc3_irq_set_latch(uint8_t data);
#endif

}  // namespace nes::ppu

#endif

namespace nes::mapper {

class Mapper {
 public:
  const int number;
  const char *name;

  Mapper(int number, const char *name) : number(number), name(name) {}

  virtual result_t init() {  return result_t::SUCCESS; }
  virtual void reset() {}
  virtual bool vblank(const nes::ppu::Registers &reg) { return false; }
  virtual bool hblank(const nes::ppu::Registers &reg, int y) { return false; }
  virtual uint8_t read(addr_t addr) { return 0; }
  virtual void write(addr_t addr, uint8_t value) {}
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

extern const uint8_t *prgrom;
extern const uint8_t *chrrom;
extern uint8_t *prgram;

extern int prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
extern int chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

extern addr_t prgram_addr_mask;
extern uint32_t prgrom_phys_size;
extern uint32_t prgrom_phys_addr_mask;
extern uint32_t chrrom_phys_size;
extern uint32_t chrrom_phys_addr_mask;

result_t init();
void deinit();

result_t map_ines(const uint8_t *ines);
void unmap();

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

void set_nametable_arrangement(NametableArrangement mode);

}  // namespace nes::memory

#endif
// #include "shapones/menu.hpp"

#ifndef SHAPONES_MENU_HPP
#define SHAPONES_MENU_HPP

// #include "shapones/common.hpp"


namespace nes::menu {

enum class tab_t {
  NES_LIST,
  COUNT,
};

extern bool shown;

static SHAPONES_INLINE bool is_shown() { return shown; }

result_t init();
void deinit();

void show();
void hide();

result_t update();
result_t render(int y, uint8_t *line_buff);

}  // namespace nes::menu

#endif
// #include "shapones/ppu.hpp"


namespace nes {

Config get_default_config();

result_t init(const Config &cfg);
void deinit();

result_t map_ines(const uint8_t *ines);
result_t reset();

result_t render_next_line(uint8_t *line_buff, bool skip_render = false,
                          ppu::status_t *status = nullptr);
result_t vsync(uint8_t *line_buff, bool skip_render = false);

}  // namespace nes

#endif
#ifdef SHAPONES_IMPLEMENTATION
// #include "shapones/apu.hpp"

// #include "shapones/cpu.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

#ifndef SHAPONES_LOCK_HPP
#define SHAPONES_LOCK_HPP

// #include "shapones/host_intf.hpp"


namespace nes {

class Exclusive {
 public:
  const int id;
  SHAPONES_INLINE Exclusive(int id) : id(id) { lock_get(id); }
  SHAPONES_INLINE ~Exclusive() { lock_release(id); }
};

}  // namespace nes

#endif// #include "shapones/menu.hpp"


// #pragma GCC optimize("Ofast")


namespace nes::apu {

static constexpr int QUARTER_FRAME_FREQUENCY = 240;
static constexpr int QUARTER_FRAME_PHASE_PREC = 16;
static constexpr uint32_t QUARTER_FRAME_PHASE_PERIOD =
    1ul << QUARTER_FRAME_PHASE_PREC;

static uint32_t sampling_rate;
static uint32_t pulse_timer_step;
static uint32_t triangle_timer_step;
static uint32_t dmc_step_coeff;
static int qframe_phase_step;

static int quarter_frame_phase;
static int quarter_frame_count;
static bool frame_step;
static bool half_frame_step;
static bool quarter_frame_step;

// see: https://www.nesdev.org/wiki/APU_Length_Counter
static constexpr uint8_t LENGTH_TABLE[] = {
    10, 254, 20, 2,  40, 4,  80, 6,  160, 8,  60, 10, 14, 12, 26, 14,
    12, 16,  24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30,
};

// see: https://www.nesdev.org/wiki/APU_DMC
static constexpr uint16_t DMC_RATE_TABLE[16] = {
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
};

static PulseState pulse[2];
static TriangleState triangle;
static NoiseState noise;
static DmcState dmc;
static Status status;

static void pulse_write_reg0(PulseState *s, uint8_t reg0);
static void pulse_write_reg1(PulseState *s, uint8_t reg1);
static void pulse_write_reg2(PulseState *s, uint8_t reg1);
static void pulse_write_reg3(PulseState *s, uint8_t reg3);
static void triangle_write_reg0(TriangleState *s, uint8_t reg0);
static void triangle_write_reg2(TriangleState *s, uint8_t reg3);
static void triangle_write_reg3(TriangleState *s, uint8_t reg3);
static void noise_write_reg0(NoiseState *s, uint8_t reg0);
static void noise_write_reg3(NoiseState *s, uint8_t reg3);
static void dmc_write_reg0(DmcState *s, uint8_t reg0);
static void dmc_write_reg1(DmcState *s, uint8_t reg1);
static void dmc_write_reg2(DmcState *s, uint8_t reg2);
static void dmc_write_reg3(DmcState *s, uint8_t reg3);

result_t init() { return result_t::SUCCESS; }
void deinit() {}

result_t reset() {
  for (int i = 0; i < 2; i++) {
    pulse[i].timer = 0;
    pulse[i].length = 0;
    pulse[i].waveform = 0;
    pulse[i].phase = 0;
    pulse[i].sweep.flags = 0;
  }

  triangle.length = 0;
  noise.length = 0;
  noise.lfsr = 1;

  dmc.silence = true;
  dmc.irq_enabled = false;
  dmc.loop = false;
  dmc.timer_step = 0;
  dmc.timer = 0;
  dmc.sample_addr = 0;
  dmc.sample_length = 0;
  dmc.addr_counter = 0;
  dmc.bytes_remaining = 0;
  dmc.shift_reg = 0;
  dmc.bits_remaining = 0;
  dmc.out_level = 0;

  quarter_frame_phase = 0;
  quarter_frame_count = 0;
  frame_step = false;
  half_frame_step = false;
  quarter_frame_step = false;

  return result_t::SUCCESS;
}

// todo: exclusive control
uint8_t reg_read(addr_t addr) {
  switch (addr) {
    case REG_STATUS: {
      // see: https://www.nesdev.org/wiki/IRQ
      uint8_t ret = status.raw;
      auto irqs = interrupt::get_irq();
      if (!!(irqs & interrupt::Source::APU_DMC)) {
        ret |= 0x80;
      }
      if (!!(irqs & interrupt::Source::APU_FRAME_COUNTER)) {
        ret |= 0x40;
      }
      interrupt::deassert_irq(interrupt::Source::APU_FRAME_COUNTER);
      return ret;
    } break;

    default: return 0;
  }
}

// todo: exclusive control
void reg_write(addr_t addr, uint8_t value) {
  switch (addr) {
    case REG_PULSE1_REG0: pulse_write_reg0(&pulse[0], value); break;
    case REG_PULSE1_REG1: pulse_write_reg1(&pulse[0], value); break;
    case REG_PULSE1_REG2: pulse_write_reg2(&pulse[0], value); break;
    case REG_PULSE1_REG3: pulse_write_reg3(&pulse[0], value); break;
    case REG_PULSE2_REG0: pulse_write_reg0(&pulse[1], value); break;
    case REG_PULSE2_REG1: pulse_write_reg1(&pulse[1], value); break;
    case REG_PULSE2_REG2: pulse_write_reg2(&pulse[1], value); break;
    case REG_PULSE2_REG3: pulse_write_reg3(&pulse[1], value); break;
    case REG_TRIANGLE_REG0: triangle_write_reg0(&triangle, value); break;
    case REG_TRIANGLE_REG2: triangle_write_reg2(&triangle, value); break;
    case REG_TRIANGLE_REG3: triangle_write_reg3(&triangle, value); break;
    case REG_NOISE_REG0: noise_write_reg0(&noise, value); break;
    case REG_NOISE_REG3: noise_write_reg3(&noise, value); break;
    case REG_DMC_REG0: dmc_write_reg0(&dmc, value); break;
    case REG_DMC_REG1: dmc_write_reg1(&dmc, value); break;
    case REG_DMC_REG2: dmc_write_reg2(&dmc, value); break;
    case REG_DMC_REG3: dmc_write_reg3(&dmc, value); break;
    case REG_STATUS:
      status.raw = value;
      if (!status.pulse0_enable) pulse[0].length = 0;
      if (!status.pulse1_enable) pulse[1].length = 0;
      if (!status.triangle_enable) triangle.length = 0;
      if (!status.noise_enable) noise.length = 0;
      if (status.dmc_enable) {
        if (dmc.bytes_remaining == 0) {
          dmc.addr_counter = dmc.sample_addr;
          dmc.bytes_remaining = dmc.sample_length;
          dmc.silence = false;
        }
      } else {
        dmc.bytes_remaining = 0;
        dmc.silence = true;
      }
      // see: https://www.nesdev.org/wiki/IRQ
      interrupt::deassert_irq(interrupt::Source::APU_DMC);
      break;
    case REG_FRAME_COUNTER:
      if (value & 0x80) {
        // todo: implement
      } else {
        // todo: implement
      }
      if (value & 0x40) {
        quarter_frame_count = 0;
        quarter_frame_phase = 0;
        frame_step = true;
        half_frame_step = true;
        quarter_frame_step = true;
      }
      break;
  }
}

result_t set_sampling_rate(uint32_t rate_hz) {
  sampling_rate = rate_hz;
  pulse_timer_step =
      (1ULL << TIMER_PREC) * cpu::CLOCK_FREQ_NTSC / sampling_rate / 2;
  triangle_timer_step =
      (1ULL << TIMER_PREC) * cpu::CLOCK_FREQ_NTSC / sampling_rate;
  qframe_phase_step =
      (QUARTER_FRAME_FREQUENCY * QUARTER_FRAME_PHASE_PERIOD) / rate_hz;
  dmc_step_coeff = ((uint64_t)cpu::CLOCK_FREQ_NTSC << TIMER_PREC) / rate_hz;
  return result_t::SUCCESS;
}

static void pulse_write_reg0(PulseState *s, uint8_t reg0) {
  Exclusive lock(LOCK_APU);
  // duty pattern
  switch ((reg0 >> 6) & 0x3) {
    case 0: s->waveform = 0b00000001; break;
    case 1: s->waveform = 0b00000011; break;
    case 2: s->waveform = 0b00001111; break;
    default: s->waveform = 0b11111100; break;
  }

  s->envelope.flags = 0;
  if (reg0 & 0x10) s->envelope.flags |= ENV_FLAG_CONSTANT;
  if (reg0 & 0x20) s->envelope.flags |= ENV_FLAG_HALT_LOOP;
  s->envelope.volume = reg0 & 0xf;

  if (!(s->envelope.flags & ENV_FLAG_CONSTANT)) {
    s->envelope.flags |= ENV_FLAG_START;
    s->envelope.divider = s->envelope.volume;
  }
}

static void pulse_write_reg1(PulseState *s, uint8_t reg1) {
  Exclusive lock(LOCK_APU);
  s->sweep.period = (reg1 >> 4) & 0x7;
  s->sweep.shift = reg1 & 0x7;
  s->sweep.flags = 0;
  if (reg1 & 0x80) s->sweep.flags |= SWP_FLAG_ENABLED;
  if (reg1 & 0x08) s->sweep.flags |= SWP_FLAG_NEGATE;
  s->sweep.divider = 0;
}

static void pulse_write_reg2(PulseState *s, uint8_t reg2) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0xff << TIMER_PREC);
  s->timer_period |= (uint32_t)reg2 << TIMER_PREC;
}

static void pulse_write_reg3(PulseState *s, uint8_t reg3) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0x700 << TIMER_PREC);
  s->timer_period |= (uint32_t)(reg3 & 0x7) << (TIMER_PREC + 8);
  s->timer = 0;
  s->length = LENGTH_TABLE[(reg3 >> 3) & 0x1f];
  s->phase = 0;
}

static void triangle_write_reg0(TriangleState *s, uint8_t reg0) {
  Exclusive lock(LOCK_APU);
  s->linear.reload_value = reg0 & 0x7f;
  if (reg0 & 0x80) {
    s->linear.flags |= LIN_FLAG_CONTROL;
  } else {
    s->linear.flags &= ~LIN_FLAG_CONTROL;
  }
}

static void triangle_write_reg2(TriangleState *s, uint8_t reg2) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0xff << TIMER_PREC);
  s->timer_period |= (uint32_t)reg2 << TIMER_PREC;
}

static void triangle_write_reg3(TriangleState *s, uint8_t reg3) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0x700 << TIMER_PREC);
  s->timer_period |= (uint32_t)(reg3 & 0x7) << (TIMER_PREC + 8);
  s->timer = 0;
  s->length = LENGTH_TABLE[(reg3 >> 3) & 0x1f];
  s->phase = 0;
  s->linear.flags |= LIN_FLAG_RELOAD;
}

static void noise_write_reg0(NoiseState *s, uint8_t reg0) {
  Exclusive lock(LOCK_APU);
  s->envelope.flags = 0;
  if (reg0 & 0x10) s->envelope.flags |= ENV_FLAG_CONSTANT;
  if (reg0 & 0x20) s->envelope.flags |= ENV_FLAG_HALT_LOOP;
  s->envelope.volume = reg0 & 0xf;

  if (!(s->envelope.flags & ENV_FLAG_CONSTANT)) {
    s->envelope.flags |= ENV_FLAG_START;
    s->envelope.divider = s->envelope.volume;
  }
}

static void noise_write_reg3(NoiseState *s, uint8_t reg3) {
  Exclusive lock(LOCK_APU);
  s->length = LENGTH_TABLE[(reg3 >> 3) & 0x1f];
}

static void dmc_write_reg0(DmcState *s, uint8_t reg0) {
  bool irq_ena_old = s->irq_enabled;
  bool irq_ena_new = (reg0 & 0x80) != 0;
  if (irq_ena_new && !irq_ena_old) {
    interrupt::deassert_irq(interrupt::Source::APU_DMC);
  }
  Exclusive lock(LOCK_APU);
  s->irq_enabled = irq_ena_new;
  s->loop = (reg0 & 0x40) != 0;
  s->timer_step = dmc_step_coeff / DMC_RATE_TABLE[reg0 & 0x0f];
}

static void dmc_write_reg1(DmcState *s, uint8_t reg0) {
  Exclusive lock(LOCK_APU);
  s->out_level = reg0 & 0x7f;
}

static void dmc_write_reg2(DmcState *s, uint8_t reg0) {
  Exclusive lock(LOCK_APU);
  s->sample_addr = 0xc000 + ((addr_t)reg0 << 6);
}

static void dmc_write_reg3(DmcState *s, uint8_t reg0) {
  Exclusive lock(LOCK_APU);
  s->sample_length = ((int)reg0 << 4) + 1;
}

// see: https://www.nesdev.org/wiki/APU_Envelope
static SHAPONES_INLINE int step_envelope(Envelope *e) {
  if (quarter_frame_step) {
    if (e->flags & ENV_FLAG_START) {
      e->flags &= ~ENV_FLAG_START;  // clear start flag
      e->decay = 15;
      e->divider = e->volume;
    } else if (e->divider > 0) {
      e->divider--;
    } else {
      e->divider = e->volume;
      if (e->decay > 0) {
        e->decay--;
      } else if (e->flags & ENV_FLAG_HALT_LOOP) {
        e->decay = 15;
      }
    }
  }

  if (e->flags & ENV_FLAG_CONSTANT) {
    return e->volume;
  } else {
    return e->decay;
  }
}

// see: https://www.nesdev.org/wiki/APU_Sweep
static SHAPONES_INLINE int step_sweep(PulseState &s) {
  int gate = 1;
  if (s.sweep.flags & SWP_FLAG_ENABLED) {
    if (half_frame_step) {
      if (s.sweep.divider < s.sweep.period) {
        s.sweep.divider++;
      } else {
        s.sweep.divider = 0;
        int change_amount = s.timer_period;
        change_amount >>= s.sweep.shift;
        if (s.sweep.flags & SWP_FLAG_NEGATE) {
          change_amount = -change_amount;
        }
        int target = s.timer_period + change_amount;
        if (target < 0) {
          target = 0;
        } else if (target >= (0x7ff << TIMER_PREC)) {
          target = (0x7ff << TIMER_PREC);
          gate = 0;
        }
        {
#if SHAPONES_MUTEX_FAST
          Exclusive lock(LOCK_APU);
#endif
          s.timer_period = target;
        }
      }
    }
  }
  return gate;
}

// see: https://www.nesdev.org/wiki/APU_Triangle
static SHAPONES_INLINE int step_linear_counter(LinearCounter &c) {
  // linear counter
  if (quarter_frame_step) {
    if (c.flags & LIN_FLAG_RELOAD) {
      c.counter = c.reload_value;
    } else if (c.counter > 0) {
      c.counter--;
    }

    if (!(c.flags & LIN_FLAG_CONTROL)) {
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_APU);
#endif
      c.flags &= ~LIN_FLAG_RELOAD;  // clear reload flag
    }
  }
  return c.counter > 0 ? 1 : 0;
}

// see: https://www.nesdev.org/wiki/APU_Pulse
static SHAPONES_INLINE int sample_pulse(PulseState &s) {
  // envelope unit
  int vol = step_envelope(&(s.envelope));

  // sweep unit
  vol *= step_sweep(s);

  // length counter
  if (half_frame_step && !(s.envelope.flags & ENV_FLAG_HALT_LOOP)) {
    if (s.length >= 0) {
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_APU);
#endif
      s.length--;
    }
  }

  // mute
  if (s.timer_period < 8) vol = 0;
  if (s.length < 0) vol = 0;

  // sequencer
  uint32_t phase;
  {
#if SHAPONES_MUTEX_FAST
    Exclusive lock(LOCK_APU);
#endif
    s.timer += pulse_timer_step;
    int step = 0;
    if (s.timer_period != 0) {
      step = s.timer / s.timer_period;
    }
    s.timer -= step * s.timer_period;
    s.phase = (s.phase + step) & 0x7;
    phase = s.phase;
  }

  int amp = (s.waveform >> phase) & 1;
  return amp * vol;
}

// see: https://www.nesdev.org/wiki/APU_Triangle
static SHAPONES_INLINE int sample_triangle(TriangleState &s) {
  int vol = 1;

  // length counter
  if (half_frame_step && !(s.linear.flags & LIN_FLAG_CONTROL)) {
    if (s.length >= 0) {
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_APU);
#endif
      s.length--;
    }
  }

  // linear counter
  step_linear_counter(s.linear);

  // phase counter
  uint32_t phase;
  {
#if SHAPONES_MUTEX_FAST
    Exclusive lock(LOCK_APU);
#endif
    s.timer += triangle_timer_step;
    int step = 0;
    if (s.timer_period != 0) {
      step = s.timer / s.timer_period;
    }
    s.timer -= step * s.timer_period;
    if (s.length > 0 && s.linear.counter > 0) {
      s.phase = (s.phase + step) & 0x1f;
    }
    phase = s.phase;
  }

  // mute
  if (s.timer_period < 8) vol = 0;
  // sequencer
  if (phase <= 15) {
    return (15 - phase) * vol;
  } else {
    return (phase - 16) * vol;
  }
}

// see: https://www.nesdev.org/wiki/APU_Noise
static SHAPONES_INLINE int sample_noise(NoiseState &s) {
  // envelope unit
  int vol = step_envelope(&(s.envelope));

  // length counter
  if (half_frame_step && !(s.envelope.flags & ENV_FLAG_HALT_LOOP)) {
    if (s.length >= 0) {
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_APU);
#endif
      s.length--;
    }
  }

  // mute
  if (s.length < 0) vol = 0;

  // LFSR
  int feedback = ((s.lfsr >> 1) ^ s.lfsr) & 1;
  s.lfsr = ((s.lfsr >> 1) & 0x3fff) | (feedback << 14);
  int amp = s.lfsr & 0xf;

  return (amp * vol) >> 4;
}

// see: https://www.nesdev.org/wiki/APU_DMC
static SHAPONES_INLINE int sample_dmc(DmcState &s) {
  int step;
  {
#if SHAPONES_MUTEX_FAST
    Exclusive lock(LOCK_APU);
#endif
    s.timer += s.timer_step;
    step = s.timer >> TIMER_PREC;
    s.timer &= (1 << TIMER_PREC) - 1;
  }

  for (int i = 0; i < step; i++) {
    if (s.bits_remaining == 0) {
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_APU);
      ;
#endif
      if (s.bytes_remaining > 0) {
        s.shift_reg = cpu::bus_read(s.addr_counter | 0x8000);
        s.addr_counter++;
        s.bytes_remaining--;
        if (s.bytes_remaining == 0) {
          if (s.loop) {
            s.addr_counter = s.sample_addr;
            s.bytes_remaining = s.sample_length;
          } else if (s.irq_enabled) {
            interrupt::assert_irq(interrupt::Source::APU_DMC);
          }
        }
      }
      s.bits_remaining = 8;
    }
    if (s.bits_remaining > 0) {
      if (!s.silence) {
        if (s.shift_reg & 1) {
#if SHAPONES_MUTEX_FAST
          Exclusive lock(LOCK_APU);
#endif
          if (s.out_level <= 125) {
            s.out_level += 2;
          }
        } else {
#if SHAPONES_MUTEX_FAST
          Exclusive lock(LOCK_APU);
#endif
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
  if (nes::menu::is_shown()) {
    for (int i = 0; i < len; i++) {
      buff[i] = 0;
    }
    return result_t::SUCCESS;
  }

#if !SHAPONES_MUTEX_FAST
  Exclusive lock(LOCK_APU);
#endif
  for (int i = 0; i < len; i++) {
    quarter_frame_phase += qframe_phase_step;
    if (quarter_frame_phase >= QUARTER_FRAME_PHASE_PERIOD) {
      quarter_frame_phase -= QUARTER_FRAME_PHASE_PERIOD;

      frame_step = (quarter_frame_count & 3) == 3;
      half_frame_step = (quarter_frame_count & 1) == 1;
      quarter_frame_step = true;

      quarter_frame_count = (quarter_frame_count + 1) & 0x3;
    } else {
      frame_step = false;
      half_frame_step = false;
      quarter_frame_step = false;
    }

    uint8_t sample = 0;
    sample += sample_pulse(pulse[0]);
    sample += sample_pulse(pulse[1]);
    sample += sample_triangle(triangle);
    sample += sample_noise(noise);
    sample *= 2;
    sample += sample_dmc(dmc);
    buff[i] = sample;
  }
  return result_t::SUCCESS;
}

}  // namespace nes::apu
// #include "shapones/cpu.hpp"

// #include "shapones/apu.hpp"

// #include "shapones/common.hpp"

// #include "shapones/dma.hpp"

// #include "shapones/input.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"

// #include "shapones/ppu.hpp"

// #include "shapones/menu.hpp"


// #pragma GCC optimize("Ofast")


namespace nes::cpu {

static Registers reg;
static bool stopped = false;
#if SHAPONES_IRQ_PENDING_SUPPORT
static int irq_pending = 0;
#else
static constexpr int irq_pending = 0;
#endif
volatile cycle_t ppu_cycle_count;

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
  } else if (apu::REG_PULSE1_REG0 <= addr && addr <= apu::REG_DMC_REG3 ||
             addr == apu::REG_STATUS) {
    apu::reg_write(addr, data);
  } else if (addr == OAM_DMA_REG) {
    dma::start(data);
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

result_t init() { return result_t::SUCCESS; }

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
  stopped = false;
  ppu_cycle_count = 0;
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
  addr_t addr = baseAddr + reg.Y;
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
  // SHAPONES_PRINTF("RTI to PC=0x%04x, SP=0x%02x\n", (unsigned int)reg.PC,
  // (unsigned int)reg.SP);
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
  constexpr int MAX_BATCH_SIZE = 32;

  input::update();

  if (menu::is_shown()) {
    return result_t::SUCCESS;
  }

  int n = MAX_BATCH_SIZE;
  while (n-- > 0) {
    cycle_t cycle = 0;

    cycle_t ppu_cycle_diff = ppu_cycle_count - ppu::cycle_following();
    if (ppu_cycle_diff >= ppu::MAX_DELAY_CYCLES) {
      break;
    }

    if (stopped) {
      cycle += 1;  // nop
    } else if (dma::is_running()) {
      // DMA is running
      cycle += dma::exec_next_cycle();
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
      // SHAPONES_PRINTF("NMI PC=0x%04x, SP=0x%02x, interrupt=%d\n", (unsigned
      // int)reg.PC, (unsigned int)reg.SP, (int)reg.status.interrupt);
      cycle += 7;  // ?
    } else if (irq_pending == 0 && !!interrupt::get_irq() &&
               !reg.status.interrupt) {
      // IRQ
      auto irq_source = interrupt::get_irq();
      auto s = reg.status;
      s.breakmode = 0;
      push(reg.PC >> 8);
      push(reg.PC & 0xff);
      push(s.raw);
      reg.status.interrupt = true;
      reg.PC = bus_read_w(VEC_IRQ);
      // SHAPONES_PRINTF("IRQ 0x%02x, PC=0x%04x, SP=0x%02x\n", (unsigned
      // int)interrupt::get_irq(), (unsigned int)reg.PC, (unsigned int)reg.SP);
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

}  // namespace nes::cpu
// #include "shapones/dma.hpp"

// #include "shapones/cpu.hpp"

// #include "shapones/ppu.hpp"


namespace nes::dma {

static bool running = false;
static int next_cycle = 0;
static addr_t next_src_addr = 0;

bool is_running() { return running; }

void start(int src_page) {
  running = true;
  next_cycle = 0;
  next_src_addr = (addr_t)src_page * 0x100;
}

int exec_next_cycle() {
  if (!running) return 0;

  ppu::oam_dma_write(next_cycle++, cpu::bus_read(next_src_addr++));

  if (next_cycle >= TRANSFER_SIZE) {
    running = false;
  }
  return 2;
}

}  // namespace nes::dma// #include "shapones/input.hpp"

// #include "shapones/menu.hpp"


namespace nes::input {

static InputControl reg;
static InputStatus raw[2];
static uint8_t shift_reg[2];

InputStatus get_status(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  return raw[player];
}

void set_status(int player, InputStatus s) {
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

}  // namespace nes::input
// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"


namespace nes::interrupt {

static volatile Source irq;
static volatile bool nmi;

void assert_irq(Source src) {
  Exclusive lock(LOCK_INTERRUPTS);
  irq = irq | src;
}
void deassert_irq(Source src) {
  Exclusive lock(LOCK_INTERRUPTS);
  irq = irq & ~src;
}
Source get_irq() { return irq; }

void assert_nmi() { nmi = true; }
void deassert_nmi() { nmi = false; }
bool is_nmi_asserted() { return nmi; }

}  // namespace nes::interrupt
// #include "shapones/mapper.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/memory.hpp"


#define SHAPONES_MAPPER_IMPLEMENTATION
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
  Map001() : Mapper(1, "MMC1") {}

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
      switch (ctrl_reg & 0x03) {
        default:
        case 0:
          set_nametable_arrangement(NametableArrangement::SINGLE_LOWER);
          break;
        case 1:
          set_nametable_arrangement(NametableArrangement::SINGLE_UPPER);
          break;
        case 2:
          set_nametable_arrangement(NametableArrangement::HORIZONTAL);
          break;
        case 3: set_nametable_arrangement(NametableArrangement::VERTICAL); break;
      }

      switch (ctrl_reg & 0x0C) {
        default:
        case 0x00:
        case 0x04: prgrom_remap(0x8000, (prg_bank & 0x0E) << 14, 0x8000); break;
        case 0x08:
          prgrom_remap(0x8000, 0, 0x4000);
          prgrom_remap(0xC000, (prg_bank & 0x0F) << 14, 0x4000);
          break;
        case 0x0C:
          prgrom_remap(0x8000, (prg_bank & 0x0F) << 14, 0x4000);
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
 public:
  Map003() : Mapper(3, "CNROM") {}

  void write(addr_t addr, uint8_t value) override {
    if (0x8000 <= addr && addr <= 0xffff) {
      chrrom_remap(0x0000, (value & 0x3) * 0x2000, 0x2000);
    }
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

  volatile bool irq_enable = false;
  volatile bool irq_reloading = false;
  volatile uint8_t irq_latch = 255;
  volatile uint8_t irq_counter = 0;

 public:
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
    } else if (0xA000 <= addr && addr <= 0xBFFF) {
      if ((addr & 0x0001) == 0) {
        if ((value & 0x01) == 0) {
          set_nametable_arrangement(NametableArrangement::HORIZONTAL);
        } else {
          set_nametable_arrangement(NametableArrangement::VERTICAL);
        }
      } else {
        // PRG RAM protect
        // TODO
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
        interrupt::deassert_irq(interrupt::Source::MMC3);
      }
      irq_enable = enable_new;
    }
  }

  bool hblank(const nes::ppu::Registers &reg, int y) override {
    bool irq = false;
    if (reg.mask.bg_enable && reg.mask.sprite_enable) {
      uint8_t irq_counter_before = irq_counter;
      if (irq_counter == 0 || irq_reloading) {
        irq_reloading = false;
        irq_counter = irq_latch;
      } else {
        irq_counter--;
      }
      if (irq_enable && irq_counter == 0) {
        interrupt::assert_irq(interrupt::Source::MMC3);
        irq = true;
      }
    }
    return irq;
  }
};

}  // namespace nes::mapper

#endif

// #pragma GCC optimize("Ofast")


namespace nes::mapper {

Mapper *instance = nullptr;

result_t init() {
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
  SHAPONES_PRINTF("Mapper No.%d\n", id);

  if (instance) {
    delete instance;
  }
  switch (id) {
    case 0: instance = new Map000(); break;
    case 1: instance = new Map001(); break;
    case 3: instance = new Map003(); break;
    case 4: instance = new Map004(); break;
    default:
      instance = new Map000();
      SHAPONES_ERRORF("Unsupported Mapper Number\n");
      break;
  }

   SHAPONES_TRY(instance->init());

  return result_t::SUCCESS;
}

}  // namespace nes::mapper
// #include "shapones/memory.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/ppu.hpp"


// #pragma GCC optimize("Ofast")


namespace nes::memory {

uint8_t wram[WRAM_SIZE];
uint8_t vram[VRAM_SIZE];
addr_t vram_addr_and = VRAM_SIZE - 1;
addr_t vram_addr_or = 0;

uint8_t dummy_memory = 0;

uint8_t *prgram = nullptr;
const uint8_t *prgrom = &dummy_memory;
const uint8_t *chrrom = &dummy_memory;

int prgrom_remap_table[PRGROM_REMAP_TABLE_SIZE];
int chrrom_remap_table[CHRROM_REMAP_TABLE_SIZE];

addr_t prgram_addr_mask = 0;
uint32_t prgrom_phys_size = PRGROM_RANGE;
uint32_t prgrom_phys_addr_mask = 0;
uint32_t chrrom_phys_size = CHRROM_RANGE;
uint32_t chrrom_phys_addr_mask = 0;
addr_t prgrom_cpu_addr_mask = PRGROM_RANGE - 1;

result_t init() {
  if (prgram) {
    prgram = new uint8_t[1];
  }
  unmap();
  return result_t::SUCCESS;
}

void deinit() {
  unmap();
  if (prgram) {
    delete[] prgram;
    prgram = nullptr;
  }
}

result_t map_ines(const uint8_t *ines) {
  // iNES file format
  // https://www.nesdev.org/wiki/INES

  unmap();

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
  chrrom_phys_size = num_chr_rom_pages * CHRROM_PAGE_SIZE;
  chrrom_phys_addr_mask = chrrom_phys_size - 1;
  SHAPONES_PRINTF("Number of CHRROM pages = %d (%dkB)\n", num_chr_rom_pages,
                  chrrom_phys_size / 1024);

  prgram_addr_mask = 0;
  for (int i = 0; i < PRGROM_REMAP_TABLE_SIZE; i++) {
    prgrom_remap_table[i] = i;
  }
  for (int i = 0; i < CHRROM_REMAP_TABLE_SIZE; i++) {
    chrrom_remap_table[i] = i;
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
  set_nametable_arrangement(mode);

  int prgram_size = ines[8] * 8192;
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
  bool has_trainer = (flags6 & 0x2) != 0;

  int start_of_prg_rom = 0x10;
  if (has_trainer) start_of_prg_rom += 0x200;
  prgrom = ines + start_of_prg_rom;

  int start_of_chr_rom = start_of_prg_rom + num_prg_rom_pages * 0x4000;
  chrrom = ines + start_of_chr_rom;

  SHAPONES_TRY(mapper::map_ines(ines));

  return result_t::SUCCESS;
}

void unmap() {
  prgrom = &dummy_memory;
  chrrom = &dummy_memory;
  prgram_addr_mask = 0;
  prgrom_phys_addr_mask = 0;
  chrrom_phys_addr_mask = 0;
}

void set_nametable_arrangement(NametableArrangement mode) {
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
// #include "shapones/menu.hpp"

// #include "shapones/font12x24.hpp"

#ifndef SHAPONES_FONT12X24_HPP
#define SHAPONES_FONT12X24_HPP

#include <stdint.h>

namespace nes::menu {

const uint16_t FONT12X24_CODE_FIRST = 0x20;
const uint16_t FONT12X24_CODE_LAST = 0xAF;
const uint32_t FONT12X24_DATA[] = {
  // 0x20 ' '
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x21 '!'
  0x0000, 0x0000, 0x0000, 0x0000, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
  0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x0000, 0x0000,
  0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x22 '"'
  0x0000, 0x0000, 0x7D07D, 0xFF0FF, 0xFF0FF, 0xFD0FD, 0xF00F0, 0x7C07C,
  0x3F03F, 0x7007, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x23 '#'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1E1E0, 0x1E1E0, 0x1E1E0, 0x1E1E0,
  0xFFFFF, 0xFFFFF, 0xF0F0, 0xF0F0, 0xF0F0, 0xF0F0, 0xFFFFF, 0xFFFFF,
  0xB4B4, 0xB4B4, 0xB4B4, 0xB4B4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x24 '$'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0F00, 0x0F00, 0x1FFF4, 0x7FFFD,
  0xF8F2F, 0xF0F2F, 0x0FBD, 0x2FF4, 0x1FF80, 0x7EF00, 0xF8F0F, 0xF8F2F,
  0x7FFFD, 0x1FFF4, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x25 '%'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF02F8, 0xF87FD, 0xBCFAF, 0x3EF0F,
  0x2FF0F, 0xFFAF, 0xBFFD, 0x3FF8, 0x2FFC0, 0x7FFE0, 0xFAFF0, 0xF0FF8,
  0xF0FBC, 0xFAF3E, 0x7FD2F, 0x2F80F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x26 '&'
  0x0000, 0x0000, 0x0000, 0x0000, 0x02F8, 0x07FD, 0x0FAF, 0x0F0F,
  0x0F0F, 0x0FAF, 0xF0BFE, 0xF83FC, 0xBCBFC, 0x3FFFE, 0x2FD7F, 0xF41F,
  0x2F41F, 0x3FD7F, 0xBFFFD, 0xF9FF4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x27 '''
  0x0000, 0x0000, 0x7D00, 0xFF00, 0xFF00, 0xFD00, 0xF000, 0x7C00,
  0x3F00, 0x0700, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x28 '('
  0x0000, 0x0000, 0xFF000, 0x3FC00, 0xFE00, 0xBF40, 0x7FC0, 0x3FD0,
  0x2FE0, 0x1FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x1FF0, 0x2FE0,
  0x3FD0, 0x7FC0, 0xBF40, 0xFE00, 0x3FC00, 0xFF000, 0x0000, 0x0000,
  // 0x29 ')'
  0x0000, 0x0000, 0x00FF, 0x03FC, 0x0BF0, 0x1FE0, 0x3FD0, 0x7FC0,
  0xBF80, 0xFF40, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF40, 0xBF80,
  0x7FC0, 0x3FD0, 0x1FE0, 0x0BF0, 0x03FC, 0x00FF, 0x0000, 0x0000,
  // 0x2A '*'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0F00, 0x0F00,
  0xF0F0F, 0xFCF3F, 0x3FFFC, 0xFFF0, 0x3FC0, 0x3FC0, 0xFFF0, 0x3FFFC,
  0xFCF3F, 0xF0F0F, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2B '+'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0xFFFFF, 0xFFFFF, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2C ','
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x07D0, 0x0FF0, 0x0FF0, 0x0FD0, 0x0F00, 0x07C0, 0x03F0, 0x0070,
  // 0x2D '-'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2E '.'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x07D0, 0x0FF0, 0x0FF0, 0x07D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2F '/'
  0x0000, 0x0000, 0xFF000, 0xFF400, 0xBF800, 0x7FC00, 0x3FD00, 0x2FE00,
  0x1FF00, 0xFF40, 0xBF80, 0x7FC0, 0x3FD0, 0x2FE0, 0x1FF0, 0x0FF4,
  0x0BF8, 0x07FC, 0x03FD, 0x02FE, 0x01FF, 0x00FF, 0x0000, 0x0000,
  // 0x30 '0'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF000F, 0xFD00F, 0xFFC0F, 0xFFF4F, 0xF1FFF, 0xF03FF, 0xF007F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x31 '1'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0F00, 0x0FD0, 0x0FFC, 0x0F7C,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x32 '2'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF8000, 0x7C000, 0x2F000, 0xFC00, 0x3F00, 0x0FC0, 0x03F0, 0x00FC,
  0x007D, 0x003F, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x33 '3'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x3F000, 0xFC00,
  0x3F00, 0x0FC0, 0x7FF0, 0x2FFF0, 0x7E000, 0xF8000, 0xF0000, 0xF0000,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x34 '4'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3D00, 0x1F00, 0x0F40, 0x07C0,
  0x03D0, 0x01F0, 0xF0F4, 0xF07C, 0xF03D, 0xF01F, 0xFFFFF, 0xFFFFF,
  0xF000, 0xF000, 0xF000, 0xF000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x35 '5'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFC, 0xFFFFC, 0x003D, 0x003D,
  0x002E, 0x002E, 0x7FFF, 0x2FFFF, 0x7E000, 0xF8000, 0xF0000, 0xF0000,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x36 '6'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0BC0, 0x07D0, 0x03E0, 0x02F0,
  0x01F4, 0x00F8, 0x7FFC, 0x2FFFD, 0x7E0BE, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x37 '7'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0xF400F, 0xF800F,
  0xBC00F, 0x7D00F, 0x3E000, 0x2F000, 0x1F400, 0xF800, 0xBC00, 0x7D00,
  0x3E00, 0x2F00, 0x1F40, 0x0F80, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x38 '8'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x2FFF8, 0x7E0BD, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x39 '9'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF000F, 0xF000F, 0xF802F, 0xBE0BD, 0x7FFF8, 0x3FFD0, 0x2F000, 0x1F400,
  0xF800, 0xBC00, 0x7D00, 0x3E00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3A ':'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x07D0, 0x0FF0, 0x0FF0, 0x07D0, 0x0000, 0x0000, 0x0000, 0x0000,
  0x07D0, 0x0FF0, 0x0FF0, 0x07D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3B ';'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x07D0, 0x0FF0, 0x0FF0, 0x07D0, 0x0000, 0x0000, 0x0000, 0x0000,
  0x07D0, 0x0FF0, 0x0FF0, 0x0FD0, 0x0F00, 0x07C0, 0x03F0, 0x0070,
  // 0x3C '<'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xD0000,
  0xF8000, 0xFF400, 0x2FE00, 0x7FD0, 0x0BF8, 0x01FF, 0x01FF, 0x0BF8,
  0x7FD0, 0x2FE00, 0xFF400, 0xF8000, 0xD0000, 0x0000, 0x0000, 0x0000,
  // 0x3D '='
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
  0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3E '>'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0007,
  0x002F, 0x01FF, 0x0BF8, 0x7FD0, 0x2FE00, 0xFF400, 0xFF400, 0x2FE00,
  0x7FD0, 0x0BF8, 0x01FF, 0x002F, 0x0007, 0x0000, 0x0000, 0x0000,
  // 0x3F '?'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF800F, 0xFC00F, 0x7F000, 0xFC00, 0x3E00, 0x1F00, 0x0000, 0x0000,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x40 '@'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF0F0F, 0xF0F0F, 0xF0F0F, 0xF0F0F, 0xF0F0F, 0xF8F0F, 0xFFF0F, 0xF7D0F,
  0x002F, 0x00BD, 0xFFF8, 0xFFD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x41 'A'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0F00, 0x1F40, 0x2F80, 0x3FC0,
  0x7FD0, 0xBFE0, 0xFAF0, 0x1F5F4, 0x2F0F8, 0x3E0BC, 0x7FFFD, 0xBFFFE,
  0xF802F, 0xF401F, 0xF000F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x42 'B'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FFF, 0x2FFFF, 0x7E00F, 0xF800F,
  0xF800F, 0x7E00F, 0x2FFFF, 0x2FFFF, 0x7E00F, 0xF800F, 0xF000F, 0xF000F,
  0xF800F, 0x7E00F, 0x2FFFF, 0x7FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x43 'C'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x44 'D'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FFF, 0x2FFFF, 0x7E00F, 0xF800F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF800F, 0x7E00F, 0x2FFFF, 0x7FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x45 'E'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x000F, 0x000F,
  0x000F, 0x000F, 0xFFFF, 0xFFFF, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x46 'F'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x000F, 0x000F,
  0x000F, 0x000F, 0xFFFF, 0xFFFF, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0x000F, 0x000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x47 'G'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0x000F, 0x000F, 0x000F, 0x000F, 0xFFF0F, 0xFFF0F, 0xF000F, 0xF000F,
  0xF802F, 0xFE0BD, 0xFFFF8, 0xF7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x48 'H'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF000F, 0xF000F, 0xFFFFF, 0xFFFFF, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x49 'I'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFF0, 0xFFF0, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0xFFF0, 0xFFF0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4A 'J'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF0000, 0xF0000, 0xF0000, 0xF0000,
  0xF0000, 0xF0000, 0xF0000, 0xF0000, 0xF0000, 0xF0000, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4B 'K'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3F000F, 0xFC00F, 0x3F00F, 0xFC0F,
  0x3F0F, 0x0FCF, 0x03FF, 0x00FF, 0x00FF, 0x03FF, 0x0FCF, 0x3F0F,
  0xFC0F, 0x3F00F, 0xFC00F, 0x3F000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4C 'L'
  0x0000, 0x0000, 0x0000, 0x0000, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4D 'M'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF401F, 0xF802F, 0xFC03F,
  0xFD07F, 0xFE0BF, 0xFF0FF, 0xFF5FF, 0xFFAFF, 0xFBFEF, 0xF7FDF, 0xF3FCF,
  0xF2F8F, 0xF1F4F, 0xF0F0F, 0xF0A0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4E 'N'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF002F, 0xF003F, 0xF00BF, 0xF00FF,
  0xF02FF, 0xF03EF, 0xF0BCF, 0xF0F8F, 0xF2F0F, 0xF3E0F, 0xFBC0F, 0xFF80F,
  0xFF00F, 0xFE00F, 0xFC00F, 0xF800F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4F 'O'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x50 'P'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FFF, 0x2FFFF, 0x7E00F, 0xF800F,
  0xF000F, 0xF000F, 0xF800F, 0x7E00F, 0x2FFFF, 0x7FFF, 0x000F, 0x000F,
  0x000F, 0x000F, 0x000F, 0x000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x51 'Q'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF0FCF, 0xF2F0F, 0xF3E0F, 0xFFC0F,
  0xFF42F, 0x7F0BD, 0xFFFF8, 0xF7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x52 'R'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FFF, 0x2FFFF, 0x7E00F, 0xF800F,
  0xF000F, 0xF000F, 0xF800F, 0x7E00F, 0x2FFFF, 0x7FFF, 0xBD0F, 0x1F80F,
  0x3F00F, 0xBD00F, 0x1F800F, 0x3F000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x53 'S'
  0x0000, 0x0000, 0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F,
  0x002F, 0x00BD, 0x0BF8, 0x7FD0, 0x2FE00, 0x7E000, 0xF8000, 0xF0000,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x54 'T'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x55 'U'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x56 'V'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF000F, 0xF401F, 0xF802F,
  0xBC03E, 0x7D07D, 0x3E0BC, 0x2F0F8, 0x1F5F4, 0xFAF0, 0xBFE0, 0x7FD0,
  0x3FC0, 0x2F80, 0x1F40, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x57 'W'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF401F, 0xF401F, 0xF802F,
  0xB8F2E, 0xBCF3E, 0x7DF7D, 0x7EFBD, 0x3FFFC, 0x3FFFC, 0x2FAF8, 0x2FAF8,
  0x1F0F4, 0x1F0F4, 0xF0F0, 0xF0F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x58 'X'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF802F, 0xBC03E, 0x3E0BC,
  0x2F0F8, 0xFAF0, 0xBFE0, 0x3FC0, 0x3FC0, 0xBFE0, 0xFAF0, 0x2F0F8,
  0x3E0BC, 0xBC03E, 0xF802F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x59 'Y'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF000F, 0xF802F, 0xBC03E, 0x3E0BC,
  0x2F0F8, 0xFAF0, 0xBFE0, 0x3FC0, 0x2F80, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5A 'Z'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0xFC000, 0xBE000,
  0x3F400, 0x1FC00, 0xBE00, 0x3F40, 0x1FC0, 0x0BE0, 0x03F4, 0x01FC,
  0x00BE, 0x003F, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5B '['
  0x0000, 0x0000, 0xFFFF0, 0xFFFF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0,
  0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0,
  0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0xFFFF0, 0xFFFF0, 0x0000, 0x0000,
  // 0x5C '\'
  0x0000, 0x0000, 0x00FF, 0x01FF, 0x02FE, 0x03FD, 0x07FC, 0x0BF8,
  0x0FF4, 0x1FF0, 0x2FE0, 0x3FD0, 0x7FC0, 0xBF80, 0xFF40, 0x1FF00,
  0x2FE00, 0x3FD00, 0x7FC00, 0xBF800, 0xFF400, 0xFF000, 0x0000, 0x0000,
  // 0x5D ']'
  0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFF00, 0xFF00, 0xFF00, 0xFF00,
  0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00,
  0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFFFF, 0xFFFF, 0x0000, 0x0000,
  // 0x5E '^'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0F00, 0x3FC0, 0xFFF0, 0x3F0FC,
  0xFC03F, 0xF000F, 0xC0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5F '_'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000,
  // 0x60 '`'
  0x0000, 0x0000, 0x7F00, 0xFC00, 0x1F000, 0x3C000, 0x70000, 0xC0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x61 'a'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xBFE0, 0x3FFFC, 0xBD004, 0xF4000, 0xFFFF8, 0xFFFFE,
  0xF401F, 0xF801F, 0xFFFFE, 0xF7FF8, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x62 'b'
  0x0000, 0x0000, 0x0000, 0x0000, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0x7FDF, 0x2FFFF, 0x7E0BF, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BF, 0x2FFFF, 0x7FDF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x63 'c'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x1FFD0, 0xFFFF8, 0xB80BD, 0x1002F, 0x000F, 0x000F,
  0x1002F, 0xF80BD, 0xBFFF8, 0x1FFD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x64 'd'
  0x0000, 0x0000, 0x0000, 0x0000, 0xF0000, 0xF0000, 0xF0000, 0xF0000,
  0xF0000, 0xF0000, 0xF7FD0, 0xFFFF8, 0xFE0BD, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0xFE0BD, 0xFFFF8, 0xF7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x65 'e'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F, 0xFFFFF, 0xFFFFF,
  0x002F, 0x00BD, 0xFFF8, 0xFFD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x66 'f'
  0x0000, 0x0000, 0x0000, 0x0000, 0x1FF40, 0x7FFD0, 0xF82F0, 0xF00F0,
  0x00F0, 0x00F0, 0xFFFFF, 0xFFFFF, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
  0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x67 'g'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF7FD0, 0xFFFF8, 0xFE0BD, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0xFE0BD, 0xFFFF8, 0xF7FD0, 0xF8000, 0x7E000, 0x2FFF0, 0x7FF0,
  // 0x68 'h'
  0x0000, 0x0000, 0x0000, 0x0000, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0x7FDF, 0x2FFFF, 0x7E0BF, 0xF802F, 0xF000F, 0xF000F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x69 'i'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0,
  0x0000, 0x0000, 0x0FF0, 0x0FF0, 0x0F00, 0x0F00, 0x0F00, 0x0F00,
  0x0F00, 0x0F00, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6A 'j'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xFF00, 0xFF00,
  0x0000, 0x0000, 0xFF00, 0xFF00, 0xF000, 0xF000, 0xF000, 0xF000,
  0xF000, 0xF000, 0xF000, 0xF000, 0xF82F, 0x7EBD, 0x2FF8, 0x07D0,
  // 0x6B 'k'
  0x0000, 0x0000, 0x0000, 0x0000, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0xD000F, 0xF800F, 0xFF40F, 0x2FE0F, 0x7FDF, 0x0FFF, 0x3FFF,
  0xFC2F, 0x3F00F, 0xFC00F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6C 'l'
  0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xF000, 0xF000,
  0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000,
  0xF000, 0xF000, 0xF000, 0xF000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6D 'm'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xB7DF, 0x3FFFF, 0xBFFFF, 0xF5F5F, 0xF0F0F, 0xF0F0F,
  0xF0F0F, 0xF0F0F, 0xF0F0F, 0xF0F0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6E 'n'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x7FDF, 0x2FFFF, 0x7E0BF, 0xF802F, 0xF000F, 0xF000F,
  0xF000F, 0xF000F, 0xF000F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6F 'o'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x7FD0, 0x2FFF8, 0x7E0BD, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BD, 0x2FFF8, 0x7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x70 'p'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x7FDF, 0x2FFFF, 0x7E0BF, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0x7E0BF, 0x2FFFF, 0x7FDF, 0x000F, 0x000F, 0x000F, 0x000F,
  // 0x71 'q'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF7FD0, 0xFFFF8, 0xFE0BD, 0xF802F, 0xF000F, 0xF000F,
  0xF802F, 0xFE0BD, 0xFFFF8, 0xF7FD0, 0xF0000, 0xF0000, 0xF0000, 0xF0000,
  // 0x72 'r'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x7FDF, 0x2FFFF, 0x7E0BF, 0xF802F, 0x000F, 0x000F,
  0x000F, 0x000F, 0x000F, 0x000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x73 's'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x2FFF8, 0x7FFFE, 0x1007F, 0x007F, 0x2FFFE, 0xBFFF8,
  0xFD000, 0xFD004, 0xBFFFD, 0x2FFF8, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x74 't'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x00F0, 0x00F0, 0xFFFFF, 0xFFFFF, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
  0xF00F0, 0xF82F0, 0x7FFD0, 0x1FF40, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x75 'u'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F, 0xF000F,
  0xF802F, 0xFE0BD, 0xFFFF8, 0xF7FD0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x76 'v'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF000F, 0xF802F, 0xBC03E, 0x3E0BC, 0x2F0F8, 0xFAF0,
  0xBFE0, 0x3FC0, 0x2F80, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x77 'w'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF000F, 0xF802F, 0xBCF3E, 0x7CF3D, 0x3DF7C, 0x3EFBC,
  0x2FFF8, 0x1FAF4, 0xF5F0, 0xF0F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x78 'x'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF000F, 0xFC03F, 0x3F0FC, 0xFFF0, 0x3FC0, 0x3FC0,
  0xFFF0, 0x3F0FC, 0xFC03F, 0xF000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x79 'y'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xF000F, 0xF802F, 0xBC03E, 0x3E0BC, 0x2F0F8, 0xFAF0,
  0xBFE0, 0x3FC0, 0x2F80, 0x0F80, 0x0BC0, 0x03E0, 0x02FF, 0x00BF,
  // 0x7A 'z'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xFFFFF, 0xFFFFF, 0x3F000, 0xFC00, 0x3F00, 0x0FC0,
  0x03F0, 0x00FC, 0xFFFFF, 0xFFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x7B '{'
  0x0000, 0x0000, 0xFFE00, 0xFFFC0, 0x2FE0, 0x0FF0, 0x0FF0, 0x0FF0,
  0x0FF0, 0x0FF0, 0x0BF8, 0x03FF, 0x03FF, 0x0BF8, 0x0FF0, 0x0FF0,
  0x0FF0, 0x0FF0, 0x0FF0, 0x2FE0, 0xFFFC0, 0xFFE00, 0x0000, 0x0000,
  // 0x7C '|'
  0x0000, 0x0000, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0,
  0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0,
  0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0FF0, 0x0000, 0x0000,
  // 0x7D '}'
  0x0000, 0x0000, 0x0BFF, 0x3FFF, 0xBF80, 0xFF00, 0xFF00, 0xFF00,
  0xFF00, 0xFF00, 0x2FE00, 0xFFC00, 0xFFC00, 0x2FE00, 0xFF00, 0xFF00,
  0xFF00, 0xFF00, 0xFF00, 0xBF80, 0x3FFF, 0x0BFF, 0x0000, 0x0000,
  // 0x7E '~'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x02F8, 0x0BFE, 0xF0FFF, 0xF0F5F, 0xF5F0F, 0xFFF0F,
  0xBFE00, 0x2F800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x7F ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x80 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x81 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x82 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x83 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x84 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x85 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x86 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x87 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x88 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x89 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x8A ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x8B ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x8C ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x8D ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x8E ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x8F ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x90 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x91 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x92 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x93 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x94 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x95 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x96 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x97 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x98 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x99 ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x9A ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x9B ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x9C ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x9D ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x9E ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0x9F ''
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0xA0 ' '
  0x6AAAAA, 0x1AAAAA, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A,
  0x5695A, 0x57D5A, 0x5BE5A, 0x5FF5A, 0x6FF9A, 0x7FFDA, 0xBFFEA, 0xFFFFA,
  0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x0006, 0x0001,
  // 0xA1 '¡'
  0x6AAAAA, 0x1AAAAA, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A,
  0xFFFFA, 0xBFFEA, 0x7FFDA, 0x6FF9A, 0x5FF5A, 0x5BE5A, 0x57D5A, 0x5695A,
  0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x000A, 0x0002,
  // 0xA2 '¢'
  0x111111, 0x444444, 0x111111, 0x444444, 0x111111, 0x444444, 0x111111, 0x444444,
  0x111111, 0x444444, 0x111111, 0x444444, 0x111111, 0x444444, 0x111111, 0x444444,
  0x111111, 0x444444, 0x111111, 0x444444, 0x111111, 0x444444, 0x111111, 0x444444,
  // 0xA3 '£'
  0x6AAAAA, 0x1AAAAA, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0xAAAAA, 0xAAAAA,
  0x000A, 0x000A, 0xAAAAA, 0xAAAAA, 0x000A, 0x000A, 0xAAAAA, 0xAAAAA,
  0x000A, 0x000A, 0x5555A, 0x5555A, 0x5555A, 0x5555A, 0x0006, 0x0001,
  // 0xA4 '¤'
  0x0000, 0x0000, 0x0000, 0x28000, 0xAA000, 0x2AA800, 0xAAAA00, 0xAAAA80,
  0xAAAAA0, 0xAAAAA0, 0xAA000, 0xAA000, 0xAA000, 0xAA000, 0x1AA000, 0x6AA000,
  0xAAA000, 0xAA9000, 0xAA8000, 0xA90000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0xA5 '¥'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0002,
  0x000A, 0x000A, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0xA6 '¦'
  0x0000, 0x0000, 0xBFE00, 0x17FFC0, 0xE402E0, 0xF000F0, 0x00F0, 0x00F0,
  0xFF80F0, 0xFFE0F0, 0x12F0F0, 0x44F0F0, 0x11F0F0, 0x44F0F0, 0x11F0F0, 0x44F0F0,
  0x11F0F0, 0x44F1F0, 0xAAAAE0, 0xAAAA40, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0xA7 '§'
  0x0000, 0x0000, 0x0000, 0x0000, 0x02FF, 0x05FF, 0x0540, 0x0500,
  0x2FFFF, 0x7FFFF, 0xA4511, 0xA0544, 0xA0511, 0xA0544, 0xA0511, 0xA0544,
  0xA0511, 0xA4544, 0x6AAAA, 0x1AAAA, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0xA8 '¨'
  0x0000, 0x0000, 0xFFF800, 0xFFFE00, 0x0F00, 0x0F00, 0xFFFF00, 0xFFFF00,
  0x0F00, 0x0F00, 0xFFFF00, 0xFFFF00, 0x0F00, 0x0F00, 0xFFFF00, 0xFFFF00,
  0x0F00, 0x0F00, 0xFFFF00, 0xFFFF00, 0xFFFE00, 0xFFF800, 0x0000, 0x0000,
  // 0xA9 '©'
  0x0000, 0x0000, 0x2FFF, 0xBFFF, 0xFF00, 0xFF00, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFF0, 0xFFF0, 0xFFFF, 0xFFFF,
  0x000F, 0x000F, 0x6A0F, 0x1A0F, 0x060F, 0x010F, 0x0000, 0x0000,
  // 0xAA 'ª'
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0xAB '«'
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0xAC '¬'
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0xAD '­'
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0xAE '®'
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
  // 0xAF '¯'
  0x555555, 0x555555, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005,
  0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x500005, 0x555555, 0x555555,
};

}  // namespace nes

#endif // SHAPONES_FONT12X24_HPP
// #include "shapones/font8x16.hpp"

#ifndef SHAPONES_FONT8X16_HPP
#define SHAPONES_FONT8X16_HPP

#include <stdint.h>

namespace nes::menu {

const uint16_t FONT8X16_CODE_FIRST = 0x20;
const uint16_t FONT8X16_CODE_LAST = 0xAF;
const uint16_t FONT8X16_DATA[] = {
  // 0x20 ' '
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x21 '!'
  0x0000, 0x0000, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0,
  0x0000, 0x0000, 0x03F0, 0x03F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x22 '"'
  0x0000, 0x2E2E, 0x3F3F, 0x3E3E, 0x2C2C, 0x0B0B, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x23 '#'
  0x0000, 0x0000, 0x1E78, 0x3FFF, 0x3FFF, 0x1E78, 0x0F3C, 0x0F3C,
  0x0B6D, 0x3FFF, 0x3FFF, 0x0B6D, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x24 '$'
  0x0000, 0x01D0, 0x0BF8, 0x2FFE, 0x3DDF, 0x00EF, 0x06FD, 0x1FE4,
  0x3EC0, 0x3DDF, 0x2FFE, 0x0BF8, 0x01D0, 0x0000, 0x0000, 0x0000,
  // 0x25 '%'
  0x0000, 0x0000, 0x3DB8, 0x1FFE, 0x0FCF, 0x07FE, 0x03F8, 0x0BF0,
  0x2FF4, 0x3CFC, 0x2FFD, 0x0B9F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x26 '&'
  0x0000, 0x0000, 0x00B8, 0x02FE, 0x03CF, 0x3EFE, 0x3DFC, 0x1FED,
  0x0F4F, 0x1F1F, 0x3FFD, 0x35F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x27 '''
  0x0000, 0x02E0, 0x03F0, 0x03E0, 0x02C0, 0x00B0, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x28 '('
  0x0000, 0x0FC0, 0x07E0, 0x03F4, 0x02F8, 0x01FC, 0x00FC, 0x00FC,
  0x01FC, 0x02F8, 0x03F4, 0x07E0, 0x0FC0, 0x0000, 0x0000, 0x0000,
  // 0x29 ')'
  0x0000, 0x00FC, 0x02F4, 0x07F0, 0x0BE0, 0x0FD0, 0x0FC0, 0x0FC0,
  0x0FD0, 0x0BE0, 0x07F0, 0x02F4, 0x00FC, 0x0000, 0x0000, 0x0000,
  // 0x2A '*'
  0x0000, 0x0000, 0x0000, 0x03C0, 0xE7DB, 0xFFFF, 0x1BE4, 0x1BE4,
  0xFFFF, 0xE7DB, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2B '+'
  0x0000, 0x0000, 0x0000, 0x0000, 0x03C0, 0x03C0, 0x3FFC, 0x3FFC,
  0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2C ','
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x02E0, 0x03F0, 0x03E0, 0x02C0, 0x00B0, 0x0000,
  // 0x2D '-'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3FFC, 0x3FFC,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x2E '.'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x02E0, 0x03F0, 0x02E0, 0x0000, 0x0000, 0x0000,
  // 0x2F '/'
  0x0000, 0x3F00, 0x3F00, 0x1F80, 0x0FC0, 0x0BD0, 0x03F0, 0x03F0,
  0x01F8, 0x00FC, 0x00BD, 0x003F, 0x003F, 0x0000, 0x0000, 0x0000,
  // 0x30 '0'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0F, 0x3F9F, 0x3DBF,
  0x3C0F, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x31 '1'
  0x0000, 0x0000, 0x03C0, 0x03F4, 0x03FF, 0x03C7, 0x03C0, 0x03C0,
  0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x32 '2'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0C, 0x3F00, 0x0FC0,
  0x03F0, 0x00FC, 0x3FFF, 0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x33 '3'
  0x0000, 0x0000, 0x0FFE, 0x0FFE, 0x07C0, 0x01F0, 0x07FC, 0x1FFC,
  0x3D00, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x34 '4'
  0x0000, 0x0000, 0x00F0, 0x00B4, 0x0078, 0x0F3C, 0x0F2D, 0x0F1E,
  0x3FFF, 0x3FFF, 0x0F00, 0x0F00, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x35 '5'
  0x0000, 0x0000, 0x0FFC, 0x0FFC, 0x003D, 0x003E, 0x07FF, 0x1FFF,
  0x3D00, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x36 '6'
  0x0000, 0x0000, 0x01F0, 0x00F4, 0x007C, 0x07FD, 0x1FFF, 0x3E2F,
  0x3C0F, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x37 '7'
  0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x3C0F, 0x3D0F, 0x2E00, 0x1F00,
  0x0F40, 0x0B80, 0x07C0, 0x03D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x38 '8'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3D1F, 0x2D1E, 0x0BF8, 0x1FFD,
  0x3D1F, 0x3D1F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x39 '9'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0F, 0x3E2F, 0x3FFD,
  0x1FF4, 0x0F40, 0x07C0, 0x03D0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3A ':'
  0x0000, 0x0000, 0x0000, 0x0000, 0x02E0, 0x03F0, 0x02E0, 0x0000,
  0x0000, 0x02E0, 0x03F0, 0x02E0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3B ';'
  0x0000, 0x0000, 0x0000, 0x0000, 0x02E0, 0x03F0, 0x02E0, 0x0000,
  0x0000, 0x02E0, 0x03F0, 0x03E0, 0x02C0, 0x00B0, 0x0000, 0x0000,
  // 0x3C '<'
  0x0000, 0x0000, 0x0300, 0x0FC0, 0x03F0, 0x00FC, 0x003F, 0x003F,
  0x00FC, 0x03F0, 0x0FC0, 0x0300, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3D '='
  0x0000, 0x0000, 0x0000, 0x0000, 0x0FFC, 0x0FFC, 0x0000, 0x0000,
  0x0FFC, 0x0FFC, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3E '>'
  0x0000, 0x0000, 0x0030, 0x00FC, 0x03F0, 0x0FC0, 0x3F00, 0x3F00,
  0x0FC0, 0x03F0, 0x00FC, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x3F '?'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3D1F, 0x3C0F, 0x3F00, 0x0FC0,
  0x03C0, 0x0000, 0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x40 '@'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3F8F, 0x3CCF, 0x3CCF,
  0x378F, 0x002F, 0x0FFD, 0x0FF4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x41 'A'
  0x0000, 0x0000, 0x03F0, 0x07F4, 0x0BB8, 0x0F7C, 0x1F3D, 0x2E2E,
  0x3FFF, 0x3FFF, 0x3C0F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x42 'B'
  0x0000, 0x0000, 0x0BFF, 0x2FFF, 0x3E0F, 0x3D0F, 0x1FFF, 0x1FFF,
  0x3D0F, 0x3E0F, 0x2FFF, 0x0BFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x43 'C'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x2E2F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x2E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x44 'D'
  0x0000, 0x0000, 0x07FF, 0x1FFF, 0x3E0F, 0x3C0F, 0x3C0F, 0x3C0F,
  0x3C0F, 0x3E0F, 0x1FFF, 0x07FF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x45 'E'
  0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x000F, 0x000F, 0x0FFF, 0x0FFF,
  0x000F, 0x000F, 0x3FFF, 0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x46 'F'
  0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x000F, 0x000F, 0x0FFF, 0x0FFF,
  0x000F, 0x000F, 0x000F, 0x000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x47 'G'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x2E2F, 0x000F, 0x3FCF, 0x3FCF,
  0x3C0F, 0x3D2F, 0x3FFD, 0x3DF4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x48 'H'
  0x0000, 0x0000, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3FFF, 0x3FFF,
  0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x49 'I'
  0x0000, 0x0000, 0x03FC, 0x03FC, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
  0x00F0, 0x00F0, 0x03FC, 0x03FC, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4A 'J'
  0x0000, 0x0000, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00, 0x3C00,
  0x3C00, 0x3E2E, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4B 'K'
  0x0000, 0x0000, 0x0C0F, 0x3F0F, 0x0FCF, 0x03FF, 0x00FF, 0x00FF,
  0x03FF, 0x0FCF, 0x3F0F, 0x0C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4C 'L'
  0x0000, 0x0000, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F, 0x000F,
  0x000F, 0x000F, 0x3FFF, 0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4D 'M'
  0x0000, 0x0000, 0x3C0F, 0x3D1F, 0x3E2F, 0x3F3F, 0x3F7F, 0x3FBF,
  0x3EEF, 0x3DDF, 0x3CCF, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4E 'N'
  0x0000, 0x0000, 0x3C0F, 0x3C1F, 0x3C3F, 0x3CBF, 0x3CFF, 0x3FFF,
  0x3F8F, 0x3F0F, 0x3D0F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x4F 'O'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0F, 0x3C0F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x50 'P'
  0x0000, 0x0000, 0x07FF, 0x1FFF, 0x3E0F, 0x3C0F, 0x3E0F, 0x1FFF,
  0x07FF, 0x000F, 0x000F, 0x000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x51 'Q'
  0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0F, 0x3C0F, 0x3EDF,
  0x3F8F, 0x3F2F, 0x2FFD, 0x3BF4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x52 'R'
  0x0000, 0x0000, 0x07FF, 0x1FFF, 0x3E0F, 0x3C0F, 0x3E0F, 0x1FFF,
  0x07FF, 0x0F0F, 0x2E0F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x53 'S'
  0x0000, 0x0000, 0x0BF8, 0x2FFE, 0x3D1F, 0x002F, 0x06FD, 0x1FE4,
  0x3E00, 0x3D1F, 0x2FFE, 0x0BF8, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x54 'T'
  0x0000, 0x0000, 0xBFFE, 0xBFFE, 0x03C0, 0x03C0, 0x03C0, 0x03C0,
  0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x55 'U'
  0x0000, 0x0000, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x56 'V'
  0x0000, 0x0000, 0x3C0F, 0x3C0F, 0x3D1F, 0x2E2E, 0x1F3D, 0x0F7C,
  0x0BB8, 0x07F4, 0x03F0, 0x02E0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x57 'W'
  0x0000, 0x0000, 0x3C0F, 0x3C0F, 0x3CCF, 0x3DDF, 0x2EEE, 0x2FFE,
  0x1FBD, 0x1F7D, 0x0F3C, 0x0F3C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x58 'X'
  0x0000, 0x0000, 0x3C0F, 0x3D1F, 0x1F3D, 0x0F7C, 0x07F4, 0x07F4,
  0x0F7C, 0x1F3D, 0x3D1F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x59 'Y'
  0x0000, 0x0000, 0xF00F, 0xF41F, 0x7C3D, 0x3D7C, 0x1FF4, 0x0FF0,
  0x07D0, 0x03C0, 0x03C0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5A 'Z'
  0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x0F00, 0x0BC0, 0x03D0, 0x01F0,
  0x00F8, 0x003C, 0x3FFF, 0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5B '['
  0x0000, 0x0FFC, 0x0FFC, 0x00FC, 0x00FC, 0x00FC, 0x00FC, 0x00FC,
  0x00FC, 0x00FC, 0x00FC, 0x0FFC, 0x0FFC, 0x0000, 0x0000, 0x0000,
  // 0x5C '\'
  0x0000, 0x003F, 0x003F, 0x00BD, 0x00FC, 0x01F8, 0x03F0, 0x03F0,
  0x0BD0, 0x0FC0, 0x1F80, 0x3F00, 0x3F00, 0x0000, 0x0000, 0x0000,
  // 0x5D ']'
  0x0000, 0x3FFC, 0x3FFC, 0x3F00, 0x3F00, 0x3F00, 0x3F00, 0x3F00,
  0x3F00, 0x3F00, 0x3F00, 0x3FFC, 0x3FFC, 0x0000, 0x0000, 0x0000,
  // 0x5E '^'
  0x0000, 0x00C0, 0x03F0, 0x0FFC, 0x3F3F, 0x0C0C, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x5F '_'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x0000, 0x0000, 0x0000,
  // 0x60 '`'
  0x0000, 0x03F0, 0x07C0, 0x0B00, 0x0C00, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x61 'a'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0BF8, 0x2FFE, 0x3C04, 0x3FF8,
  0x3FFE, 0x3E0F, 0x3FFE, 0x3DF8, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x62 'b'
  0x0000, 0x0000, 0x000F, 0x000F, 0x07DF, 0x1FFF, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x1FFF, 0x07DF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x63 'c'
  0x0000, 0x0000, 0x0000, 0x0000, 0x07F4, 0x1FFD, 0x0D2F, 0x000F,
  0x000F, 0x0D2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x64 'd'
  0x0000, 0x0000, 0x3C00, 0x3C00, 0x3DF4, 0x3FFD, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x3FFD, 0x3DF4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x65 'e'
  0x0000, 0x0000, 0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0F,
  0x3FFF, 0x002F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x66 'f'
  0x0000, 0x0000, 0x0BE0, 0x2FF8, 0x0D7C, 0x003C, 0x03FF, 0x03FF,
  0x003C, 0x003C, 0x003C, 0x003C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x67 'g'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3DF4, 0x3FFD, 0x3E2F, 0x3C0F,
  0x3E2F, 0x3FFD, 0x3DF4, 0x3E1C, 0x1FFD, 0x07F4, 0x0000, 0x0000,
  // 0x68 'h'
  0x0000, 0x0000, 0x000F, 0x000F, 0x07DF, 0x1FFF, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x69 'i'
  0x0000, 0x0000, 0x00F0, 0x00F0, 0x0000, 0x00F0, 0x00F0, 0x00F0,
  0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6A 'j'
  0x0000, 0x0000, 0x03C0, 0x03C0, 0x0000, 0x03C0, 0x03C0, 0x03C0,
  0x03C0, 0x03C0, 0x03C0, 0x03E0, 0x01FF, 0x007F, 0x0000, 0x0000,
  // 0x6B 'k'
  0x0000, 0x0000, 0x000F, 0x000F, 0x0C0F, 0x3F0F, 0x0FCF, 0x03FF,
  0x03FF, 0x0FCF, 0x3F0F, 0x0C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6C 'l'
  0x0000, 0x0000, 0x00FC, 0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x00F0,
  0x00F0, 0x00F0, 0x00F0, 0x00F0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6D 'm'
  0x0000, 0x0000, 0x0000, 0x0000, 0x09B7, 0x2FFF, 0x3FFF, 0x3DDF,
  0x3CCF, 0x3CCF, 0x3CCF, 0x3CCF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6E 'n'
  0x0000, 0x0000, 0x0000, 0x0000, 0x07DF, 0x1FFF, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x6F 'o'
  0x0000, 0x0000, 0x0000, 0x0000, 0x07F4, 0x1FFD, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x1FFD, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x70 'p'
  0x0000, 0x0000, 0x0000, 0x0000, 0x07DF, 0x1FFF, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x1FFF, 0x07DF, 0x000F, 0x000F, 0x0000, 0x0000,
  // 0x71 'q'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3DF4, 0x3FFD, 0x3E2F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x3FFD, 0x3DF4, 0x3C00, 0x3C00, 0x0000, 0x0000,
  // 0x72 'r'
  0x0000, 0x0000, 0x0000, 0x0000, 0x07DF, 0x1FFF, 0x0D2F, 0x000F,
  0x000F, 0x000F, 0x000F, 0x000F, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x73 's'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0BF8, 0x2FFE, 0x3D1F, 0x07FD,
  0x1FF4, 0x3D1F, 0x2FFE, 0x0BF8, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x74 't'
  0x0000, 0x0000, 0x003C, 0x003C, 0x0FFF, 0x0FFF, 0x003C, 0x003C,
  0x003C, 0x0D7C, 0x2FF8, 0x0BE0, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x75 'u'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3C0F, 0x3C0F, 0x3C0F, 0x3C0F,
  0x3C0F, 0x3E2F, 0x3FFD, 0x3DF4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x76 'v'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3C0F, 0x3D1F, 0x3E2F, 0x2F3E,
  0x1F7D, 0x0FBC, 0x0BF8, 0x07F4, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x77 'w'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3C0F, 0x3CCF, 0x3DDF, 0x2EEE,
  0x2FFE, 0x1FBD, 0x1F7D, 0x0F3C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x78 'x'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0C0C, 0x3E2F, 0x0FFC, 0x07F4,
  0x07F4, 0x0FFC, 0x3E2F, 0x0C0C, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x79 'y'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3C0F, 0x3C0F, 0x3D1F, 0x1F3D,
  0x0FFC, 0x07F4, 0x03F0, 0x00FC, 0x003F, 0x000C, 0x0000, 0x0000,
  // 0x7A 'z'
  0x0000, 0x0000, 0x0000, 0x0000, 0x3FFF, 0x3FFF, 0x3F00, 0x0FC0,
  0x03F0, 0x00FC, 0x3FFF, 0x3FFF, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x7B '{'
  0x0000, 0x0000, 0x0FE0, 0x0FF8, 0x01FC, 0x00FC, 0x00FC, 0x007F,
  0x007F, 0x00FC, 0x00FC, 0x01FC, 0x0FF8, 0x0FE0, 0x0000, 0x0000,
  // 0x7C '|'
  0x0000, 0x0000, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0,
  0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x03F0, 0x0000, 0x0000,
  // 0x7D '}'
  0x0000, 0x0000, 0x02FC, 0x0BFC, 0x0FD0, 0x0FC0, 0x0FC0, 0x3F40,
  0x3F40, 0x0FC0, 0x0FC0, 0x0FD0, 0x0BFC, 0x02FC, 0x0000, 0x0000,
  // 0x7E '~'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3C7D, 0x3CFF,
  0x3FCF, 0x1F4F, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  // 0x7F ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x80 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x81 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x82 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x83 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x84 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x85 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x86 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x87 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x88 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x89 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8A ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8B ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8C ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8D ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8E ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x8F ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x90 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x91 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x92 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x93 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x94 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x95 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x96 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
  // 0x97 ''
  0x5555, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001,
  0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x4001, 0x5555,
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
  0x6AAA, 0x1556, 0x1556, 0x1556, 0x1556, 0x1696, 0x17D6, 0x1BE6,
  0x1FF6, 0x2FFA, 0x3FFE, 0x1556, 0x1556, 0x1556, 0x1556, 0x0001,
  // 0xA1 '¡'
  0x6AAA, 0x1556, 0x1556, 0x1556, 0x1556, 0x3FFE, 0x2FFA, 0x1FF6,
  0x1BE6, 0x17D6, 0x1696, 0x1556, 0x1556, 0x1556, 0x1556, 0x0001,
  // 0xA2 '¢'
  0x1111, 0x4444, 0x1111, 0x4444, 0x1111, 0x4444, 0x1111, 0x4444,
  0x1111, 0x4444, 0x1111, 0x4444, 0x1111, 0x4444, 0x1111, 0x4444,
  // 0xA3 '£'
  0x6AAA, 0x1556, 0x1556, 0x1556, 0x1556, 0x1AA6, 0x1006, 0x1AA6,
  0x1006, 0x1AA6, 0x1006, 0x1556, 0x1556, 0x1556, 0x1556, 0x0001,
  // 0xA4 '¤'
  0x0000, 0x0000, 0x2000, 0xA800, 0xAA00, 0xAA80, 0xAAA0, 0xA800,
  0xA800, 0xA800, 0xA800, 0xA800, 0xA400, 0x9000, 0x0000, 0x0000,
  // 0xA5 '¥'
  0x0000, 0x0000, 0x0000, 0x0000, 0x0002, 0x000A, 0x002A, 0x0000,
  0x0000, 0x0000, 0x0001, 0x0AAA, 0x0AAA, 0x0AAA, 0x0000, 0x0000,
  // 0xA6 '¦'
  0x0000, 0x0000, 0x1F80, 0x55E0, 0x8070, 0x0030, 0xFE30, 0x0330,
  0x0330, 0x0330, 0x0330, 0x0330, 0xAA90, 0x0000, 0x0000, 0x0000,
  // 0xA7 '§'
  0x0000, 0x0000, 0x0000, 0x0000, 0x007F, 0x0055, 0x07FF, 0x0840,
  0x0840, 0x0840, 0x0840, 0x0840, 0x06AA, 0x0000, 0x0000, 0x0000,
  // 0xA8 '¨'
  0x0000, 0x0000, 0xFF80, 0xFFC0, 0x03C0, 0xFFC0, 0x03C0, 0xFFC0,
  0x03C0, 0xFFC0, 0x03C0, 0xFFC0, 0xFFC0, 0xFF80, 0x0000, 0x0000,
  // 0xA9 '©'
  0x0000, 0x0000, 0x02FF, 0x03FF, 0x03F0, 0x03FF, 0x03FC, 0x03FF,
  0x03C0, 0x03FF, 0x0003, 0x01A3, 0x0063, 0x0013, 0x0000, 0x0000,
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
};

}  // namespace nes

#endif // SHAPONES_FONT8X16_HPP
// #include "shapones/host_intf.hpp"

// #include "shapones/input.hpp"


namespace nes::menu {

static constexpr int MAX_NUM_FILES = 1024;
#if SHAPONES_MENU_LARGE_FONT
static constexpr int CHAR_WIDTH = 12;
static constexpr int CHAR_HEIGHT = 24;
static constexpr int CLIENT_X = 0;
static constexpr int CLIENT_Y = 0;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT;
#else
static constexpr int CHAR_WIDTH = 8;
static constexpr int CHAR_HEIGHT = 16;
static constexpr int CLIENT_X = 3;
static constexpr int CLIENT_Y = 2;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH - 6;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT - 4;
#endif
static constexpr int BUFF_WIDTH = SCREEN_WIDTH / CHAR_WIDTH;
static constexpr int BUFF_HEIGHT = SCREEN_HEIGHT / CHAR_HEIGHT;

bool shown = false;
tab_t tab = tab_t::NES_LIST;

const uint8_t PALETTE_FILE[] = {0x1D, 0x00, 0x10, 0x20, 0x01, 0x11, 0x21, 0x20};
static constexpr int NUM_PALETTES = sizeof(PALETTE_FILE) / 4;

uint8_t text_buff[BUFF_WIDTH * BUFF_HEIGHT];
uint8_t palette_buff[BUFF_WIDTH * BUFF_HEIGHT];

input::InputStatus key_pressed;
input::InputStatus key_down;

bool disk_mounted = false;
char current_dir[nes::MAX_PATH_LENGTH] = "";
file_info_t file_list[MAX_NUM_FILES];
int list_scroll_y = 0;
int selected_index = 0;
int num_files = 0;

static result_t change_tab(tab_t tab, bool force = false);
static result_t update_nes_list();
static void clear_file_list();
static result_t refresh_file_list();
static void print_file_list();
static void scroll_to(int index);
static int print_text(int x, int y, const char *str, int max_len = 999999);
static void print_char(int x, int y, char c);
static void fill_text(int x, int y, int w, int h, char c = ' ');
static void fill_palette(int x, int y, int w, int h, uint8_t p = 0x00);
static void clip_rect(int *x, int *y, int *w, int *h);
static bool is_root_dir(const char *path);
static int find_parent_separator(const char *path);
static int find_last_separator(const char *path, int start_idx = -1);
static result_t append_separator(char *path);
static result_t append_path(char *path, const char *name);

result_t init() {
  key_pressed.raw = 0;
  key_down.raw = 0;

  shown = false;
  tab = tab_t::NES_LIST;
  clear_file_list();
  for (int i = 0; i < BUFF_WIDTH * BUFF_HEIGHT; i++) {
    text_buff[i] = '\0';
    palette_buff[i] = 0x00;
  }
  return result_t::SUCCESS;
}
void deinit() { clear_file_list(); }

void show() {
  if (shown) return;
  shown = true;
  change_tab(tab, true);
}

void hide() {
  if (!shown) return;
  shown = false;
}

result_t update() {
  if (!shown) return result_t::SUCCESS;

  input::InputStatus prev_pressed = key_pressed;
  key_pressed = input::get_status(0);
  key_down.raw = key_pressed.raw & ~prev_pressed.raw;

  switch (tab) {
    default:
    case tab_t::NES_LIST: SHAPONES_TRY(update_nes_list()); break;
  }
  return result_t::SUCCESS;
}

static result_t update_nes_list() {
  if (!disk_mounted) refresh_file_list();

  if (num_files == 0) return result_t::SUCCESS;

  if (key_down.up) {
    selected_index = (selected_index + num_files - 1) % num_files;
    if (selected_index < list_scroll_y) {
      list_scroll_y--;
    }
    scroll_to(selected_index);
  } else if (key_down.down) {
    selected_index = (selected_index + 1) % num_files;
    if (selected_index >= list_scroll_y + CLIENT_HEIGHT) {
      list_scroll_y++;
    }
    scroll_to(selected_index);
  } else if (key_down.A) {
    if (0 <= selected_index && selected_index < num_files) {
      file_info_t &fi = file_list[selected_index];
      if (!fi.is_dir) {
        char path[nes::MAX_PATH_LENGTH];
        strncpy(path, current_dir, nes::MAX_PATH_LENGTH);
        SHAPONES_TRY(append_path(path, fi.name));
        SHAPONES_TRY(request_load_nes_file(path));
      } else {
        if (strcmp(fi.name, "..") == 0) {
          // parent directory
          int sep_idx = find_parent_separator(current_dir);
          if (sep_idx >= 0) {
            current_dir[sep_idx] = '\0';
          } else {
            current_dir[0] = '\0';
          }
        } else {
          // sub directory
          SHAPONES_TRY(append_path(current_dir, fi.name));
        }
        refresh_file_list();
      }
    }
  }

  return result_t::SUCCESS;
}

result_t render(int y, uint8_t *line_buff) {
  if (!shown) return result_t::SUCCESS;

  int iy = y / CHAR_HEIGHT;
  if (iy < 0 || iy >= BUFF_HEIGHT) {
    return result_t::SUCCESS;
  }

  for (int ix = 0; ix < BUFF_WIDTH; ix++) {
    int x = ix * CHAR_WIDTH;
    uint8_t c = text_buff[iy * BUFF_WIDTH + ix];
    uint8_t p = palette_buff[iy * BUFF_WIDTH + ix] % NUM_PALETTES;
#if SHAPONES_MENU_LARGE_FONT
    const uint8_t code_first = FONT12X24_CODE_FIRST;
    const uint8_t code_last = FONT12X24_CODE_LAST;
#else
    const uint8_t code_first = FONT8X16_CODE_FIRST;
    const uint8_t code_last = FONT8X16_CODE_LAST;
#endif
    if (c < code_first || code_last < c) {
      continue;
    }

#if SHAPONES_MENU_LARGE_FONT
    uint32_t font_word =
        FONT12X24_DATA[((int)c - code_first) * CHAR_HEIGHT + (y % CHAR_HEIGHT)];
#else
    uint16_t font_word =
        FONT8X16_DATA[((int)c - code_first) * CHAR_HEIGHT + (y % CHAR_HEIGHT)];
#endif
    for (int ipix = 0; ipix < CHAR_WIDTH; ipix++) {
      int color_index = font_word & 0x03;
      font_word >>= 2;
      line_buff[x + ipix] = PALETTE_FILE[p * 4 + color_index];
    }
  }

  return result_t::SUCCESS;
}

static result_t change_tab(tab_t t, bool force) {
  if (tab == t && !force) return result_t::SUCCESS;

  tab = t;

  switch (tab) {
    case tab_t::NES_LIST: SHAPONES_TRY(refresh_file_list()); break;
  }

  return result_t::SUCCESS;
}

static void clear_file_list() {
  for (int i = 0; i < num_files; i++) {
    if (file_list[i].name) {
      delete[] file_list[i].name;
      file_list[i].name = nullptr;
    }
  }
  num_files = 0;
}

static result_t refresh_file_list() {
  clear_file_list();

  if (!disk_mounted) {
    if (fs_mount() == result_t::SUCCESS) {
      disk_mounted = true;
    } else {
      return result_t::SUCCESS;
    }
  }

  if (current_dir[0] == '\0') {
    SHAPONES_TRY(fs_get_current_dir(current_dir));
    SHAPONES_TRY(append_separator(current_dir));
  }

  if (!is_root_dir(current_dir)) {
    // add parent directory entry
    file_list[0].is_dir = true;
    file_list[0].name = strdup("..");
    num_files = 1;
  }

  result_t res = fs_enum_files(current_dir, [](const file_info_t &info) {
    if (num_files < MAX_NUM_FILES) {
      file_list[num_files] = info;
      file_list[num_files].name = strdup(info.name);
      num_files++;
    }
  });
  if (res != result_t::SUCCESS) {
    num_files = 0;
  }

  list_scroll_y = 0;
  selected_index = 0;

  print_file_list();

  return res;
}

static void scroll_to(int index) {
  if (index < list_scroll_y) {
    list_scroll_y = index;
  } else if (index >= list_scroll_y + CLIENT_HEIGHT) {
    list_scroll_y = index - CLIENT_HEIGHT + 1;
  }
  print_file_list();
}

static void print_file_list() {
  // file list area
  {
    const int TEXT_END_X = CLIENT_X + CLIENT_WIDTH - 1;
    for (int iy = 0; iy < CLIENT_HEIGHT; iy++) {
      int y = iy + CLIENT_Y;
      int i = iy + list_scroll_y;
      fill_text(CLIENT_X, y, CLIENT_WIDTH - 1, 1);
      if (0 <= i && i < num_files) {
        int x = CLIENT_X;
        if (!file_list[i].is_dir) {
          x += print_text(x, y, "\xA8\xA9");  // file icon
        } else if (strcmp(file_list[i].name, "..") == 0) {
          x += print_text(x, y, "\xA4\xA5");  // parent folder icon
        } else {
          x += print_text(x, y, "\xA6\xA7");  // folder icon
        }
        x += 1;
        x += print_text(x, y, file_list[i].name, TEXT_END_X - x);
        if (x > TEXT_END_X - 1) x = TEXT_END_X - 1;
        if (file_list[i].is_dir) {
          print_text(x, y, "/");
        }
      }
      if (i == selected_index) {
        fill_palette(CLIENT_X, y, CLIENT_WIDTH - 1, 1, 1);  // highlight color
      } else {
        fill_palette(CLIENT_X, y, CLIENT_WIDTH - 1, 1, 0);  // normal color
      }
    }
  }
  // scroll bar
  {
    int x = CLIENT_X + CLIENT_WIDTH - 1;
    int y = CLIENT_Y;
    int p = 0;
    if (num_files > CLIENT_HEIGHT) {
      int n = (num_files - CLIENT_HEIGHT);
      p = (list_scroll_y * (CLIENT_HEIGHT - 3) + (n - 1)) / n;
    }
    fill_palette(x, y, CLIENT_HEIGHT, 0);
    print_char(x, y, 0xA0);                      // up arrow
    print_char(x, y + CLIENT_HEIGHT - 1, 0xA1);  // down arrow
    fill_text(x, y + 1, 1, CLIENT_HEIGHT - 2, 0xA2);
    print_char(x, y + 1 + p, 0xA3);  // scroll bar
  }
}

static int print_text(int x, int y, const char *str, int max_len) {
  int n = 0;
  char c;
  while (n < max_len && (c = str[n++]) != '\0') {
    print_char(x++, y, c);
  }
  return n - 1;
}

static void print_char(int x, int y, char c) {
  if (0 <= y && y < BUFF_HEIGHT && 0 <= x && x < BUFF_WIDTH) {
    text_buff[y * BUFF_WIDTH + x] = c;
  }
}

static void fill_text(int x, int y, int w, int h, char c) {
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
    for (int ix = 0; ix < w; ix++) {
      palette_buff[(y + iy) * BUFF_WIDTH + (x + ix)] = p;
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

static bool is_root_dir(const char *path) {
  return find_parent_separator(path) < 0;
}

static int find_parent_separator(const char *path) {
  int n = strnlen(path, nes::MAX_PATH_LENGTH);
  if (n == 0) {
    return -1;
  }
  if (path[n - 1] == '/') {
    n--;
  }
  return find_last_separator(path, n - 1);
}

static int find_last_separator(const char *path, int start_idx) {
  if (start_idx < 0) {
    start_idx = strnlen(path, nes::MAX_PATH_LENGTH);
  }
  for (int i = start_idx - 1; i >= 0; i--) {
    if (path[i] == '/') {
      return i;
    }
  }
  return -1;
}

static result_t append_separator(char *path) {
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  if (path[len - 1] == '/') {
    return result_t::SUCCESS;
  }
  if (len + 1 >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }
  path[len++] = '/';
  path[len] = '\0';
  return result_t::SUCCESS;
}

static result_t append_path(char *path, const char *name) {
  SHAPONES_TRY(append_separator(path));
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  int name_len = strnlen(name, nes::MAX_PATH_LENGTH);
  if (len + name_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  strcat(path, name);
  return result_t::SUCCESS;
}

}  // namespace nes::menu
// #include "shapones/ppu.hpp"

// #include "shapones/cpu.hpp"

// #include "shapones/interrupt.hpp"

// #include "shapones/lock.hpp"

// #include "shapones/mapper.hpp"

// #include "shapones/memory.hpp"

// #include "shapones/menu.hpp"


// #pragma GCC optimize("Ofast")


namespace nes::ppu {

static Registers reg;

volatile cycle_t cycle_count;

static int focus_x;
static int focus_y;

static volatile uint16_t scroll;
static int fine_x;

static uint8_t bus_read_data_latest = 0;
static uint8_t bus_read_data_delayed = 0;

static bool scroll_ppuaddr_high_stored = false;

static bool nmi_level = false;

static uint8_t palette_file[PALETTE_NUM_BANK * PALETTE_SIZE];

static OamEntry oam[MAX_SPRITE_COUNT];
static SpriteLine sprite_lines[MAX_VISIBLE_SPRITES];
static int num_visible_sprites;

// todo: delete
#if 0
static volatile bool mmc3_irq_enable = false;
static volatile bool mmc3_irq_reloading = false;
static volatile uint8_t mmc3_irq_latch = 255;
static volatile uint8_t mmc3_irq_counter = 0;
#endif

static uint8_t bus_read(addr_t addr);
static void bus_write(addr_t addr, uint8_t data);
static uint8_t oam_read(addr_t addr);
static void oam_write(addr_t addr, uint8_t data);
static uint8_t palette_read(addr_t addr);
static void palette_write(addr_t addr, uint8_t data);

static void render_bg(uint8_t *line_buff, int x0, int x1, bool skip_render);
static void enum_visible_sprites(bool skip_render);
static void render_sprite(uint8_t *line_buff, int x0, int x1, bool skip_render);

result_t init() { return result_t::SUCCESS; }
void deinit() {}

result_t reset() {
  // https://www.nesdev.org/wiki/PPU_power_up_state
  reg.control.raw = 0;
  reg.mask.raw = 0;
  reg.oam_addr = 0;
  reg.scroll = 0;
  reg.status.raw = 0;
  reg.fine_x = 0;

  bus_read_data_latest = 0;
  bus_read_data_delayed = 0;

  focus_x = 0;
  focus_y = 0;

  num_visible_sprites = 0;

  scroll_ppuaddr_high_stored = false;

  nmi_level = false;

  cycle_count = 0;

  return result_t::SUCCESS;
}

bool is_in_hblank() { return focus_x >= SCREEN_WIDTH; }

int current_focus_y() { return focus_y; }

uint8_t reg_read(addr_t addr) {
  uint8_t retval;
  switch (addr) {
    case REG_PPUSTATUS: {
      Exclusive lock(LOCK_PPU);
      retval = reg.status.raw;
      reg.status.vblank_flag = 0;
      scroll_ppuaddr_high_stored = false;
    } break;

    case REG_OAMDATA: retval = oam_read(reg.oam_addr); break;

    case REG_PPUDATA: {
      Exclusive lock(LOCK_PPU);
      addr_t addr = scroll & SCROLL_MASK_PPU_ADDR;
      bus_read(addr);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scroll &= ~SCROLL_MASK_PPU_ADDR;
      scroll |= addr;
      retval = bus_read_data_delayed;
    } break;

    default:
      SHAPONES_PRINTF("*Warning: Invalid PPU register read addr: 0x%x\n",
                      (int)addr);
      retval = 0;
      break;
  }
  return retval;
}

void reg_write(addr_t addr, uint8_t data) {
  switch (addr) {
    case REG_PPUCTRL: {
      Exclusive lock(LOCK_PPU);
      // name sel bits
      reg.scroll &= 0xf3ff;
      reg.scroll |= (uint16_t)(data & 0x3) << 10;
      // other bits
      reg.control.raw = data;
    } break;

    case REG_PPUMASK: reg.mask.raw = data; break;

    case REG_OAMADDR: reg.oam_addr = data; break;

    case REG_OAMDATA: oam_write(reg.oam_addr, data); break;

    case REG_PPUSCROLL: {
      Exclusive lock(LOCK_PPU);
      if (!scroll_ppuaddr_high_stored) {
        reg.scroll &= ~SCROLL_MASK_COARSE_X;
        reg.scroll |= (data >> 3) & SCROLL_MASK_COARSE_X;
        reg.fine_x = data & 0x7;
        scroll_ppuaddr_high_stored = true;
      } else {
        reg.scroll &= ~(SCROLL_MASK_COARSE_Y | SCROLL_MASK_FINE_Y);
        reg.scroll |= ((uint16_t)data << 2) & SCROLL_MASK_COARSE_Y;
        reg.scroll |= ((uint16_t)data << 12) & SCROLL_MASK_FINE_Y;
        scroll_ppuaddr_high_stored = false;
      }
    } break;

    case REG_PPUADDR: {
      Exclusive lock(LOCK_PPU);
      if (!scroll_ppuaddr_high_stored) {
        reg.scroll &= 0x00ffu;
        reg.scroll |= ((uint16_t)data << 8) & 0x3f00u;
        scroll_ppuaddr_high_stored = true;
      } else {
        reg.scroll &= 0xff00u;
        reg.scroll |= (uint16_t)data;
        scroll = reg.scroll;
        scroll_ppuaddr_high_stored = false;
      }
    } break;

    case REG_PPUDATA: {
      Exclusive lock(LOCK_PPU);
      uint_fast16_t scr = scroll;
      addr_t addr = scr & SCROLL_MASK_PPU_ADDR;
      bus_write(addr, data);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scr &= ~SCROLL_MASK_PPU_ADDR;
      scr |= addr;
      scroll = scr;
    } break;

    default:
      SHAPONES_PRINTF("*Warning: Invalid PPU register write addr: 0x%x\n",
                      (int)addr);
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
  if (VRAM_BASE <= addr && addr < VRAM_BASE + VRAM_SIZE) {
    memory::vram_write(addr - VRAM_BASE, data);
  } else if (PALETTE_FILE_BASE <= addr &&
             addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
    palette_write(addr - PALETTE_FILE_BASE, data);
  } else if (VRAM_MIRROR_BASE <= addr &&
             addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
    memory::vram_write(addr - VRAM_MIRROR_BASE, data);
  } else {
    // SHAPONES_PRINTF("*Warning: Invalid PPU bus write addr: 0x%x\n",
    // addr);
  }
}

static SHAPONES_INLINE uint8_t oam_read(addr_t addr) {
  switch (addr & 0x3) {
    case 0: return oam[addr / 4].y;
    case 1: return oam[addr / 4].tile;
    case 2: return oam[addr / 4].attr;
    default: return oam[addr / 4].x;
  }
}

static SHAPONES_INLINE void oam_write(addr_t addr, uint8_t data) {
  switch (addr & 0x3) {
    case 0: oam[addr / 4].y = data; break;
    case 1: oam[addr / 4].tile = data; break;
    case 2: oam[addr / 4].attr = data; break;
    default: oam[addr / 4].x = data; break;
  }
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
  bool menu_shown = nes::menu::is_shown();
  timing_t timing = timing_t::NONE;
  bool irq = false;

  while (true) {
    cycle_t cycle_diff = SCREEN_WIDTH;
    if (!menu_shown) {
      cycle_diff = cpu::ppu_cycle_leading() - cycle_count;
      if (cycle_diff <= 0) {
        break;
      }
    }

    // events
    if (focus_x == 0) {
      if (focus_y == SCREEN_HEIGHT + 1) {
        // vblank flag/interrupt
        Exclusive lock(LOCK_PPU);
        reg.status.vblank_flag = 1;
      } else if (focus_y == SCAN_LINES - 1) {
        // clear flags
        Exclusive lock(LOCK_PPU);
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
    int step_count;
    int dist_to_end;
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

    int next_focus_x = focus_x + step_count;

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

    if (focus_y < SCREEN_HEIGHT && !menu_shown) {
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

    if (!menu_shown) {
      // step cycle counter
      cycle_count += step_count;
    }

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

  if (menu_shown) {
    if (!!(timing & timing_t::END_OF_VISIBLE_LINE)) {
      nes::menu::render(focus_y, line_buff);
    }
    if (!!(timing & timing_t::START_OF_VBLANK_LINE)) {
      nes::menu::update();
    }
  }

  if (status) {
    status->focus_y = focus_y;
    status->timing = timing;
  }

  return result_t::SUCCESS;
}

static void render_bg(uint8_t *line_buff, int x0_block, int x1_block,
                      bool skip_render) {
#if !SHAPONES_MUTEX_FAST
  Exclusive lock(LOCK_PPU);
#endif

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
      uint_fast16_t scr;
      {
#if SHAPONES_MUTEX_FAST
        Exclusive lock(LOCK_PPU);
#endif
        scr = scroll;
      }

      // determine step count
      x0 = x0_block;
      if (visible_area && bg_enabled) {
        x1 = x0 + (BLOCK_SIZE - ((scr & 0x1) * TILE_SIZE + fine_x));
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
            int chr_shift_size = ((scr << 4) & 0x10) | (fine_x << 1);
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
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_PPU);
#endif
      uint_fast16_t scr = scroll;
      int fy = focus_y;
      int fx = fine_x;
      if (fy < SCREEN_HEIGHT) {
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
          constexpr uint16_t copy_mask = 0x041fu;
          scr &= ~copy_mask;
          scr |= reg.scroll & copy_mask;
          fx = reg.fine_x;
        }
        fine_x = fx;
      } else if (fy == SCAN_LINES - 1) {
        if (280 <= x1 && x0 <= 304) {
          // vertical recovery
          constexpr uint16_t copy_mask = 0x7be0u;
          scr &= ~copy_mask;
          scr |= reg.scroll & copy_mask;
        }
      }
      scroll = scr;
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
    const auto &s = oam[i];

    // vertical hit test
    int src_y = focus_y - (s.y + SPRITE_Y_OFFSET);
    if (0 <= src_y && src_y < h) {
      int tile_index = 0;
      uint_fast16_t chr = 0xFFFF;

      if (!skip_render) {
        if (s.attr & OAM_ATTR_INVERT_V) {
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
            tile_index = s.tile & 0xfe;
          } else {
            tile_index = s.tile | 0x01;
          }

          if (s.tile & 0x1) {
            tile_index += 0x1000 / 16;
          }
        } else {
          // 8x8 sprite
          tile_index = s.tile;
          if (reg.control.sprite_name_sel) {
            tile_index += 0x1000 / 16;
          }
        }
        // read CHRROM
        int chrrom_index = (tile_index << 4) + (src_y & 0x7);
        chr = memory::chrrom_read_double(chrrom_index,
                                         s.attr & OAM_ATTR_INVERT_H);
      }

      // store sprite information
      SpriteLine &sl = sprite_lines[num_visible_sprites++];
      sl.chr = chr;
      sl.x = s.x;
      sl.palette_offset = (4 + (s.attr & OAM_ATTR_PALETTE)) * PALETTE_SIZE;
      sl.attr = 0;
      if (s.attr & OAM_ATTR_PRIORITY) sl.attr |= SL_ATTR_BEHIND;
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

}  // namespace nes::ppu
// #include "shapones/shapones.hpp"


namespace nes {

Config get_default_config() {
  Config cfg;
  cfg.apu_sampling_rate = 44100;
  return cfg;
}

result_t init(const Config &cfg) {
  for (int i = 0; i < NUM_LOCKS; i++) {
    SHAPONES_TRY(nes::lock_init(i));
  }
  SHAPONES_TRY(memory::init());
  SHAPONES_TRY(mapper::init());
  SHAPONES_TRY(cpu::init());
  SHAPONES_TRY(ppu::init());
  SHAPONES_TRY(apu::init());
  SHAPONES_TRY(menu::init());
  apu::set_sampling_rate(cfg.apu_sampling_rate);
  cpu::stop();
  return result_t::SUCCESS;
}

void deinit() {
  cpu::deinit();
  ppu::deinit();
  apu::deinit();
  mapper::deinit();
  memory::deinit();
  menu::deinit();
  for (int i = 0; i < NUM_LOCKS; i++) {
    nes::lock_deinit(i);
  }
}

result_t map_ines(const uint8_t *ines) {
  SHAPONES_TRY(memory::map_ines(ines));
  SHAPONES_TRY(reset());
  return result_t::SUCCESS;
}

result_t reset() {
  SHAPONES_TRY(cpu::reset());
  SHAPONES_TRY(ppu::reset());
  SHAPONES_TRY(apu::reset());
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

}  // namespace nes
#endif

#endif
