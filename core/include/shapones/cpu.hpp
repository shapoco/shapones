#ifndef SHAPONES_CPU_HPP
#define SHAPONES_CPU_HPP

#include "shapones/common.hpp"

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
