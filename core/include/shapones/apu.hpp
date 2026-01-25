#ifndef SHAPONES_APU_HPP
#define SHAPONES_APU_HPP

#include "shapones/common.hpp"

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
    BufferReader reader(buff);
    timer = reader.u32();
    timer_period = reader.u32();
    phase = reader.u32();
    length = reader.u8();
    waveform = reader.u8();
    buff += 32;
    envelope.load(buff);
    buff += envelope_t::STATE_SIZE;
    sweep.load(buff);
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
    BufferReader reader(buff);
    timer = reader.u32();
    timer_period = reader.u32();
    phase = reader.u32();
    length = reader.u8();
    buff += 24;
    linear.load(buff);
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
    BufferReader reader(buff);
    timer = reader.u32();
    timer_period = reader.u32();
    lfsr = reader.u16();
    flags = reader.u8();
    length = reader.u8();
    buff += 24;
    envelope.load(buff);
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
    writer.u32(sample_addr);
    writer.u32(addr_counter);
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
    sample_addr = reader.u32();
    addr_counter = reader.u32();
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
