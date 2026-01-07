#ifndef SHAPONES_APU_HPP
#define SHAPONES_APU_HPP

#include "shapones/shapones.hpp"

namespace nes::apu {

static constexpr int TIMER_PREC = 16;

static constexpr addr_t REG_PULSE1_REG0     = 0x4000;
static constexpr addr_t REG_PULSE1_REG1     = 0x4001;
static constexpr addr_t REG_PULSE1_REG2     = 0x4002;
static constexpr addr_t REG_PULSE1_REG3     = 0x4003;
static constexpr addr_t REG_PULSE2_REG0     = 0x4004;
static constexpr addr_t REG_PULSE2_REG1     = 0x4005;
static constexpr addr_t REG_PULSE2_REG2     = 0x4006;
static constexpr addr_t REG_PULSE2_REG3     = 0x4007;
static constexpr addr_t REG_TRIANGLE_REG0   = 0x4008;
static constexpr addr_t REG_TRIANGLE_REG2   = 0x400a;
static constexpr addr_t REG_TRIANGLE_REG3   = 0x400b;
static constexpr addr_t REG_NOISE_REG0      = 0x400c;
static constexpr addr_t REG_NOISE_REG2      = 0x400e;
static constexpr addr_t REG_NOISE_REG3      = 0x400f;
static constexpr addr_t REG_DMC_REG0        = 0x4010;
static constexpr addr_t REG_DMC_REG1        = 0x4011;
static constexpr addr_t REG_DMC_REG2        = 0x4012;
static constexpr addr_t REG_DMC_REG3        = 0x4013;
static constexpr addr_t REG_STATUS          = 0x4015;

static constexpr int ENV_FLAG_START     = 0x1;
static constexpr int ENV_FLAG_CONSTANT  = 0x2;
static constexpr int ENV_FLAG_HALT_LOOP = 0x4;

static constexpr int SWP_FLAG_ENABLED   = 0x1;
static constexpr int SWP_FLAG_NEGATE    = 0x2;

static constexpr int LIN_FLAG_RELOAD    = 0x1;
static constexpr int LIN_FLAG_CONTROL   = 0x2;

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
    bool flags;
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
        uint8_t frame_interrupt : 1;
        uint8_t dmc_interrupt : 1;
    };
};

void reset();
uint8_t reg_read(addr_t addr);
void reg_write(addr_t addr, uint8_t value);
void set_sampling_rate(int rate_hz);
void service(uint8_t *buff, int len);

}

#endif
