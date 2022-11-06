#ifndef SHAPONES_APU_HPP
#define SHAPONES_APU_HPP

#include "shapones/shapones.hpp"

namespace nes::apu {

static constexpr int STEP_COEFF_PREC = 8;
static constexpr int PULSE_PHASE_PREC = 24;
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
    uint8_t reg2;
    uint8_t waveform;
    uint32_t timer;
    uint32_t timer_period;
    uint32_t phase;
    int length;
    Envelope envelope;
    Sweep sweep;
};

struct TriangleState {
    uint8_t reg2;
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

void reset();
uint8_t reg_read(addr_t addr);
void reg_write(addr_t addr, uint8_t value);
void set_sampling_rate(int rate_hz);
void service(uint8_t *buff, int len);

}

#endif
