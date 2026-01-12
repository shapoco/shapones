#include "shapones/apu.hpp"
#include "shapones/cpu.hpp"
#include "shapones/interrupt.hpp"

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

PulseState pulse[2];
TriangleState triangle;
NoiseState noise;
DmcState dmc;
Status status;

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
static int step_envelope(Envelope *e);
static int step_sweep(PulseState *s);
static int sample_pulse(PulseState *s);
static int sample_triangle(TriangleState *s);
static int sample_noise(NoiseState *s);
static int sample_dmc(DmcState *s);

void reset() {
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

void set_sampling_rate(int rate_hz) {
    sampling_rate = rate_hz;
    pulse_timer_step =
        (1ULL << TIMER_PREC) * cpu::CLOCK_FREQUENCY / sampling_rate / 2;
    triangle_timer_step =
        (1ULL << TIMER_PREC) * cpu::CLOCK_FREQUENCY / sampling_rate;
    qframe_phase_step =
        (QUARTER_FRAME_FREQUENCY * QUARTER_FRAME_PHASE_PERIOD) / rate_hz;
    dmc_step_coeff = ((uint64_t)cpu::CLOCK_FREQUENCY << TIMER_PREC) / rate_hz;
}

void service(uint8_t *buff, int len) {
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
        sample += sample_pulse(&pulse[0]);
        sample += sample_pulse(&pulse[1]);
        sample += sample_triangle(&triangle);
        sample += sample_noise(&noise);
        sample *= 2;
        sample += sample_dmc(&dmc);
        buff[i] = sample;
    }
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
static int step_envelope(Envelope *e) {
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
static int step_sweep(PulseState *s) {
    int gate = 1;
    if (s->sweep.flags & SWP_FLAG_ENABLED) {
        if (half_frame_step) {
            if (s->sweep.divider < s->sweep.period) {
                s->sweep.divider++;
            } else {
                s->sweep.divider = 0;
                int change_amount = s->timer_period;
                change_amount >>= s->sweep.shift;
                if (s->sweep.flags & SWP_FLAG_NEGATE) {
                    change_amount = -change_amount;
                }
                int target = s->timer_period + change_amount;
                if (target < 0) {
                    target = 0;
                } else if (target >= (0x7ff << TIMER_PREC)) {
                    target = (0x7ff << TIMER_PREC);
                    gate = 0;
                }
                {
                    Exclusive lock(LOCK_APU);
                    s->timer_period = target;
                }
            }
        }
    }
    return gate;
}

// see: https://www.nesdev.org/wiki/APU_Triangle
static int step_linear_counter(LinearCounter *c) {
    // linear counter
    if (quarter_frame_step) {
        if (c->flags & LIN_FLAG_RELOAD) {
            c->counter = c->reload_value;
        } else if (c->counter > 0) {
            c->counter--;
        }

        if (!(c->flags & LIN_FLAG_CONTROL)) {
            Exclusive lock(LOCK_APU);
            c->flags &= ~LIN_FLAG_RELOAD;  // clear reload flag
        }
    }
    return c->counter > 0 ? 1 : 0;
}

// see: https://www.nesdev.org/wiki/APU_Pulse
static int sample_pulse(PulseState *s) {
    // envelope unit
    int vol = step_envelope(&(s->envelope));

    // sweep unit
    vol *= step_sweep(s);

    // length counter
    if (half_frame_step && !(s->envelope.flags & ENV_FLAG_HALT_LOOP)) {
        if (s->length >= 0) {
            Exclusive lock(LOCK_APU);
            s->length--;
        }
    }

    // mute
    if (s->timer_period < 8) vol = 0;
    if (s->length < 0) vol = 0;

    // sequencer
    uint32_t phase;
    {
        Exclusive lock(LOCK_APU);
        s->timer += pulse_timer_step;
        int step = 0;
        if (s->timer_period != 0) {
            step = s->timer / s->timer_period;
        }
        s->timer -= step * s->timer_period;
        s->phase = (s->phase + step) & 0x7;
        phase = s->phase;
    }

    int amp = (s->waveform >> phase) & 1;
    return amp * vol;
}

// see: https://www.nesdev.org/wiki/APU_Triangle
static int sample_triangle(TriangleState *s) {
    int vol = 1;

    // length counter
    if (half_frame_step && !(s->linear.flags & LIN_FLAG_CONTROL)) {
        if (s->length >= 0) {
            Exclusive lock(LOCK_APU);
            s->length--;
        }
    }

    // linear counter
    step_linear_counter(&(s->linear));

    // phase counter
    uint32_t phase;
    {
        Exclusive lock(LOCK_APU);
        s->timer += triangle_timer_step;
        int step = 0;
        if (s->timer_period != 0) {
            step = s->timer / s->timer_period;
        }
        s->timer -= step * s->timer_period;
        if (s->length > 0 && s->linear.counter > 0) {
            s->phase = (s->phase + step) & 0x1f;
        }
        phase = s->phase;
    }

    // mute
    if (s->timer_period < 8) vol = 0;

    // sequencer
    if (phase <= 15) {
        return (15 - phase) * vol;
    } else {
        return (phase - 16) * vol;
    }
}

// see: https://www.nesdev.org/wiki/APU_Noise
static int sample_noise(NoiseState *s) {
    // envelope unit
    int vol = step_envelope(&(s->envelope));

    // length counter
    if (half_frame_step && !(s->envelope.flags & ENV_FLAG_HALT_LOOP)) {
        if (s->length >= 0) {
            Exclusive lock(LOCK_APU);
            s->length--;
        }
    }

    // mute
    if (s->length < 0) vol = 0;

    // LFSR
    int feedback = ((s->lfsr >> 1) ^ s->lfsr) & 1;
    s->lfsr = ((s->lfsr >> 1) & 0x3fff) | (feedback << 14);
    int amp = s->lfsr & 0xf;

    return (amp * vol) >> 4;
}

// see: https://www.nesdev.org/wiki/APU_DMC
static int sample_dmc(DmcState *s) {
    int step;
    {
        Exclusive lock(LOCK_APU);
        s->timer += s->timer_step;
        step = s->timer >> TIMER_PREC;
        s->timer &= (1 << TIMER_PREC) - 1;
    }

    for (int i = 0; i < step; i++) {
        if (s->bits_remaining == 0) {
            Exclusive lock(LOCK_APU);;
            if (s->bytes_remaining > 0) {
                s->shift_reg = cpu::bus_read(s->addr_counter | 0x8000);
                s->addr_counter++;
                s->bytes_remaining--;
                if (s->bytes_remaining == 0) {
                    if (s->loop) {
                        s->addr_counter = s->sample_addr;
                        s->bytes_remaining = s->sample_length;
                    } else if (s->irq_enabled) {
                        interrupt::assert_irq(interrupt::Source::APU_DMC);
                    }
                }
            }
            s->bits_remaining = 8;
        }
        if (s->bits_remaining > 0) {
            if (!s->silence) {
                if (s->shift_reg & 1) {
                    Exclusive lock(LOCK_APU);
                    if (s->out_level <= 125) {
                        s->out_level += 2;
                    }
                } else {
                    Exclusive lock(LOCK_APU);
                    if (s->out_level >= 2) {
                        s->out_level -= 2;
                    }
                }
                s->shift_reg >>= 1;
            }
            s->bits_remaining--;
        }
    }

    return s->out_level;
}

}  // namespace nes::apu
