#include "shapones/apu.hpp"
#include "shapones/cpu.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"
#include "shapones/menu.hpp"

namespace nes::apu {

static constexpr int QUARTER_FRAME_FREQUENCY = 240;
static constexpr int QUARTER_FRAME_PHASE_PREC = 16;
static constexpr uint32_t QUARTER_FRAME_PHASE_PERIOD =
    1ul << QUARTER_FRAME_PHASE_PREC;

static uint32_t sampling_rate;
static uint32_t pulse_timer_step;
static uint32_t triangle_timer_step;
static uint32_t noise_timer_step;
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

// see: https://www.nesdev.org/wiki/APU_Noise
static constexpr uint16_t NOISE_PERIOD_TABLE[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
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

static void pulse_write_reg0(PulseState *s, uint8_t value);
static void pulse_write_reg1(PulseState *s, uint8_t value);
static void pulse_write_reg2(PulseState *s, uint8_t value);
static void pulse_write_reg3(PulseState *s, uint8_t value);
static void triangle_write_reg0(TriangleState *s, uint8_t value);
static void triangle_write_reg2(TriangleState *s, uint8_t value);
static void triangle_write_reg3(TriangleState *s, uint8_t value);
static void noise_write_reg0(NoiseState *s, uint8_t value);
static void noise_write_reg2(NoiseState *s, uint8_t value);
static void noise_write_reg3(NoiseState *s, uint8_t value);
static void dmc_write_reg0(DmcState *s, uint8_t value);
static void dmc_write_reg1(DmcState *s, uint8_t value);
static void dmc_write_reg2(DmcState *s, uint8_t value);
static void dmc_write_reg3(DmcState *s, uint8_t value);

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
  triangle.timer = 0;

  noise.timer = 0;
  noise.length = 0;
  noise.lfsr = 0xFFFF;
  noise.mode = false;
  noise.timer_period = NOISE_PERIOD_TABLE[0] * (1u << TIMER_PREC);

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
    case REG_NOISE_REG2: noise_write_reg2(&noise, value); break;
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
        }
      } else {
        dmc.bytes_remaining = 0;
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
  noise_timer_step = cpu::CLOCK_FREQ_NTSC / sampling_rate;
  qframe_phase_step =
      (QUARTER_FRAME_FREQUENCY * QUARTER_FRAME_PHASE_PERIOD) / rate_hz;
  dmc_step_coeff = ((uint64_t)cpu::CLOCK_FREQ_NTSC << TIMER_PREC) / rate_hz;
  return result_t::SUCCESS;
}

static void pulse_write_reg0(PulseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  // duty pattern
  switch ((value >> 6) & 0x3) {
    case 0: s->waveform = 0b00000001; break;
    case 1: s->waveform = 0b00000011; break;
    case 2: s->waveform = 0b00001111; break;
    default: s->waveform = 0b11111100; break;
  }

  s->envelope.flags = 0;
  if (value & 0x10) s->envelope.flags |= ENV_FLAG_CONSTANT;
  if (value & 0x20) s->envelope.flags |= ENV_FLAG_HALT_LOOP;
  s->envelope.volume = value & 0xf;

  if (!(s->envelope.flags & ENV_FLAG_CONSTANT)) {
    s->envelope.flags |= ENV_FLAG_START;
    s->envelope.divider = s->envelope.volume;
  }
}

static void pulse_write_reg1(PulseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->sweep.period = (value >> 4) & 0x7;
  s->sweep.shift = value & 0x7;
  s->sweep.flags = 0;
  if (value & 0x80) s->sweep.flags |= SWP_FLAG_ENABLED;
  if (value & 0x08) s->sweep.flags |= SWP_FLAG_NEGATE;
  s->sweep.divider = 0;
}

static void pulse_write_reg2(PulseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0xff << TIMER_PREC);
  s->timer_period |= (uint32_t)value << TIMER_PREC;
}

static void pulse_write_reg3(PulseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0x700 << TIMER_PREC);
  s->timer_period |= (uint32_t)(value & 0x7) << (TIMER_PREC + 8);
  s->timer = 0;
  s->length = LENGTH_TABLE[(value >> 3) & 0x1f];
  s->phase = 0;
}

static void triangle_write_reg0(TriangleState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->linear.reload_value = value & 0x7f;
  if (value & 0x80) {
    s->linear.flags |= LIN_FLAG_CONTROL;
  } else {
    s->linear.flags &= ~LIN_FLAG_CONTROL;
  }
}

static void triangle_write_reg2(TriangleState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0xff << TIMER_PREC);
  s->timer_period |= (uint32_t)value << TIMER_PREC;
}

static void triangle_write_reg3(TriangleState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->timer_period &= ~(0x700 << TIMER_PREC);
  s->timer_period |= (uint32_t)(value & 0x7) << (TIMER_PREC + 8);
  s->timer = 0;
  s->length = LENGTH_TABLE[(value >> 3) & 0x1f];
  s->phase = 0;
  s->linear.flags |= LIN_FLAG_RELOAD;
}

static void noise_write_reg0(NoiseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->envelope.flags = 0;
  if (value & 0x10) s->envelope.flags |= ENV_FLAG_CONSTANT;
  if (value & 0x20) s->envelope.flags |= ENV_FLAG_HALT_LOOP;
  s->envelope.volume = value & 0xf;

  if (!(s->envelope.flags & ENV_FLAG_CONSTANT)) {
    s->envelope.flags |= ENV_FLAG_START;
    s->envelope.divider = s->envelope.volume;
  }
}

static void noise_write_reg2(NoiseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->mode = !!(value & 0x80);
  s->timer_period = NOISE_PERIOD_TABLE[value & 0xF];
}

static void noise_write_reg3(NoiseState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->length = LENGTH_TABLE[(value >> 3) & 0x1f];
}

static void dmc_write_reg0(DmcState *s, uint8_t value) {
  bool irq_ena_old = s->irq_enabled;
  bool irq_ena_new = (value & 0x80) != 0;
  if (irq_ena_new && !irq_ena_old) {
    interrupt::deassert_irq(interrupt::Source::APU_DMC);
  }
  {
    Exclusive lock(LOCK_APU);
    s->irq_enabled = irq_ena_new;
    s->loop = (value & 0x40) != 0;
    s->timer_step = dmc_step_coeff / DMC_RATE_TABLE[value & 0x0f];
  }
}

static void dmc_write_reg1(DmcState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->out_level = value & 0x7f;
}

static void dmc_write_reg2(DmcState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->sample_addr = 0xc000 + ((addr_t)value << 6);
}

static void dmc_write_reg3(DmcState *s, uint8_t value) {
  Exclusive lock(LOCK_APU);
  s->sample_length = ((int)value << 4) + 1;
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
    if (s.mode) {
      fb = ((s.lfsr >> 6) ^ (s.lfsr >> 0)) & 1;
    } else {
      fb = ((s.lfsr >> 1) ^ (s.lfsr >> 0)) & 1;
    }
    s.lfsr = (s.lfsr >> 1) | (fb << 14);
  }
  return ((s.lfsr & 1) == 0) ? (vol >> 1) : 0;
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
#endif
      if (s.bytes_remaining > 0) {
        s.shift_reg = cpu::bus_read(s.addr_counter | 0x8000);
        s.addr_counter++;
        s.bytes_remaining--;
        if (s.bytes_remaining == 0) {
          if (s.loop) {
            s.addr_counter = s.sample_addr;
            s.bytes_remaining = s.sample_length;
          } else {
            status.dmc_enable = 0;
            if (s.irq_enabled) {
              interrupt::assert_irq(interrupt::Source::APU_DMC);
            }
          }
        }
      }
      s.bits_remaining = 8;
    }
    if (s.bits_remaining > 0) {
      if (status.dmc_enable) {
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
