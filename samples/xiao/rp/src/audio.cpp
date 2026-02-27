#include "shapones/xiao/audio.hpp"
#include "shapones/apu.hpp"
#include "shapones/xiao/pins.h"

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

namespace shapones::xiao::audio {

static constexpr uint32_t AUDIO_SAMPLE_FREQ_HZ = 22050;
static constexpr int AUDIO_BUFF_LEN = 256;
static constexpr int SAMPLE_BITS = 8;

static int pwm_slice;
static int pwm_channel;
static int dma_ch;
static int next_write_bank;
static int next_read_bank;

static uint8_t apu_out_buff[AUDIO_BUFF_LEN] = {0};
static uint32_t audio_dma_buff[AUDIO_BUFF_LEN * 2] = {0};

static void dma_handler();
static void fill_buffer();
static void start_dma();

uint32_t get_sampling_rate_hz() { return AUDIO_SAMPLE_FREQ_HZ; }

void init() {
  gpio_set_function(XIAO_AUDIO_OUT_PIN, GPIO_FUNC_PWM);
  gpio_set_dir(XIAO_AUDIO_OUT_PIN, GPIO_OUT);

  constexpr int PWM_PERIOD = 1 << SAMPLE_BITS;
  int sys_clk_freq = clock_get_hz(clk_sys);
  float pwm_clkdiv = (float)sys_clk_freq / (AUDIO_SAMPLE_FREQ_HZ * PWM_PERIOD);
  pwm_slice = pwm_gpio_to_slice_num(XIAO_AUDIO_OUT_PIN);
  pwm_channel = pwm_gpio_to_channel(XIAO_AUDIO_OUT_PIN);
  pwm_set_gpio_level(XIAO_AUDIO_OUT_PIN, 0);

  pwm_config pwm_cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&pwm_cfg, pwm_clkdiv);
  pwm_config_set_wrap(&pwm_cfg, PWM_PERIOD - 1);
  pwm_init(pwm_slice, &pwm_cfg, true);

  dma_ch = dma_claim_unused_channel(true);
  dma_channel_set_irq1_enabled(dma_ch, true);
  irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
  irq_set_enabled(DMA_IRQ_1, true);

  dma_channel_config dma_cfg = dma_channel_get_default_config(dma_ch);
  channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_cfg, true);
  channel_config_set_write_increment(&dma_cfg, false);
  channel_config_set_dreq(&dma_cfg, DREQ_PWM_WRAP0 + pwm_slice);
  dma_channel_configure(dma_ch, &dma_cfg, &pwm_hw->slice[pwm_slice].cc,
                        audio_dma_buff, AUDIO_BUFF_LEN, false);

  next_write_bank = 0;
  next_read_bank = 0;

  fill_buffer();
  fill_buffer();
  start_dma();
}

void deinit() {
  // drain charge from pin
  gpio_init(XIAO_AUDIO_OUT_PIN);
  gpio_set_dir(XIAO_AUDIO_OUT_PIN, GPIO_OUT);
  gpio_put(XIAO_AUDIO_OUT_PIN, 0);
  // disable pin
  gpio_deinit(XIAO_AUDIO_OUT_PIN);
}

void stream() {}

static void dma_handler() {
  dma_hw->ints1 = 1u << dma_ch;
  start_dma();
  fill_buffer();
}

static void fill_buffer() {
  apu::service(apu_out_buff, AUDIO_BUFF_LEN);
  int buff_offset = next_write_bank * AUDIO_BUFF_LEN;
  for (int i = 0; i < AUDIO_BUFF_LEN; i++) {
    audio_dma_buff[buff_offset + i] = (uint32_t)apu_out_buff[i] << (pwm_channel * 16);
  }
  next_write_bank = (next_write_bank + 1) & 1;
}

static void start_dma() {
  dma_channel_set_read_addr(
      dma_ch, audio_dma_buff + (next_read_bank * AUDIO_BUFF_LEN), true);
  next_read_bank = (next_read_bank + 1) & 1;
}

}  // namespace shapones::xiao::audio
