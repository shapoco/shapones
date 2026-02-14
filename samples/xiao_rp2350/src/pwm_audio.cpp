#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "pwm_audio.hpp"

namespace shapones::xiao::rp::pwm_audio {

static const dma_channel_transfer_size DMA_WIDTH =
    sizeof(sample_t) == 1   ? DMA_SIZE_8
    : sizeof(sample_t) == 2 ? DMA_SIZE_16
                            : DMA_SIZE_32;

static constexpr int SAMPLE_BITS = 8;
static constexpr uint32_t SAMPLE_FREQ = 22050;

static fill_buffer_cb_t dma_handler;
static int pwm_slice;
static int pwm_channel;

static sample_t buff[LATENCY * 2];
static int dma_ch;
static int playing_bank;

static void start_dma();

void init(uint32_t sys_clk_freq, fill_buffer_cb_t cb) {
  dma_handler = cb;

  gpio_set_function(PWM_OUT_PIN, GPIO_FUNC_PWM);
  gpio_set_dir(PWM_OUT_PIN, GPIO_OUT);

  constexpr int PWM_PERIOD = 1 << SAMPLE_BITS;
  float pwm_clkdiv = (float)sys_clk_freq / (SAMPLE_FREQ * PWM_PERIOD);
  pwm_slice = pwm_gpio_to_slice_num(PWM_OUT_PIN);
  pwm_channel = pwm_gpio_to_channel(PWM_OUT_PIN);

  pwm_config pwm_cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&pwm_cfg, pwm_clkdiv);
  pwm_config_set_wrap(&pwm_cfg, PWM_PERIOD - 1);
  pwm_init(pwm_slice, &pwm_cfg, true);

  pwm_set_gpio_level(PWM_OUT_PIN, 0);
}

void deinit() {
  // drain charge from pin
  gpio_init(PWM_OUT_PIN);
  gpio_set_dir(PWM_OUT_PIN, GPIO_OUT);
  gpio_put(PWM_OUT_PIN, 0);
  // disable pin
  gpio_deinit(PWM_OUT_PIN);
}

void play() {
  dma_ch = dma_claim_unused_channel(true);
  dma_channel_set_irq0_enabled(dma_ch, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  playing_bank = 0;
  start_dma();
}

void stop() {
  dma_channel_unclaim(dma_ch);
  pwm_set_gpio_level(PWM_OUT_PIN, 1 << (SAMPLE_BITS - 1));
}

void flip_buffer() {
  int fill_bank = playing_bank;
  playing_bank = (playing_bank + 1) & 1;
  start_dma();
  dma_hw->ints0 = (1u << dma_ch);
}

sample_t* get_buffer(int bank) { return buff + (bank * LATENCY); }

sample_t* get_next_buffer() {
  int next_bank = (playing_bank + 1) & 1;
  return get_buffer(next_bank);
}

static void start_dma() {
  dma_channel_config dma_cfg = dma_channel_get_default_config(dma_ch);
  channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_cfg, true);
  channel_config_set_write_increment(&dma_cfg, false);
  channel_config_set_dreq(&dma_cfg, DREQ_PWM_WRAP0 + pwm_slice);
  dma_channel_configure(dma_ch, &dma_cfg,
                        &pwm_hw->slice[pwm_slice].cc,     // write addr
                        buff + (playing_bank * LATENCY),  // read addr
                        LATENCY,                          // number of data
                        true                              // start immediately
  );
}

}  // namespace shapones::xiao::rp::pwm_audio