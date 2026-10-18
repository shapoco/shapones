// NES sound out of both PicoCalc speakers: PWM on GPIO 26 (left) and 27
// (right), which are the two channels of PWM slice 5, fed by DMA from a
// double buffer that the APU refills in the DMA interrupt (the XIAO port's
// scheme, with both channels driven).
#include "audio.hpp"
#include "shapones/apu.hpp"

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

namespace picocalc::audio {

static constexpr int PIN_LEFT = 26, PIN_RIGHT = 27;
static constexpr uint32_t SAMPLE_HZ = 22050;
static constexpr int BUFF_LEN = 256;

static int slice, dma_ch;
static int write_bank, read_bank;
static volatile bool muted = false;

static uint8_t apu_buff[BUFF_LEN];
static uint32_t dma_buff[BUFF_LEN * 2];

static void fill_buffer() {
  shapones::apu::service(apu_buff, BUFF_LEN);
  uint32_t *out = dma_buff + write_bank * BUFF_LEN;
  for (int i = 0; i < BUFF_LEN; i++) {
    uint32_t v = muted ? 0x80 : apu_buff[i];
    out[i] = v | (v << 16);  // channel A (left) and B (right)
  }
  write_bank ^= 1;
}

static void start_dma() {
  dma_channel_set_read_addr(dma_ch, dma_buff + read_bank * BUFF_LEN, true);
  read_bank ^= 1;
}

static void dma_handler() {
  if (!(dma_hw->ints1 & (1u << dma_ch))) return;
  dma_hw->ints1 = 1u << dma_ch;
  start_dma();
  fill_buffer();
}

uint32_t sampling_rate_hz() { return SAMPLE_HZ; }

void init() {
  gpio_set_function(PIN_LEFT, GPIO_FUNC_PWM);
  gpio_set_function(PIN_RIGHT, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(PIN_LEFT);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (SAMPLE_HZ * 256));
  pwm_config_set_wrap(&cfg, 255);
  pwm_init(slice, &cfg, true);
  pwm_set_both_levels(slice, 0x80, 0x80);

  dma_ch = dma_claim_unused_channel(true);
  dma_channel_set_irq1_enabled(dma_ch, true);
  irq_add_shared_handler(DMA_IRQ_1, dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_1, true);

  dma_channel_config dc = dma_channel_get_default_config(dma_ch);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_read_increment(&dc, true);
  channel_config_set_write_increment(&dc, false);
  channel_config_set_dreq(&dc, pwm_get_dreq(slice));
  dma_channel_configure(dma_ch, &dc, &pwm_hw->slice[slice].cc, dma_buff,
                        BUFF_LEN, false);

  write_bank = read_bank = 0;
  fill_buffer();
  fill_buffer();
  start_dma();
}

void set_muted(bool m) { muted = m; }

}  // namespace picocalc::audio
