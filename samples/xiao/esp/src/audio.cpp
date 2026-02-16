#include "shapones/xiao/audio.hpp"
#include "shapones/apu.hpp"
#include "shapones/xiao/pins.h"

#include <driver/i2s_pdm.h>

namespace shapones::xiao::audio {

static constexpr uint32_t AUDIO_I2S_FREQ_HZ = 96000;
static constexpr uint32_t AUDIO_SAMPLE_FREQ_HZ = AUDIO_I2S_FREQ_HZ / 6;
static constexpr int AUDIO_BUFF_LEN = 256;

static i2s_chan_handle_t audio_i2s_ch;
static uint8_t apu_out_buff[AUDIO_BUFF_LEN] = {0};
static int16_t audio_buff[AUDIO_BUFF_LEN] = {0};
static constexpr uint32_t SAMPLE_SIZE = sizeof(audio_buff[0]);
static uint32_t audio_wr_ptr = 0;
static uint32_t audio_rd_ptr = 0;

static void fill_buffer(bool preload);

uint32_t get_sampling_rate_hz() {
  return AUDIO_SAMPLE_FREQ_HZ;
}

void init() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_i2s_ch, nullptr));

  i2s_pdm_tx_clk_config_t clk_cfg =
      I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG(AUDIO_SAMPLE_FREQ_HZ);
  i2s_pdm_tx_slot_config_t slot_cfg = I2S_PDM_TX_SLOT_DAC_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  i2s_pdm_tx_config_t tx_cfg{
      .clk_cfg = clk_cfg,
      .slot_cfg = slot_cfg,
      .gpio_cfg =
          {
              .clk = I2S_GPIO_UNUSED,
              .dout = (gpio_num_t)XIAO_AUDIO_OUT_PIN,
              .dout2 = I2S_GPIO_UNUSED,
              .invert_flags =
                  {
                      .clk_inv = false,
                  },
          },
  };
  ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(audio_i2s_ch, &tx_cfg));

  fill_buffer(true);
  ESP_ERROR_CHECK(i2s_channel_enable(audio_i2s_ch));
}

void deinit() {}

void stream() { fill_buffer(false); }

static void fill_buffer(bool preload) {
  uint32_t buff_free = (audio_rd_ptr - audio_wr_ptr) & (AUDIO_BUFF_LEN - 1);
  if (buff_free == 0) {
    buff_free = AUDIO_BUFF_LEN;
  }
  if (buff_free > 1) {
    buff_free--;
    shapones::apu::service(apu_out_buff, buff_free);
    for (int i = 0; i < buff_free; i++) {
      int16_t val = (int16_t)apu_out_buff[i] - 128;
      audio_buff[audio_wr_ptr] = val * 256;
      audio_wr_ptr = (audio_wr_ptr + 1) & (AUDIO_BUFF_LEN - 1);
    }
  }

  if (audio_wr_ptr < audio_rd_ptr) {
    size_t to_write = (AUDIO_BUFF_LEN - audio_rd_ptr) * SAMPLE_SIZE;
    size_t written = 0;
    if (preload) {
      i2s_channel_preload_data(audio_i2s_ch, &audio_buff[audio_rd_ptr],
                               to_write, &written);
    } else {
      i2s_channel_write(audio_i2s_ch, &audio_buff[audio_rd_ptr], to_write,
                        &written, 0);
    }
    audio_rd_ptr =
        (audio_rd_ptr + written / SAMPLE_SIZE) & (AUDIO_BUFF_LEN - 1);
  }
  if (audio_rd_ptr < audio_wr_ptr) {
    size_t to_write = (audio_wr_ptr - audio_rd_ptr) * SAMPLE_SIZE;
    size_t written = 0;
    if (preload) {
      i2s_channel_preload_data(audio_i2s_ch, &audio_buff[audio_rd_ptr],
                               to_write, &written);
    } else {
      i2s_channel_write(audio_i2s_ch, &audio_buff[audio_rd_ptr], to_write,
                        &written, 0);
    }
    audio_rd_ptr =
        (audio_rd_ptr + written / SAMPLE_SIZE) & (AUDIO_BUFF_LEN - 1);
  }
}

}  // namespace shapones::xiao::audio
