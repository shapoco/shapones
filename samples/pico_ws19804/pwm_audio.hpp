#ifndef PWM_AUDIO_HPP
#define PWM_AUDIO_HPP

#include "stdint.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

using PwmAudioCallback = std::function<void(uint32_t *buff, int length)>;

class PwmAudio;
static PwmAudio *inst;

class PwmAudio {
public:
    using sample_t = uint32_t;

    const dma_channel_transfer_size DMA_WIDTH =
        sizeof(sample_t) == 1 ? DMA_SIZE_8 :
        sizeof(sample_t) == 2 ? DMA_SIZE_16 :
                                DMA_SIZE_32;
    
    const int PIN;
    const int SLICE_NUM;
    const int SAMPLE_BITS;
    const float FREQUENCY;
    const int LATENCY;
    PwmAudioCallback callback;

    sample_t *buff;
    int dma_ch;
    int playing_bank;

    PwmAudio(int pin, int latency, int sample_bits, float freq_ratio, PwmAudioCallback callback) :
        PIN(pin),
        SLICE_NUM(pwm_gpio_to_slice_num(pin)),
        SAMPLE_BITS(sample_bits),
        FREQUENCY(freq_ratio),
        LATENCY(latency),
        buff(new sample_t[latency]),
        callback(callback)
    {
        gpio_set_function(PIN, GPIO_FUNC_PWM);

        int pwm_period = 1 << SAMPLE_BITS;
        float pwm_clkdiv = freq_ratio / pwm_period;

        pwm_config pwm_cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&pwm_cfg, pwm_clkdiv);
        pwm_config_set_wrap(&pwm_cfg, pwm_period - 1);
        pwm_init(SLICE_NUM, &pwm_cfg, true);

        pwm_set_gpio_level(PIN, 1 << (SAMPLE_BITS-1));

        inst = this;
    }

    void play() {
        dma_ch = dma_claim_unused_channel(true);
        dma_channel_set_irq0_enabled(dma_ch, true);
        irq_set_exclusive_handler(DMA_IRQ_0, &PwmAudio::dma_finished_handler);
        irq_set_enabled(DMA_IRQ_0, true);

        fill_buffer(0);
        fill_buffer(1);
        playing_bank = 0;
        start_dma();
    }
    
    void stop() {
        dma_channel_unclaim(dma_ch);
        pwm_set_gpio_level(PIN, 1 << (SAMPLE_BITS-1));
    }

    void fill_buffer(int bank) {
        callback(buff + (bank * LATENCY), LATENCY);
    }

    void start_dma() {
        dma_channel_config dma_cfg = dma_channel_get_default_config(dma_ch);
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&dma_cfg, true);
        channel_config_set_write_increment(&dma_cfg, false);
        channel_config_set_dreq(&dma_cfg, DREQ_PWM_WRAP0 + SLICE_NUM);
        dma_channel_configure(
            dma_ch,
            &dma_cfg,
            &pwm_hw->slice[SLICE_NUM].cc, // write addr
            buff + (playing_bank * LATENCY), // read addr
            LATENCY, // number of data
            true // start immediately
        );
    }

    void dma_finished() {
        int fill_bank = playing_bank;
        playing_bank = (playing_bank + 1) & 1;
        inst->start_dma();
        dma_hw->ints0 = (1u << dma_ch);
        callback(buff + (fill_bank * LATENCY), LATENCY);
    }

    static void dma_finished_handler() {
        inst->dma_finished();
    }
};

#endif
