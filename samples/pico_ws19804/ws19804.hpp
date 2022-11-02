#pragma once

#include "stdint.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "ws19804.pio.h"

#include <algorithm>

class WS19804 {
public:
    static constexpr int WIDTH = 320;
    static constexpr int HEIGHT = 240;
    static constexpr uint PIN_DC = 8;
    static constexpr uint PIN_CS = 9;
    static constexpr uint PIN_SCK = 10;
    static constexpr uint PIN_DIN = 11;
    static constexpr uint PIN_RST = 12;
    static constexpr uint PIN_BL = 13;
    
    PIO spi_pio;
    uint spi_sm;
    uint spi_offset;
    uint dma_tx;
    
    // Parallel init
    WS19804()
    {
        gpio_init(PIN_RST);
        gpio_init(PIN_DC);
        gpio_init(PIN_BL);
        gpio_init(PIN_CS);
        gpio_set_dir(PIN_RST, GPIO_OUT);
        gpio_set_dir(PIN_DC, GPIO_OUT);
        gpio_set_dir(PIN_BL, GPIO_OUT);
        gpio_set_dir(PIN_CS, GPIO_OUT);
        gpio_put(PIN_RST, 1);
        gpio_put(PIN_CS, 1);
        gpio_put(PIN_BL, 0);
    }
    
    void init() {
        spi_pio = pio0;
        spi_sm = 0;
        spi_offset = pio_add_program(spi_pio, &ws19804_program);
        
        pio_gpio_init(spi_pio, PIN_DIN);
        pio_gpio_init(spi_pio, PIN_SCK);
        pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, PIN_DIN, 1, true);
        pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, PIN_SCK, 1, true);
        pio_sm_config pio_cfg = ws19804_program_get_default_config(spi_offset);
        sm_config_set_sideset_pins(&pio_cfg, PIN_SCK);
        sm_config_set_out_pins(&pio_cfg, PIN_DIN, 1);
        sm_config_set_fifo_join(&pio_cfg, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&pio_cfg, 2.0f); // 250MHz / 2 / 2 --> 62.5 MHz
        sm_config_set_out_shift(&pio_cfg, false, true, 8);
        pio_sm_init(spi_pio, spi_sm, spi_offset, &pio_cfg);
        pio_sm_set_enabled(spi_pio, spi_sm, true);

        dma_tx = dma_claim_unused_channel(true);
        
        // hardware reset
        gpio_put(PIN_RST, 1);
        sleep_ms(1);
        gpio_put(PIN_RST, 0);
        sleep_ms(10);
        gpio_put(PIN_RST, 1);
        sleep_ms(10);
        
        write_command(0x11);
        sleep_ms(120);

        //MX, MY, RGB mode
        {
            uint8_t data[] = { 0xa0 };
            write_command(0x36, data, sizeof(data));
        }

        {
            //uint8_t data[] = { 0x05 }; // RGB565
            uint8_t data[] = { 0x03 }; // RGB444
            write_command(0x3a, data, sizeof(data));
        }
        
        {
            uint8_t data[] = { 0x0c, 0x0c, 0x00, 0x33, 0x33 };
            write_command(0xb2, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x72 };
            write_command(0xb7, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x3d };
            write_command(0xbb, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x01 };
            write_command(0xc2, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x19 };
            write_command(0xc3, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x20 };
            write_command(0xc4, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x0f };
            write_command(0xc6, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0xa4, 0xa1 };
            write_command(0xd0, data, sizeof(data));
        }
        
        {
            uint8_t data[] = {
                0x70, 0x04, 0x08, 0x09, 0x09, 0x05, 0x2a, 0x33,
                0x41, 0x07, 0x13, 0x13, 0x29, 0x2f,
            };
            write_command(0xe0, data, sizeof(data));
        }
        {
            uint8_t data[] = {
                0x70, 0x03, 0x09, 0x0a, 0x09, 0x06, 0x2b, 0x34,
                0x41, 0x07, 0x12, 0x14, 0x28, 0x2e,
            };
            write_command(0xe1, data, sizeof(data));
        }

        write_command(0x21);
        write_command(0x29);

        gpio_put(PIN_BL, 1);
    }

    void clear(uint16_t color) {
        uint8_t data[WIDTH * 3 / 2];
        for (int x = 0; x < WIDTH * 3 / 2; x++) {
            data[x] = color;
        }
        for (int y = 0; y < HEIGHT; y++) {
            start_write_data(0, y, WIDTH, 1, data);
            finish_write_data();
        }
    }

    void start_write_data(int x0, int y0, int w, int h, uint8_t *data) {
        int x1 = x0 + w - 1;
        int y1 = y0 + h - 1;
        {
            uint8_t data[] = { 
                (uint8_t)(x0 >> 8), 
                (uint8_t)(x0 & 0xff), 
                (uint8_t)(x1 >> 8), 
                (uint8_t)(x1 & 0xff)
            };
            write_command(0x2a, data, sizeof(data));
        }
        {
            uint8_t data[] = { 
                (uint8_t)(y0 >> 8), 
                (uint8_t)(y0 & 0xff), 
                (uint8_t)(y1 >> 8), 
                (uint8_t)(y1 & 0xff)
            };
            write_command(0x2b, data, sizeof(data));
        }
        //write_command(0x2c, data, w * h * 3 / 2);
        {
            gpio_put(PIN_DC, 0);
            gpio_put(PIN_CS, 0);
            uint8_t cmd = 0x2c;
            write_blocking(&cmd, 1);
            gpio_put(PIN_DC, 1);
            
            {
                dma_channel_config dma_cfg = dma_channel_get_default_config(dma_tx);
                channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
                channel_config_set_dreq(&dma_cfg, pio_get_dreq(spi_pio, spi_sm, true));
                dma_channel_configure(dma_tx, &dma_cfg,
                                      &spi_pio->txf[spi_sm], // write address
                                      data, // read address
                                      w * h * 3 / 2, // element count (each element is of size transfer_data_size)
                                      false); // don't start yet
            }
            
            dma_start_channel_mask(1u << dma_tx);
        }
    }
    
    void finish_write_data() {
        dma_channel_wait_for_finish_blocking(dma_tx);
        pio_wait_idle();
        gpio_put(PIN_CS, 1);
    }

    void write_command(uint8_t cmd, const uint8_t *data, int len) {
        gpio_put(PIN_DC, 0); // command mode
        gpio_put(PIN_CS, 0);
        write_blocking(&cmd, 1);
        if (data) {
            gpio_put(PIN_DC, 1); // data mode
            write_blocking(data, len);
        }
        gpio_put(PIN_CS, 1);
    }
  
    void write_command(uint8_t cmd) {
        write_command(cmd, nullptr, 0);
    }
    
    void write_blocking(const uint8_t *data, int len) {
        for(int i = 0; i < len; i++) {
            pio_push(data[i]);
        }
        pio_wait_idle();
    }
    
    void pio_push(uint8_t data) {
        while (pio_sm_is_tx_fifo_full(spi_pio, spi_sm)) { }
        *(volatile uint8_t*)&spi_pio->txf[spi_sm] = data;
    }

    void pio_wait_idle() {
        uint32_t stall_mask = 1u << (spi_sm + PIO_FDEBUG_TXSTALL_LSB);
        spi_pio->fdebug = stall_mask;
        while (!(spi_pio->fdebug & stall_mask)) { }
    }
};
