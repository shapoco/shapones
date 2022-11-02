#pragma once

#include "stdint.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include <algorithm>

class WS19804 {
public:
    static constexpr int WIDTH = 320;
    static constexpr int HEIGHT = 240;
    static constexpr uint PIN_UNUSED = 255;
    static constexpr uint PIN_DC = 8;
    static constexpr uint PIN_CS = 9;
    static constexpr uint PIN_SCK = 10;
    static constexpr uint PIN_DIN = 11;
    static constexpr uint PIN_RST = 12;
    static constexpr uint PIN_BL = 13;

    spi_inst_t * const spi;
    uint dma_tx;
    
    // Parallel init
    WS19804() : spi(spi1)
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
        
        spi_init(spi, 62.5 * MHZ);
        gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
        gpio_set_function(PIN_DIN, GPIO_FUNC_SPI);
        
        // https://forums.raspberrypi.com/viewtopic.php?t=311664&start=25
        // see also: ~/pico/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c
        hw_write_masked(&spi_get_hw(spi)->cr0, 1 << SPI_SSPCR0_SCR_LSB, SPI_SSPCR0_SCR_BITS);
    }

    void init() {
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
            uint8_t data[] = { 0x05 };
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
        uint16_t data[WIDTH];
        for (int x = 0; x < WIDTH; x++) {
            data[x] = color;
        }
        for (int y = 0; y < HEIGHT; y++) {
            start_write_data(0, y, WIDTH, 1, data);
            finish_write_data();
        }
    }

    void start_write_data(int x0, int y0, int w, int h, uint16_t *data) {
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
        //write_command(0x2c, (const uint8_t*)line_buff, w * h * 2);
        {
            dma_channel_config c = dma_channel_get_default_config(dma_tx);
            channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
            channel_config_set_dreq(&c, spi_get_dreq(spi1, true));
            dma_channel_configure(dma_tx, &c,
                                  &spi_get_hw(spi1)->dr, // write address
                                  data, // read address
                                  w * h * 2, // element count (each element is of size transfer_data_size)
                                  false); // don't start yet
        
            gpio_put(PIN_DC, 0);
            gpio_put(PIN_CS, 0);
            uint8_t cmd = 0x2c;
            spi_write_blocking(spi, &cmd, 1);
            gpio_put(PIN_DC, 1);
            dma_start_channel_mask(1u << dma_tx);
        }
    }
    
    void finish_write_data() {
        dma_channel_wait_for_finish_blocking(dma_tx);
        while (spi_is_busy(spi)) { }
        gpio_put(PIN_CS, 1);
    }

    void write_command(uint8_t cmd, const uint8_t *data, int len) {
        gpio_put(PIN_DC, 0); // command mode
        gpio_put(PIN_CS, 0);
        spi_write_blocking(spi, &cmd, 1);
        if (data) {
            gpio_put(PIN_DC, 1); // data mode
            spi_write_blocking(spi, data, len);
        }
        gpio_put(PIN_CS, 1);
    }
  
    void write_command(uint8_t cmd) {
        write_command(cmd, nullptr, 0);
    }
};
