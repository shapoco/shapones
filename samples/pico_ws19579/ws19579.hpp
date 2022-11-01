#pragma once

#include "stdint.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include <algorithm>

class WS19579 {
public:
    static constexpr int WIDTH = 160;
    static constexpr int HEIGHT = 128;
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
    WS19579() : spi(spi1)
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
        
        //// SWRESET
        //write_command(0x01);
        //sleep_ms(200);

        //ST7735R Frame Rate
        {
            uint8_t data[] = { 0x01, 0x2C, 0x2D };
            //uint8_t data[] = { 0x05, 0x3c, 0x3c };
            write_command(0xb1, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x01, 0x2C, 0x2D };
            //uint8_t data[] = { 0x05, 0x3c, 0x3c };
            write_command(0xb2, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d };
            //uint8_t data[] = { 0x05, 0x3c, 0x3c, 0x05, 0x3c, 0x3c };
            write_command(0xb3, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x07 };
            write_command(0xb4, data, sizeof(data));
        }

        //ST7735R Power Sequence
        {
            uint8_t data[] = { 0xA2, 0x02, 0x84 };
            write_command(0xC0, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0xC5 };
            write_command(0xC1, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x0A, 0x00 };
            write_command(0xC2, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x8A, 0x2A };
            write_command(0xC3, data, sizeof(data));
        }

        {
            uint8_t data[] = { 0x8A, 0xEE };
            write_command(0xC4, data, sizeof(data));
        }

        //VCOM
        {
            uint8_t data[] = { 0x0E };
            write_command(0xC5, data, sizeof(data));
        }

        //ST7735R Gamma Sequence
        {
            uint8_t data[] = {
                0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 
                0x1f, 0x1b, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10
            };
            write_command(0xe0, data, sizeof(data));
        }
        {
            uint8_t data[] = {
                0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 
                0x30, 0x30, 0x39, 0x3f, 0x00, 0x07, 0x03, 0x10
            };
            write_command(0xe1, data, sizeof(data));
        }

        //Enable test command
        {
            uint8_t data[] = { 0x01 };
            write_command(0xF0, data, sizeof(data));
        }

        //Disable ram power save mode
        {
            uint8_t data[] = { 0x00 };
            write_command(0xF6, data, sizeof(data));
        }

        //65k mode
        {
            uint8_t data[] = { 0x05 };
            write_command(0x3a, data, sizeof(data));
        }

        //MX, MY, RGB mode
        {
            uint8_t data[] = { 0xa0 };
            write_command(0x36, data, sizeof(data));
        }

        //sleep out
        write_command(0x11);
        sleep_ms(20);

        //Turn on the LCD display
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
        x0 += 1;
        y0 += 2;
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
