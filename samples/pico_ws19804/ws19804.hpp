#ifndef WS19804_HPP
#define WS19804_HPP

#define SHAPONES_USE_PIO (1)

#include "pico/stdlib.h"
#include "stdint.h"
#if SHAPONES_USE_PIO
#include "hardware/pio.h"
#else
#include "hardware/spi.h"
#endif
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

namespace ws19804 {

static constexpr int WIDTH = 320;
static constexpr int HEIGHT = 240;
static constexpr uint PIN_DC = 8;
static constexpr uint PIN_LCD_CS = 9;
static constexpr uint PIN_TP_CS = 16;
static constexpr uint PIN_SCK = 10;
static constexpr uint PIN_MOSI = 11;
static constexpr uint PIN_MISO = 12;
static constexpr uint PIN_RST = 15;
static constexpr uint PIN_BL = 13;

enum direction_t { EMPTY, TX, RX };

// setup GPIO, PIO, LCD
void init(int sys_clk_hz);

// set SPI clock frequency
void set_spi_speed(int new_speed);

// clear display
void clear(uint16_t color);

// start DMA
void start_write_data(int x0, int y0, int w, int h, uint8_t *data);

// wait DMA to finish
void finish_write_data();

// write LCD command
void write_command(uint8_t cmd, const uint8_t *data, int len);

// write LCD command
void write_command(uint8_t cmd);

// SPI write
void write_blocking(const uint8_t *data, int len);

// SPI read
void read_blocking(uint8_t tx_repeat, uint8_t *buff, int len);

}  // namespace ws19804

#endif
