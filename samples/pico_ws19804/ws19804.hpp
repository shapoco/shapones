#ifndef WS19804_HPP
#define WS19804_HPP

#include "stdint.h"

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

enum direction_t {
    EMPTY, TX, RX
};

void init(int sys_clk_hz);
void setup_pio(direction_t new_dir, int new_speed);
void set_direction(direction_t new_dir);
void set_speed(int new_speed);

void clear(uint16_t color);

void start_write_data(int x0, int y0, int w, int h, uint8_t *data);
void finish_write_data();
void write_command(uint8_t cmd, const uint8_t *data, int len);
void write_command(uint8_t cmd);
void write_blocking(const uint8_t *data, int len);
void read_blocking(uint8_t tx_repeat, uint8_t *buff, int len);

}

#endif
