#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "ws19579.hpp"
#include "shapones/shapones.hpp"
#include "nes_rom.hpp"

WS19579 lcd;

static const uint16_t COLOR_TABLE[] = {
    0x8410, 0x01F4, 0x0096, 0x4012, 0xA00B, 0xC005, 0xB820, 0x88A0, 
    0x5960, 0x1220, 0x0240, 0x0225, 0x020C, 0x0000, 0x0020, 0x0020, 
    0xC638, 0x03BF, 0x22BF, 0x81BF, 0xE976, 0xF94A, 0xF900, 0xD180, 
    0xC300, 0x3400, 0x0460, 0x044A, 0x04D9, 0x2104, 0x0841, 0x0841, 
    0xFFFF, 0x0EBF, 0x6D1F, 0xD41F, 0xFA3E, 0xFB11, 0xFC46, 0xFCE2, 
    0xFDE4, 0x9F01, 0x2F86, 0x0F94, 0x07DF, 0x5AEB, 0x0861, 0x0861, 
    0xFFFF, 0xA7FF, 0xB77F, 0xDD5D, 0xFD5F, 0xFD56, 0xFE96, 0xFF74, 
    0xFFB3, 0xD752, 0xA775, 0xA79B, 0x9FFF, 0xDEFB, 0x1082, 0x1082, 
};

uint8_t line_buff_0[nes::SCREEN_WIDTH];
uint8_t line_buff_1[nes::SCREEN_WIDTH];
uint8_t color_buff[256*240];
uint16_t frame_buff[128*120];

static volatile bool vsync_flag = false;

static void lcd_driver_entry();

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(100);
    stdio_init_all();
    set_sys_clock_khz(250000, true);
    //set_sys_clock_khz(270000, true);
    //set_sys_clock_khz(275000, true);
    set_sys_clock_khz(280000, true);
    //set_sys_clock_khz(290000, true);
    //set_sys_clock_khz(300000, true);
    setup_default_uart();
    
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 0);
    
    lcd.init();
    lcd.clear(0);

    nes::memory::map_ines(nes_rom);
    nes::reset();
    
    multicore_launch_core1(lcd_driver_entry);
    
    for(;;) {
        gpio_put(2, 1);
        vsync_flag = true;
        nes::vsync(line_buff_0);
        vsync_flag = false;
        gpio_put(2, 0);
        for (int y = 0; y < 120; y++) {
            nes::render_next_line(color_buff + y * 512);
            nes::render_next_line(color_buff + y * 512 + 256);
        }
    }

    return 0;
}

static void lcd_driver_entry() {
    for(;;) {
        while (!vsync_flag) { sleep_us(100); }
        while (vsync_flag) { sleep_us(100); }
        lcd.finish_write_data();
        
        uint16_t *wr_ptr = frame_buff;
        for (int y = 0; y < 120; y++) {
            for (int x = 0; x < 128; x++) {
                uint32_t c0 = COLOR_TABLE[color_buff[y * 512 + x * 2] & 0x3f];
                uint32_t c1 = COLOR_TABLE[color_buff[y * 512 + x * 2 + 1] & 0x3f];
                uint16_t r = (((c0 & 0x001f) + (c1 & 0x001f)) / 2) & 0x001f;
                uint16_t g = (((c0 & 0x07e0) + (c1 & 0x07e0)) / 2) & 0x07e0;
                uint16_t b = (((c0 & 0xf800) + (c1 & 0xf800)) / 2) & 0xf800;
                uint16_t co = r | g | b;
                *(wr_ptr++) = ((co >> 8) & 0xff) | ((co << 8) & 0xff00);
            }
        }
        lcd.start_write_data(16, 4, 128, 120, frame_buff);
    }
}
