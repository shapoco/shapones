#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "ws19804.hpp"
#include "shapones/shapones.hpp"
#include "nes_rom.hpp"

WS19804 lcd;

static const uint16_t COLOR_TABLE[] = {
    0x1084, 0xF401, 0x9600, 0x1240, 0x0BA0, 0x05C0, 0x20B8, 0xA088, 
    0x6059, 0x2012, 0x4002, 0x2502, 0x0C02, 0x0000, 0x2000, 0x2000, 
    0x38C6, 0xBF03, 0xBF22, 0xBF81, 0x76E9, 0x4AF9, 0x00F9, 0x80D1, 
    0x00C3, 0x0034, 0x6004, 0x4A04, 0xD904, 0x0421, 0x4108, 0x4108, 
    0xFFFF, 0xBF0E, 0x1F6D, 0x1FD4, 0x3EFA, 0x11FB, 0x46FC, 0xE2FC, 
    0xE4FD, 0x019F, 0x862F, 0x940F, 0xDF07, 0xEB5A, 0x6108, 0x6108, 
    0xFFFF, 0xFFA7, 0x7FB7, 0x5DDD, 0x5FFD, 0x56FD, 0x96FE, 0x74FF, 
    0xB3FF, 0x52D7, 0x75A7, 0x9BA7, 0xFF9F, 0xFBDE, 0x8210, 0x8210, 
};

uint8_t dummy[nes::SCREEN_WIDTH];
uint8_t color_buff[256*240];
uint16_t frame_buff[240*240];

static volatile bool vsync_flag = false;

static void lcd_driver_entry();

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(100);
    stdio_init_all();
    //set_sys_clock_khz(250000, true);
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
        vsync_flag = true;
        nes::vsync(dummy);
        vsync_flag = false;
        
        gpio_put(2, 1);
        for (int y = 0; y < 120; y++) {
            nes::render_next_line(color_buff + y * 256);
        }
        
        gpio_put(2, 0);
        for (int y = 120; y < 240; y++) {
            nes::render_next_line(color_buff + y * 256);
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
        for (int y = 0; y < 240; y++) {
            uint8_t *rd_ptr = color_buff + (y * 256 + 8);
            for (int x = 0; x < 240; x++) {
                *(wr_ptr++) = COLOR_TABLE[*(rd_ptr++) & 0x3f];
            }
        }
        lcd.start_write_data(20, 0, 240, 240, frame_buff);
    }
}
