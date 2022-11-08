#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "ws19804.hpp"
#include "pwm_audio.hpp"
#include "shapones/shapones.hpp"
#include "nes_rom.hpp"

static constexpr uint32_t SYS_CLK_FREQ = 250 * MHZ;

static constexpr int PIN_MONITOR = 1;

static constexpr int PIN_PAD_A      = 2;
static constexpr int PIN_PAD_B      = 3;
static constexpr int PIN_PAD_START  = 4;
static constexpr int PIN_PAD_SELECT = 6;
static constexpr int PIN_PAD_RIGHT  = 7;
static constexpr int PIN_PAD_DOWN   = 14;
static constexpr int PIN_PAD_LEFT   = 26;
static constexpr int PIN_PAD_UP     = 27;

static constexpr int PIN_SPEAKER = 28;
static constexpr int SPK_LATENCY = 256;
static constexpr int SPK_PWM_FREQ = 22050;

static void boot_nes();
static void cpu_loop();
static void ppu_loop();
static void apu_dma_handler();
static void apu_fill_buffer(PwmAudio::sample_t *buff);

WS19804 lcd;
PwmAudio speaker(PIN_SPEAKER, SPK_LATENCY, 8, (float)SYS_CLK_FREQ / SPK_PWM_FREQ, apu_dma_handler);

static const uint16_t COLOR_TABLE[] = {
    0x555, 0x027, 0x019, 0x308, 0x406, 0x603, 0x500, 0x410, 
    0x230, 0x040, 0x040, 0x040, 0x034, 0x000, 0x000, 0x000, 
    0x999, 0x05C, 0x33F, 0x62E, 0x81B, 0xA16, 0x922, 0x740, 
    0x560, 0x270, 0x080, 0x072, 0x067, 0x000, 0x000, 0x000, 
    0xFFF, 0x5AF, 0x78F, 0xB6F, 0xE5F, 0xF5B, 0xF76, 0xD82, 
    0xAB0, 0x7C0, 0x5D2, 0x3D7, 0x3BD, 0x444, 0x000, 0x000, 
    0xFFF, 0xADF, 0xCCF, 0xDBF, 0xFBF, 0xFBD, 0xFBB, 0xEC9, 
    0xDD7, 0xBE7, 0xAE9, 0x9EB, 0xADE, 0xAAA, 0x000, 0x000, 
};

static const int input_pins[] = {
    PIN_PAD_A, PIN_PAD_B, PIN_PAD_SELECT, PIN_PAD_START,
    PIN_PAD_UP, PIN_PAD_DOWN, PIN_PAD_LEFT, PIN_PAD_RIGHT
};

uint8_t line_buff[nes::SCREEN_WIDTH];
uint8_t frame_buff[240*240*3/2];
uint8_t spk_buff[SPK_LATENCY];

static volatile bool vsync_flag = false;

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(100);
    stdio_init_all();
    set_sys_clock_khz(SYS_CLK_FREQ / 1000, true);
    setup_default_uart();
    
    gpio_init(PIN_MONITOR);
    gpio_set_dir(PIN_MONITOR, GPIO_OUT);
    gpio_put(PIN_MONITOR, 0);
    
    for(int i = 0; i < 8; i++) {
        int pin = input_pins[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
    
    lcd.init();
    lcd.clear(0);

    boot_nes();

    return 0;
}

static void boot_nes() {
    nes::memory::map_ines(nes_rom);
    nes::reset();
    nes::apu::set_sampling_rate(SPK_PWM_FREQ);

    apu_fill_buffer(speaker.get_buffer(0));
    apu_fill_buffer(speaker.get_buffer(1));
    speaker.play();
    
    multicore_launch_core1(ppu_loop);

    cpu_loop();
}

static void cpu_loop() {
    for(;;) {
        nes::cpu::service();
        
        nes::input::InputStatus input_status;
        input_status.raw = 0;
        for(int i = 0; i < 8; i++) {
            if ( ! gpio_get(input_pins[i])) {
                input_status.raw |= (1 << i);
            }
        }
        nes::input::set_raw(0, input_status);
        
        if (vsync_flag) {
            vsync_flag = false;
            lcd.finish_write_data();
            lcd.start_write_data(25, 0, 240, 240, frame_buff);
            static int tmp = 0;
            gpio_put(PIN_MONITOR, tmp);
            tmp ^= 1;
        }
    }
}

static void ppu_loop() {
    for(;;) {
        bool eol = nes::ppu::service(line_buff);
        if (eol) {
            int y = nes::ppu::current_focus_y();
            if (y < nes::SCREEN_HEIGHT) {
                uint8_t *rd_ptr = line_buff + 8;
                uint8_t *wr_ptr = frame_buff + (y * (240 * 3 / 2));
                for (int x = 0; x < 240; x+=2) {
                    uint16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                    uint16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                    *(wr_ptr++) = (c0 >> 4) & 0xff;
                    *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
                    *(wr_ptr++) = c1 & 0xff;
                }
            }
            else if (y == nes::SCREEN_HEIGHT) {
                vsync_flag = true;
            }
        }
        
    }
}

static void apu_dma_handler() {
    speaker.flip_buffer();
    apu_fill_buffer(speaker.get_next_buffer());
}

static void apu_fill_buffer(PwmAudio::sample_t *buff) {
    nes::apu::service(spk_buff, speaker.LATENCY);
    for (int i = 0; i < speaker.LATENCY; i++) {
        buff[i] = spk_buff[i];
    }
}