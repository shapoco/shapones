#include "../include.h"

#include "shapones/shapones.hpp"

static constexpr uint16_t NUM_WORKERS = 2;

static int feed_index = 0;
static uint16_t line_buff[WIDTH];

static uint64_t last_busy_time_ms = 0;
static volatile bool busy = false;

static void core1_main();
static void paint();

int main() {
#if USE_PICOPAD10 || USE_PICOPAD20
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    WaitMs(100);
    set_sys_clock_khz(250000, true);
#endif

    Core1Exec(core1_main);

    while (True) {
        uint64_t now_ms = Time64() / 1000;

        // fb::button_t key_pressed = fb::button_t::NONE;
        // if (KeyPressedFast(KEY_RIGHT)) key_pressed |= fb::button_t::RIGHT;
        // if (KeyPressedFast(KEY_LEFT)) key_pressed |= fb::button_t::LEFT;
        // if (KeyPressedFast(KEY_UP)) key_pressed |= fb::button_t::UP;
        // if (KeyPressedFast(KEY_DOWN)) key_pressed |= fb::button_t::DOWN;
        // if (KeyPressedFast(KEY_A)) key_pressed |= fb::button_t::A;
        // if (KeyPressedFast(KEY_B)) key_pressed |= fb::button_t::B;
        // if (KeyPressedFast(KEY_X)) key_pressed |= fb::button_t::X;
        // if (KeyPressedFast(KEY_Y)) key_pressed |= fb::button_t::Y;

        // if ((key_pressed & fb::button_t::Y) != fb::button_t::NONE) {
        //   ResetToBootLoader();
        // }

        paint();

        if (!busy) {
            WaitMs(20);
        }
    }
}

static void core1_main() {
    while (true) {
        if (!busy) {
            WaitMs(20);
        }
    }
}

static void paint() {}
