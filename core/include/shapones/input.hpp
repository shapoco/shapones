#ifndef SHAPONES_INPUT_HPP
#define SHAPONES_INPUT_HPP

#include "shapones/common.hpp"

namespace nes::input {

static constexpr int BTN_A      = 0;
static constexpr int BTN_B      = 1;
static constexpr int BTN_SELECT = 2;
static constexpr int BTN_START  = 3;
static constexpr int BTN_UP     = 4;
static constexpr int BTN_DOWN   = 5;
static constexpr int BTN_LEFT   = 6;
static constexpr int BTN_RIGHT  = 7;

union InputStatus {
    uint8_t raw;
    struct {
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t select : 1;
        uint8_t start : 1;
        uint8_t up : 1;
        uint8_t down : 1;
        uint8_t left : 1;
        uint8_t right : 1;
    };
};

union InputControl {
    uint8_t raw;
    struct {
        uint8_t strobe : 1;
        uint8_t reserved : 7;
    };
};

InputStatus get_raw(int player);
void set_raw(int player, InputStatus data);
void update();
uint8_t read_latched(int player);
void write_control(uint8_t data);

}

#endif
