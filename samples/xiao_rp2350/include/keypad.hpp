#ifndef SHAPONES_XIAO_RP2350_KEYPAD_HPP
#define SHAPONES_XIAO_RP2350_KEYPAD_HPP

#include "parallel_switch.hpp"

namespace shapones::xiao::rp2350::keypad {

static constexpr int NUM_ADC_PINS = 3;
static constexpr int ADC_CHANNELS[NUM_ADC_PINS] = {0, 1, 2};

void init();
bool update();
bool is_pressed(int key);

}  // namespace shapones::xiao::rp2350::keypad

#endif  // SHAPONES_XIAO_RP2350_KEYPAD_HPP
