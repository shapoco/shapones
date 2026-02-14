#ifndef SHAPONES_XIAO_RP2350_KEYPAD_HPP
#define SHAPONES_XIAO_RP2350_KEYPAD_HPP

#include "mcp23017.hpp"

namespace shapones::xiao::rp::keypad {

void init(mcp23017::Driver *ie);
void update();
bool is_pressed(int key);

}  // namespace shapones::xiao::rp2350::keypad

#endif  // SHAPONES_XIAO_RP2350_KEYPAD_HPP
