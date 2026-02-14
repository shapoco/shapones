#ifndef SHAPONES_XIAO_RP2350_POWER_HPP
#define SHAPONES_XIAO_RP2350_POWER_HPP

#include "mcp23017.hpp"

namespace shapones::xiao::rp::power {

void init(mcp23017::Driver *ie);
void service();
void shutdown();

}

#endif  
