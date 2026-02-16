#ifndef SHAPONES_XIAO_RP2350_POWER_HPP
#define SHAPONES_XIAO_RP2350_POWER_HPP

#include "pca9555.hpp"

namespace shapones::xiao::rp::power {

void init(pca9555::Driver *ie);
void service();
void shutdown();

}

#endif  
