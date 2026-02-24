#ifndef SHAPONES_XIAO_POWER_HPP
#define SHAPONES_XIAO_POWER_HPP

#include <stdint.h>

namespace shapones::xiao::power {

void init();
void service();
void deep_sleep();

}  // namespace shapones::xiao::power

#endif
