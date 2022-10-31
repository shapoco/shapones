#ifndef SHAPONES_INTERRUPT_HPP
#define SHAPONES_INTERRUPT_HPP

#include "shapones.hpp"

namespace nes::interrupt {

void assert_irq();
void deassert_irq();
bool is_irq_asserted();

void assert_nmi();
void deassert_nmi();
bool is_nmi_asserted();

}

#endif
