#include "shapones/interrupt.hpp"

namespace nes::interrupt {

static bool irq = false;
static bool nmi = false;

void assert_irq() { irq = true; }
void deassert_irq() { irq = false; }
bool is_irq_asserted() { return irq; }

void assert_nmi() { nmi = true; }
void deassert_nmi() { nmi = false; }
bool is_nmi_asserted() { return nmi; }

}
