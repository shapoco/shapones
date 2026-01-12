#include "shapones/interrupt.hpp"

namespace nes::interrupt {

static volatile Source irq;
static volatile bool nmi;

void assert_irq(Source src) {
    Exclusive lock(LOCK_INTERRUPTS);
    irq = irq | src;
}
void deassert_irq(Source src) {
    Exclusive lock(LOCK_INTERRUPTS);
    irq = irq & ~src;
}
Source get_irq() { return irq; }

void assert_nmi() { nmi = true; }
void deassert_nmi() { nmi = false; }
bool is_nmi_asserted() { return nmi; }

}  // namespace nes::interrupt
