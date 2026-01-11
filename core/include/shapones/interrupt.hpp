#ifndef SHAPONES_INTERRUPT_HPP
#define SHAPONES_INTERRUPT_HPP

#include "shapones/common.hpp"

namespace nes::interrupt {

enum class Source : uint32_t {
    APU_FRAME_COUNTER = (1 << 0),
    APU_DMC = (1 << 1),
    MMC3 = (1 << 2),
};

static NES_ALWAYS_INLINE Source operator|(Source a, Source b) {
    return static_cast<Source>(static_cast<uint32_t>(a) |
                               static_cast<uint32_t>(b));
}
static NES_ALWAYS_INLINE Source operator&(Source a, Source b) {
    return static_cast<Source>(static_cast<uint32_t>(a) &
                               static_cast<uint32_t>(b));
}
static NES_ALWAYS_INLINE bool operator!(Source a) {
    return !static_cast<uint32_t>(a);
}
static NES_ALWAYS_INLINE Source operator~(Source a) {
    return static_cast<Source>(~static_cast<uint32_t>(a));
}
static NES_ALWAYS_INLINE Source& operator|=(Source &a, Source b) {
    a = a | b;
    return a;
}
static NES_ALWAYS_INLINE Source& operator&=(Source &a, Source b) {
    a = a & b;
    return a;
}

void assert_irq(Source src);
void deassert_irq(Source src);
Source get_irq();

void assert_nmi();
void deassert_nmi();
bool is_nmi_asserted();

}  // namespace nes::interrupt

#endif
