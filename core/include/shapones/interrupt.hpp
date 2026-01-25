#ifndef SHAPONES_INTERRUPT_HPP
#define SHAPONES_INTERRUPT_HPP

#include "shapones/common.hpp"

namespace nes::interrupt {

enum class source_t : uint32_t {
  APU_FRAME_COUNTER = (1 << 0),
  APU_DMC = (1 << 1),
  MAPPER = (1 << 2),
};

static SHAPONES_INLINE source_t operator|(source_t a, source_t b) {
  return static_cast<source_t>(static_cast<uint32_t>(a) |
                             static_cast<uint32_t>(b));
}
static SHAPONES_INLINE source_t operator&(source_t a, source_t b) {
  return static_cast<source_t>(static_cast<uint32_t>(a) &
                             static_cast<uint32_t>(b));
}
static SHAPONES_INLINE bool operator!(source_t a) {
  return !static_cast<uint32_t>(a);
}
static SHAPONES_INLINE source_t operator~(source_t a) {
  return static_cast<source_t>(~static_cast<uint32_t>(a));
}
static SHAPONES_INLINE source_t& operator|=(source_t& a, source_t b) {
  a = a | b;
  return a;
}
static SHAPONES_INLINE source_t& operator&=(source_t& a, source_t b) {
  a = a & b;
  return a;
}

result_t init();
void deinit();

result_t reset();

void assert_irq(source_t src);
void deassert_irq(source_t src);
source_t get_irq();

void assert_nmi();
void deassert_nmi();
bool is_nmi_asserted();

uint32_t get_state_size();
result_t save_state(void *file_handle);
result_t load_state(void *file_handle);

}  // namespace nes::interrupt

#endif
