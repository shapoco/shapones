#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"

namespace nes::interrupt {

static constexpr uint32_t STATE_SIZE = 16;

static volatile source_t irq;
static volatile bool nmi;

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  SpinLockBlock lock(SPINLOCK_IRQ);
  irq = static_cast<source_t>(0);
  nmi = false;
  return result_t::SUCCESS;
}

void assert_irq(source_t src) {
  SpinLockBlock lock(SPINLOCK_IRQ);
  irq = irq | src;
}
void deassert_irq(source_t src) {
  SpinLockBlock lock(SPINLOCK_IRQ);
  irq = irq & ~src;
}
source_t get_irq() { return irq; }

void assert_nmi() { nmi = true; }
void deassert_nmi() { nmi = false; }
bool is_nmi_asserted() { return nmi; }

uint32_t get_state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  memset(buffer, 0, STATE_SIZE);
  uint8_t *p = buffer;
  BufferWriter writer(p);
  writer.u32(static_cast<uint32_t>(irq));
  writer.b(nmi);
  return fs_write(file_handle, buffer, STATE_SIZE);
}

result_t load_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buffer, STATE_SIZE));
  const uint8_t *p = buffer;
  BufferReader reader(p);
  irq = static_cast<source_t>(reader.u32());
  nmi = reader.b();
  return result_t::SUCCESS;
}

}  // namespace nes::interrupt
