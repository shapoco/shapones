#include "shapones/xiao/lock.h"

#include <pico/sem.h>
#include <pico/stdlib.h>

static constexpr int NUM_SPINLOCKS = 16;
static uint32_t used_spinlocks = 0;

struct spin_lock_handle_t {
  spin_lock_t *hw_lock;
  uint32_t irqs;
  int id;
};

#if defined(__cplusplus)
extern "C" {
#endif

void xiao_spinlock_init(xiao_spin_lock_t *lock) {
  int id = __builtin_ffs(~used_spinlocks) - 1;
  if (0 <= id && id < NUM_SPINLOCKS) {
    spin_lock_handle_t *handle = new spin_lock_handle_t;
    handle->hw_lock = spin_lock_init(id);
    handle->id = id;
    lock->handle = handle;
    spin_lock_claim(id);
    used_spinlocks |= (1U << id);
  } else {
    // No available spinlock, handle error as needed
  }
}

void xiao_spinlock_deinit(xiao_spin_lock_t *lock) {
  spin_lock_handle_t *handle = static_cast<spin_lock_handle_t *>(lock->handle);
  if (!handle) return;
  spin_lock_unclaim(handle->id);
  used_spinlocks &= ~(1U << handle->id);
  delete handle;
  lock->handle = nullptr;
}

void xiao_spinlock_get(xiao_spin_lock_t *lock) {
  spin_lock_handle_t *handle = static_cast<spin_lock_handle_t *>(lock->handle);
  if (!handle) return;
  handle->irqs = spin_lock_blocking(handle->hw_lock);
}

void xiao_spinlock_release(xiao_spin_lock_t *lock) {
  spin_lock_handle_t *handle = static_cast<spin_lock_handle_t *>(lock->handle);
  if (!handle) return;
  spin_unlock(handle->hw_lock, handle->irqs);
}

void xiao_semaphore_init(xiao_semaphore_t *sem) {
  semaphore_t *sem_handle = new semaphore_t;
  sem->handle = sem_handle;
  sem_init(sem_handle, 1, 1);
}

void xiao_semaphore_deinit(xiao_semaphore_t *sem) {
  delete static_cast<semaphore_t *>(sem->handle);
  sem->handle = nullptr;
}

void xiao_semaphore_take(xiao_semaphore_t *sem) {
  sem_acquire_blocking(static_cast<semaphore_t *>(sem->handle));
}

bool xiao_semaphore_try_take(xiao_semaphore_t *sem) {
  return sem_try_acquire(static_cast<semaphore_t *>(sem->handle));
}

void xiao_semaphore_give(xiao_semaphore_t *sem) {
  sem_release(static_cast<semaphore_t *>(sem->handle));
}

#if defined(__cplusplus)
}
#endif
