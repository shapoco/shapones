#include "shapones/xiao/lock.h"

#include <Arduino.h>

#if defined(__cplusplus)
extern "C" {
#endif

void xiao_spinlock_init(xiao_spin_lock_t *lock) {
  spinlock_t *spinlock = new spinlock_t;
  spinlock_initialize(spinlock);
  lock->handle = spinlock;
}

void xiao_spinlock_deinit(xiao_spin_lock_t *lock) {
  if (!lock->handle) return;
  spinlock_t *spinlock = static_cast<spinlock_t *>(lock->handle);
  delete spinlock;
  lock->handle = nullptr;
}

void xiao_spinlock_get(xiao_spin_lock_t *lock) {
  if (!lock->handle) return;
  spinlock_t *spinlock = static_cast<spinlock_t *>(lock->handle);
  taskENTER_CRITICAL(spinlock);
}

void xiao_spinlock_release(xiao_spin_lock_t *lock) {
  if (!lock->handle) return;
  spinlock_t *spinlock = static_cast<spinlock_t *>(lock->handle);
  taskEXIT_CRITICAL(spinlock);
}

void xiao_semaphore_init(xiao_semaphore_t *sem) {
  SemaphoreHandle_t handle = xSemaphoreCreateBinary();
  if (!handle) return;
  xSemaphoreGive(handle);
  sem->handle = handle;
}

void xiao_semaphore_deinit(xiao_semaphore_t *sem) {
  if (!sem->handle) return;
  SemaphoreHandle_t handle = static_cast<SemaphoreHandle_t>(sem->handle);
  vSemaphoreDelete(handle);
  sem->handle = nullptr;
}

void xiao_semaphore_take(xiao_semaphore_t *sem) {
  if (!sem->handle) return;
  SemaphoreHandle_t handle = static_cast<SemaphoreHandle_t>(sem->handle);
  xSemaphoreTake(handle, portMAX_DELAY);
}

bool xiao_semaphore_try_take(xiao_semaphore_t *sem) {
  if (!sem->handle) return false;
  SemaphoreHandle_t handle = static_cast<SemaphoreHandle_t>(sem->handle);
  return xSemaphoreTake(handle, 0) == pdTRUE;
}

void xiao_semaphore_give(xiao_semaphore_t *sem) {
  if (!sem->handle) return;
  SemaphoreHandle_t handle = static_cast<SemaphoreHandle_t>(sem->handle);
  xSemaphoreGive(handle);
}

#if defined(__cplusplus)
}
#endif
