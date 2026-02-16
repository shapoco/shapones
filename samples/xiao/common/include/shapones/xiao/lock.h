#ifndef SHAPONES_XIAO_LOCK_HPP
#define SHAPONES_XIAO_LOCK_HPP

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct {
  void *handle;
} xiao_spin_lock_t;

typedef struct {
  void *handle;
} xiao_semaphore_t;

void xiao_spinlock_init(xiao_spin_lock_t *lock);
void xiao_spinlock_deinit(xiao_spin_lock_t *lock);
void xiao_spinlock_get(xiao_spin_lock_t *lock);
void xiao_spinlock_release(xiao_spin_lock_t *lock);

void xiao_semaphore_init(xiao_semaphore_t *sem);
void xiao_semaphore_deinit(xiao_semaphore_t *sem);
void xiao_semaphore_take(xiao_semaphore_t *sem);
bool xiao_semaphore_try_take(xiao_semaphore_t *sem);
void xiao_semaphore_give(xiao_semaphore_t *sem);

#if defined(__cplusplus)
}
#endif

#endif
