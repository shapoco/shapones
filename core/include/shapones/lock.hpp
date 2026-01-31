#ifndef SHAPONES_LOCK_HPP
#define SHAPONES_LOCK_HPP

#include "shapones/host_intf.hpp"

namespace nes {

class SpinLockBlock {
 public:
  const int id;
  SHAPONES_INLINE SpinLockBlock(int id) : id(id) { spinlock_get(id); }
  SHAPONES_INLINE ~SpinLockBlock() { spinlock_release(id); }
};

class SemaphoreBlock {
 public:
  const int id;
  SHAPONES_INLINE SemaphoreBlock(int id) : id(id) { semaphore_take(id); }
  SHAPONES_INLINE ~SemaphoreBlock() { semaphore_give(id); }
};

}  // namespace nes

#endif