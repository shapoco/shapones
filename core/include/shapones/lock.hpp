#ifndef SHAPONES_LOCK_HPP
#define SHAPONES_LOCK_HPP

#include "shapones/host_intf.hpp"

namespace nes {

class LockBlock {
 public:
  const int id;
  SHAPONES_INLINE LockBlock(int id) : id(id) { lock_get(id); }
  SHAPONES_INLINE ~LockBlock() { lock_release(id); }
};

class SemBlock {
 public:
  const int id;
  SHAPONES_INLINE SemBlock(int id) : id(id) { sem_take(id); }
  SHAPONES_INLINE ~SemBlock() { sem_give(id); }
};

}  // namespace nes

#endif