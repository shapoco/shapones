#ifndef SHAPONES_LOCK_HPP
#define SHAPONES_LOCK_HPP

#include "shapones/host_intf.hpp"

namespace nes {

class Exclusive {
 public:
  const int id;
  SHAPONES_INLINE Exclusive(int id) : id(id) { lock_get(id); }
  SHAPONES_INLINE ~Exclusive() { lock_release(id); }
};

}  // namespace nes

#endif