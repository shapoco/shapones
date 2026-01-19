#ifndef SHAPONES_MAP000_HPP
#define SHAPONES_MAP000_HPP

#include "shapones/mapper.hpp"

namespace nes::mapper {

class Map000 : public Mapper {
 public:
  Map000() : Mapper(0, "NROM") {}
};

}  // namespace nes::mapper

#endif
