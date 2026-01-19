#include "shapones/mapper.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/memory.hpp"

#define SHAPONES_MAPPER_IMPLEMENTATION
#include "shapones/mappers/map000.hpp"
#include "shapones/mappers/map001.hpp"
#include "shapones/mappers/map003.hpp"
#include "shapones/mappers/map004.hpp"

#pragma GCC optimize("Ofast")

namespace nes::mapper {

Mapper *instance = nullptr;

void init(const uint8_t *ines) {
  uint8_t flags6 = ines[6];
  uint8_t flags7 = ines[7];

  int id = (flags7 & 0xf0) | ((flags6 >> 4) & 0xf);
  SHAPONES_PRINTF("Mapper No.%d\n", id);

  if (instance) {
    delete instance;
  }
  switch (id) {
    case 0: instance = new Map000(); break;
    case 1: instance = new Map001(); break;
    case 3: instance = new Map003(); break;
    case 4: instance = new Map004(); break;
    default:
      instance = new Map000();
      SHAPONES_ERRORF("Unsupported Mapper Number\n");
      break;
  }

  instance->init();
}

void deinit() {
  if (instance) {
    delete instance;
    instance = nullptr;
  }
}

}  // namespace nes::mapper
