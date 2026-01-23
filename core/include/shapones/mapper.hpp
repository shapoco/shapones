#ifndef SHAPONES_MAPPER_HPP
#define SHAPONES_MAPPER_HPP

#include "shapones/common.hpp"
#include "shapones/ppu.hpp"

namespace nes::mapper {

class Mapper {
 public:
  const int number;
  const char *name;

  Mapper(int number, const char *name) : number(number), name(name) {}
  virtual ~Mapper() {}

  virtual result_t init() { return result_t::SUCCESS; }
  virtual void reset() {}
  virtual bool vblank(const nes::ppu::Registers &reg) { return false; }
  virtual bool hblank(const nes::ppu::Registers &reg, int y) { return false; }
  virtual uint8_t read(addr_t addr) { return 0; }
  virtual void write(addr_t addr, uint8_t value) {}
};

extern Mapper *instance;

result_t init();
void deinit();

result_t map_ines(const uint8_t *ines);

}  // namespace nes::mapper

#endif