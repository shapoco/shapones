#if SHAPONES_SUPPORT_INES_FILE
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#endif

#include "shapones/cpu.hpp"
#include "shapones/memory.hpp"
#include "shapones/utils_std.hpp"

namespace nes {

#if SHAPONES_SUPPORT_INES_FILE
std::vector<uint8_t> ines_image;

void load_ines_file(const char* path) {
  std::ifstream ifs(path, std::ios::binary);

  ifs.seekg(0, std::ios::end);
  auto size = ifs.tellg();
  ifs.seekg(0);

  std::vector<uint8_t> vec(size);
  ifs.read((char*)&vec[0], size);
  ines_image = std::move(vec);

  memory::map_ines(&ines_image[0]);
}
#endif

}  // namespace nes
