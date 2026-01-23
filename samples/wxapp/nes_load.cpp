#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "nes_load.hpp"

char nes_path[nes::MAX_PATH_LENGTH] = "";
std::vector<uint8_t> ines_image;

nes::result_t load_nes_file(const char *path) {
  try {
    std::ifstream ifs(path, std::ios::binary);

    ifs.seekg(0, std::ios::end);
    auto size = ifs.tellg();
    ifs.seekg(0);

    std::vector<uint8_t> vec(size);
    ifs.read((char *)&vec[0], size);
    ines_image = std::move(vec);

    nes::map_ines(&ines_image[0]);
    nes::menu::hide();
  } catch (...) {
    return nes::result_t::ERR_FAILED_TO_READ_FILE;
  }
  return nes::result_t::SUCCESS;
}
