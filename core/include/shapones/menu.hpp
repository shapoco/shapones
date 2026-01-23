#ifndef SHAPONES_MENU_HPP
#define SHAPONES_MENU_HPP

#include "shapones/common.hpp"

namespace nes::menu {

enum class tab_t {
  NES_LIST,
  COUNT,
};

extern bool shown;

static SHAPONES_INLINE bool is_shown() { return shown; }

result_t init();
void deinit();

void show();
void hide();

result_t update();
result_t render(int y, uint8_t *line_buff);

}  // namespace nes::menu

#endif
