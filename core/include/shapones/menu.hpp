#ifndef SHAPONES_MENU_HPP
#define SHAPONES_MENU_HPP

#include "shapones/common.hpp"

namespace nes::menu {

enum class tab_t {
  NES_LIST,
  SAVE_LIST,
  COUNT,
};

extern bool shown;

static SHAPONES_INLINE bool is_shown() { return shown; }

result_t init();
void deinit();

void show();
void hide();

result_t service();
result_t overlay(int y, uint8_t *line_buff);

}  // namespace nes::menu

#endif
