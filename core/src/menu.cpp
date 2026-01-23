#include "shapones/menu.hpp"
#include "shapones/font12x24.hpp"
#include "shapones/font8x16.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"

namespace nes::menu {

static constexpr int MAX_NUM_FILES = 1024;
#if SHAPONES_MENU_LARGE_FONT
static constexpr int CHAR_WIDTH = 12;
static constexpr int CHAR_HEIGHT = 24;
#else
static constexpr int CHAR_WIDTH = 8;
static constexpr int CHAR_HEIGHT = 16;
#endif
static constexpr int BUFF_WIDTH = SCREEN_WIDTH / CHAR_WIDTH;
static constexpr int BUFF_HEIGHT = SCREEN_HEIGHT / CHAR_HEIGHT;

static constexpr int CLIENT_X = 2;
static constexpr int CLIENT_Y = 2;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH - 4;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT - 4;

static constexpr int LIST_LINES = CLIENT_HEIGHT - 1;

// Workaround for the issue where repeated use of strdup in PicoLibSDK causes a
// panic:
static char *strdup_safe(const char *src) {
  if (src == nullptr) return nullptr;
  size_t len = strlen(src);
  char *dst = new char[len + 1];
  strcpy(dst, src);
  return dst;
}

bool shown = false;
tab_t tab = tab_t::NES_LIST;

const uint8_t PALETTE_FILE[] = {0x1D, 0x00, 0x10, 0x20, 0x01, 0x11, 0x21, 0x20};
static constexpr int NUM_PALETTES = sizeof(PALETTE_FILE) / 4;

uint8_t text_buff[BUFF_WIDTH * BUFF_HEIGHT];
uint8_t palette_buff[BUFF_WIDTH * BUFF_HEIGHT];

input::InputStatus key_pressed;
input::InputStatus key_down;

bool disk_mounted = false;
char current_dir[nes::MAX_PATH_LENGTH] = "";
file_info_t file_list[MAX_NUM_FILES];
int list_scroll_y = 0;
int selected_index = 0;
int num_files = 0;

static result_t change_tab(tab_t tab, bool force = false);
static result_t update_nes_list();
static void clear_file_list();
static result_t refresh_file_list();
static void print_file_list();
static void scroll_to(int index);
static int print_text(int x, int y, const char *str, int max_len = 999999);
static void print_char(int x, int y, char c);
static void fill_text(int x, int y, int w, int h, char c = ' ');
static void fill_palette(int x, int y, int w, int h, uint8_t p = 0x00);
static void clip_rect(int *x, int *y, int *w, int *h);
static bool is_root_dir(const char *path);
static int find_parent_separator(const char *path);
static int find_last_separator(const char *path, int start_idx = -1);
static result_t append_separator(char *path);
static result_t append_path(char *path, const char *name);

result_t init() {
  key_pressed.raw = 0;
  key_down.raw = 0;

  shown = false;
  tab = tab_t::NES_LIST;
  clear_file_list();
  for (int i = 0; i < BUFF_WIDTH * BUFF_HEIGHT; i++) {
    text_buff[i] = '\0';
    palette_buff[i] = 0x00;
  }
  return result_t::SUCCESS;
}
void deinit() { clear_file_list(); }

void show() {
  if (shown) return;
  shown = true;
  change_tab(tab, true);
}

void hide() {
  if (!shown) return;
  shown = false;
}

result_t update() {
  if (!shown) return result_t::SUCCESS;

  input::InputStatus prev_pressed = key_pressed;
  key_pressed = input::get_status(0);
  key_down.raw = key_pressed.raw & ~prev_pressed.raw;

  switch (tab) {
    default:
    case tab_t::NES_LIST: SHAPONES_TRY(update_nes_list()); break;
  }
  return result_t::SUCCESS;
}

static result_t update_nes_list() {
  if (!disk_mounted) refresh_file_list();

  if (num_files == 0) return result_t::SUCCESS;

  if (key_down.up) {
    selected_index = (selected_index + num_files - 1) % num_files;
    scroll_to(selected_index);
  } else if (key_down.down) {
    selected_index = (selected_index + 1) % num_files;
    scroll_to(selected_index);
  } else if (key_down.A) {
    if (0 <= selected_index && selected_index < num_files) {
      file_info_t &fi = file_list[selected_index];
      if (!fi.is_dir) {
        char path[nes::MAX_PATH_LENGTH];
        strncpy(path, current_dir, nes::MAX_PATH_LENGTH);
        SHAPONES_TRY(append_path(path, fi.name));
        SHAPONES_TRY(request_load_nes_file(path));
      } else {
        if (strcmp(fi.name, "..") == 0) {
          // parent directory
          int sep_idx = find_parent_separator(current_dir);
          if (sep_idx >= 0) {
            current_dir[sep_idx + 1] = '\0';
          }
        } else {
          // sub directory
          SHAPONES_TRY(append_path(current_dir, fi.name));
        }
        SHAPONES_TRY(append_separator(current_dir));
        refresh_file_list();
      }
    }
  }

  return result_t::SUCCESS;
}

result_t render(int y, uint8_t *line_buff) {
  if (!shown) return result_t::SUCCESS;

  int iy = y / CHAR_HEIGHT;
  if (iy < 0 || iy >= BUFF_HEIGHT) {
    return result_t::SUCCESS;
  }

  for (int ix = 0; ix < BUFF_WIDTH; ix++) {
    int x = ix * CHAR_WIDTH;
    uint8_t c = text_buff[iy * BUFF_WIDTH + ix];
    uint8_t p = palette_buff[iy * BUFF_WIDTH + ix] % NUM_PALETTES;
#if SHAPONES_MENU_LARGE_FONT
    const uint8_t code_first = FONT12X24_CODE_FIRST;
    const uint8_t code_last = FONT12X24_CODE_LAST;
#else
    const uint8_t code_first = FONT8X16_CODE_FIRST;
    const uint8_t code_last = FONT8X16_CODE_LAST;
#endif
    if (c < code_first || code_last < c) {
      continue;
    }

#if SHAPONES_MENU_LARGE_FONT
    uint32_t font_word =
        FONT12X24_DATA[((int)c - code_first) * CHAR_HEIGHT + (y % CHAR_HEIGHT)];
#else
    uint16_t font_word =
        FONT8X16_DATA[((int)c - code_first) * CHAR_HEIGHT + (y % CHAR_HEIGHT)];
#endif
    for (int ipix = 0; ipix < CHAR_WIDTH; ipix++) {
      int color_index = font_word & 0x03;
      font_word >>= 2;
      line_buff[x + ipix] = PALETTE_FILE[p * 4 + color_index];
    }
  }

  return result_t::SUCCESS;
}

static result_t change_tab(tab_t t, bool force) {
  if (tab == t && !force) return result_t::SUCCESS;

  tab = t;

  switch (tab) {
    case tab_t::NES_LIST: SHAPONES_TRY(refresh_file_list()); break;
  }

  return result_t::SUCCESS;
}

static void clear_file_list() {
  for (int i = 0; i < num_files; i++) {
    if (file_list[i].name) {
      delete[] file_list[i].name;
      file_list[i].name = nullptr;
    }
  }
  num_files = 0;
}

static result_t refresh_file_list() {
  clear_file_list();

  if (!disk_mounted) {
    if (fs_mount() == result_t::SUCCESS) {
      disk_mounted = true;
    } else {
      return result_t::SUCCESS;
    }
  }

  if (current_dir[0] == '\0') {
    SHAPONES_TRY(fs_get_current_dir(current_dir));
    SHAPONES_TRY(append_separator(current_dir));
  }

  if (!is_root_dir(current_dir)) {
    // add parent directory entry
    file_list[0].is_dir = true;
    file_list[0].name = strdup_safe("..");
    num_files = 1;
  }

  result_t res = fs_enum_files(current_dir, [](const file_info_t &info) {
    file_list[num_files] = info;
    file_list[num_files].name = strdup_safe(info.name);
    num_files++;
    return (num_files < MAX_NUM_FILES);
  });
  if (res != result_t::SUCCESS) {
    num_files = 0;
  }

  // sort by name
  for (int i = 0; i < num_files - 1; i++) {
    file_info_t &fi = file_list[i];
    for (int j = i + 1; j < num_files; j++) {
      file_info_t &fj = file_list[j];
      bool swap = false;
      if (fi.is_dir != fj.is_dir) {
        swap = fj.is_dir;
      } else {
        swap = strcmp(fi.name, fj.name) > 0;
      }
      if (swap) {
        file_info_t temp = file_list[i];
        file_list[i] = file_list[j];
        file_list[j] = temp;
      }
    }
  }

  list_scroll_y = 0;
  selected_index = 0;

  print_file_list();

  return res;
}

static void scroll_to(int index) {
  if (index < list_scroll_y) {
    list_scroll_y = index;
  } else if (index >= list_scroll_y + LIST_LINES) {
    list_scroll_y = index - LIST_LINES + 1;
  }
  print_file_list();
}

static void print_file_list() {
  // current directory
  fill_text(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1);
  print_text(CLIENT_X, CLIENT_Y, current_dir, CLIENT_WIDTH);
  fill_palette(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1, 0);

  // file list area
  {
    const int TEXT_END_X = CLIENT_X + CLIENT_WIDTH - 1;
    for (int iy = 0; iy < LIST_LINES; iy++) {
      file_info_t &fi = file_list[list_scroll_y + iy];
      int y = CLIENT_Y + 1 + iy;
      int i = list_scroll_y + iy;
      fill_text(CLIENT_X, y, CLIENT_WIDTH - 1, 1);
      if (0 <= i && i < num_files) {
        int x = CLIENT_X;
        if (!fi.is_dir) {
          x += print_text(x, y, "\xA8\xA9");  // file icon
        } else if (strcmp(fi.name, "..") == 0) {
          x += print_text(x, y, "\xA4\xA5");  // parent folder icon
        } else {
          x += print_text(x, y, "\xA6\xA7");  // folder icon
        }
        x += 1;
        x += print_text(x, y, fi.name, TEXT_END_X - x);
        if (x > TEXT_END_X - 1) x = TEXT_END_X - 1;
        if (fi.is_dir) {
          print_text(x, y, "/");
        }
      }
      if (i == selected_index) {
        fill_palette(CLIENT_X, y, CLIENT_WIDTH - 1, 1, 1);  // highlight color
      } else {
        fill_palette(CLIENT_X, y, CLIENT_WIDTH - 1, 1, 0);  // normal color
      }
    }
  }
  // scroll bar
  {
    int x = CLIENT_X + CLIENT_WIDTH - 1;
    int y = CLIENT_Y + 1;
    int p = 0;
    if (num_files > LIST_LINES) {
      int n = (num_files - LIST_LINES);
      p = (list_scroll_y * (LIST_LINES - 3) + (n - 1)) / n;
    }
    fill_palette(x, y, LIST_LINES, 0);
    print_char(x, y, 0xA0);                   // up arrow
    print_char(x, y + LIST_LINES - 1, 0xA1);  // down arrow
    fill_text(x, y + 1, 1, LIST_LINES - 2, 0xA2);
    print_char(x, y + 1 + p, 0xA3);  // scroll bar
  }
}

static int print_text(int x, int y, const char *str, int max_len) {
  int n = 0;
  char c;
  while (n < max_len && (c = str[n++]) != '\0') {
    print_char(x++, y, c);
  }
  return n - 1;
}

static void print_char(int x, int y, char c) {
  if (0 <= y && y < BUFF_HEIGHT && 0 <= x && x < BUFF_WIDTH) {
    text_buff[y * BUFF_WIDTH + x] = c;
  }
}

static void fill_text(int x, int y, int w, int h, char c) {
  clip_rect(&x, &y, &w, &h);
  for (int iy = 0; iy < h; iy++) {
    for (int ix = 0; ix < w; ix++) {
      text_buff[(y + iy) * BUFF_WIDTH + (x + ix)] = c;
    }
  }
}

static void fill_palette(int x, int y, int w, int h, uint8_t p) {
  clip_rect(&x, &y, &w, &h);
  for (int iy = 0; iy < h; iy++) {
    for (int ix = 0; ix < w; ix++) {
      palette_buff[(y + iy) * BUFF_WIDTH + (x + ix)] = p;
    }
  }
}

static void clip_rect(int *x, int *y, int *w, int *h) {
  if (*x < 0) {
    *w += *x;
    *x = 0;
  }
  if (*x + *w > BUFF_WIDTH) {
    *w = BUFF_WIDTH - *x;
  }
  if (*y < 0) {
    *h += *y;
    *y = 0;
  }
  if (*y + *h > BUFF_HEIGHT) {
    *h = BUFF_HEIGHT - *y;
  }
}

static bool is_root_dir(const char *path) {
  return find_parent_separator(path) <= 0;
}

static int find_parent_separator(const char *path) {
  int n = strnlen(path, nes::MAX_PATH_LENGTH);
  if (n == 0) {
    return -1;
  }
  if (path[n - 1] == '/') {
    n--;
  }
  return find_last_separator(path, n);
}

static int find_last_separator(const char *path, int start_idx) {
  if (start_idx < 0) {
    start_idx = strnlen(path, nes::MAX_PATH_LENGTH);
  } else if (start_idx == 0) {
    return -1;
  }
  for (int i = start_idx - 1; i >= 0; i--) {
    if (path[i] == '/') {
      return i;
    }
  }
  return -1;
}

static result_t append_separator(char *path) {
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  if (path[len - 1] == '/') {
    return result_t::SUCCESS;
  }
  if (len + 1 >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }
  path[len++] = '/';
  path[len] = '\0';
  return result_t::SUCCESS;
}

static result_t append_path(char *path, const char *name) {
  SHAPONES_TRY(append_separator(path));
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  int name_len = strnlen(name, nes::MAX_PATH_LENGTH);
  if (len + name_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  strcat(path, name);
  return result_t::SUCCESS;
}

}  // namespace nes::menu
