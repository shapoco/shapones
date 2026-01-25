#include "shapones/menu.hpp"
#include "shapones/font8x16.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/state.hpp"

namespace nes::menu {

static constexpr int MAX_MENU_ITEMS = 1024;
static constexpr int CHAR_WIDTH = 8;
static constexpr int CHAR_HEIGHT = 16;
static constexpr int BUFF_WIDTH = SCREEN_WIDTH / CHAR_WIDTH;
static constexpr int BUFF_HEIGHT = SCREEN_HEIGHT / CHAR_HEIGHT;

static constexpr int CLIENT_X = 1;
static constexpr int CLIENT_Y = 2;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH - 2;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT - 4;

static constexpr int NUM_TABS = (int)tab_t::COUNT;

enum class icon_t {
  PARENT_DIR,
  FOLDER,
  FILE,
  ADD,
};

// Workaround for the issue where repeated use of strdup in PicoLibSDK causes a
// panic:
static char *strdup_safe(const char *src) {
  if (src == nullptr) return nullptr;
  size_t len = strlen(src);
  char *dst = new char[len + 1];
  strcpy(dst, src);
  return dst;
}

struct MenuItem {
 public:
  const icon_t icon;
  const char *label;
  MenuItem(icon_t icon, const char *label)
      : icon(icon), label(strdup_safe(label)) {}
  ~MenuItem() {
    if (label != nullptr) {
      delete[] label;
      label = nullptr;
    }
  }
};

bool shown = false;
tab_t tab = tab_t::NES_LIST;
volatile bool vsync = false;

const uint8_t PALETTE_FILE[] = {0x1D, 0x00, 0x10, 0x20, 0x21, 0x11, 0x01, 0x3F,
                                0x2D, 0x00, 0x10, 0x3D, 0x3F, 0x0C, 0x1C, 0x2C};
static constexpr int NUM_PALETTES = sizeof(PALETTE_FILE) / 4;

uint8_t text_buff[BUFF_WIDTH * BUFF_HEIGHT];
uint8_t palette_buff[BUFF_WIDTH * BUFF_HEIGHT];

input::status_t key_pressed;
input::status_t key_down;
input::status_t key_up;

bool disk_mounted = false;
char current_dir[nes::MAX_PATH_LENGTH + 1] = "";
MenuItem *menu_items[MAX_MENU_ITEMS];
int list_top = CLIENT_Y;
int list_height = CLIENT_HEIGHT;
int list_scroll_y = 0;
int selected_index = 0;
int num_menu_items = 0;

static constexpr int STATUS_TEXT_MAX_LENGTH = BUFF_WIDTH;
char status_text[STATUS_TEXT_MAX_LENGTH + 1] = "";

static result_t change_tab(tab_t tab, bool force = false);
static result_t service_nes_list();
static result_t service_state_list();
static result_t process_default_key_input();
static void clear_menu_list();
static result_t load_file_list();
static result_t load_state_list();
static void draw_list();
static void scroll_to(int index);
static int print_text(int x, int y, const char *str, int max_len = 999999);
static void print_char(int x, int y, char c);
static void fill_char(int x, int y, int w, int h, char c = ' ');
static void fill_palette(int x, int y, int w, int h, uint8_t p = 0x00);
static void clip_rect(int *x, int *y, int *w, int *h);
static bool is_root_dir(const char *path);
static int find_parent_separator(const char *path);
static int find_char_rev(const char *path, char c, int start_idx = -1);
static result_t append_separator(char *path);
static result_t append_path(char *path, const char *name);
static result_t replace_ext(char *path, const char *new_ext);

result_t init() {
  deinit();
  key_pressed.raw = 0;
  key_down.raw = 0;
  key_up.raw = 0;

  shown = false;
  tab = tab_t::NES_LIST;
  for (int i = 0; i < BUFF_WIDTH * BUFF_HEIGHT; i++) {
    text_buff[i] = '\0';
    palette_buff[i] = 0x00;
  }
  return result_t::SUCCESS;
}
void deinit() { clear_menu_list(); }

void show() {
  if (shown) return;
  shown = true;
  change_tab(tab, true);
}

void hide() {
  if (!shown) return;
  shown = false;
}

result_t service() {
  if (!vsync) return result_t::SUCCESS;
  vsync = false;

  if (!shown) return result_t::SUCCESS;

  input::status_t prev_pressed = key_pressed;
  key_pressed = input::get_status(0);
  key_down.raw = key_pressed.raw & ~prev_pressed.raw;
  key_up.raw = ~key_pressed.raw & prev_pressed.raw;

  switch (tab) {
    case tab_t::NES_LIST: service_nes_list(); break;
    case tab_t::SAVE_LIST: service_state_list(); break;
  }

  fill_char(0, 0, BUFF_WIDTH, 1, ' ');
  print_char(0, 1, '\x80');
  fill_char(1, 1, BUFF_WIDTH - 2, 1, '\x81');
  print_char(BUFF_WIDTH - 1, 1, '\x82');
  fill_palette(0, 0, BUFF_WIDTH, 2, 0);

  print_text(1, 1, "\x88\x89");
  for (int itab = 0; itab < NUM_TABS; itab++) {
    for (int i = 0; i < 3; i++) {
      char c = '\x90' + itab * 3 + i;
      if (itab == (int)tab) {
        c += 0x10;
      }
      print_char(3 + itab * 3 + i, 1, c);
    }
  }

  fill_char(0, 2, 1, BUFF_HEIGHT - 3, '\x83');
  fill_palette(0, 2, 1, BUFF_HEIGHT - 3, 0);

  fill_char(BUFF_WIDTH - 1, 2, 1, BUFF_HEIGHT - 3, '\x84');
  fill_palette(BUFF_WIDTH - 1, 2, 1, BUFF_HEIGHT - 3, 0);

  print_char(0, BUFF_HEIGHT - 1, '\x85');
  fill_char(1, BUFF_HEIGHT - 1, BUFF_WIDTH - 2, 1, '\x86');
  print_char(BUFF_WIDTH - 1, BUFF_HEIGHT - 1, '\x87');
  fill_palette(0, BUFF_HEIGHT - 1, BUFF_WIDTH, 1, 0);

  fill_char(CLIENT_X, CLIENT_Y + CLIENT_HEIGHT, CLIENT_WIDTH, 1);
  print_text(CLIENT_X, CLIENT_Y + CLIENT_HEIGHT, status_text, CLIENT_WIDTH);
  fill_palette(CLIENT_X, CLIENT_Y + CLIENT_HEIGHT, CLIENT_WIDTH, 1, 2);

  return result_t::SUCCESS;
}

static result_t service_nes_list() {
  if (key_up.A) {
    if (0 <= selected_index && selected_index < num_menu_items) {
      const MenuItem *mi = menu_items[selected_index];
      int n = strnlen(mi->label, nes::MAX_FILENAME_LENGTH + 1);
      bool is_dir = (n > 0 && mi->label[n - 1] == '/');
      if (!is_dir) {
        char path[nes::MAX_PATH_LENGTH + 1];
        strncpy(path, current_dir, nes::MAX_PATH_LENGTH);
        SHAPONES_TRY(append_path(path, mi->label));
        SHAPONES_TRY(request_load_nes_file(path));
      } else {
        if (strcmp(mi->label, "../") == 0) {
          // parent directory
          int sep_idx = find_parent_separator(current_dir);
          if (sep_idx >= 0) {
            current_dir[sep_idx + 1] = '\0';
          }
        } else {
          // sub directory
          SHAPONES_TRY(append_path(current_dir, mi->label));
        }
        SHAPONES_TRY(append_separator(current_dir));
        load_file_list();
      }
    }
  } else {
    return process_default_key_input();
  }
  return result_t::SUCCESS;
}

static result_t service_state_list() {
  if (key_up.A || key_up.B) {
    if (0 <= selected_index && selected_index < num_menu_items) {
      auto *mi = menu_items[selected_index];
      char state_path[nes::MAX_PATH_LENGTH + 1];
      strncpy(state_path, get_ines_path(), nes::MAX_PATH_LENGTH);
      SHAPONES_TRY(replace_ext(state_path, STATE_FILE_EXT));
      if (key_up.A) {
        SHAPONES_TRY(load_state(state_path, selected_index));
      } else {
        SHAPONES_TRY(save_state(state_path, selected_index));
      }
    }
  } else {
    return process_default_key_input();
  }
  return result_t::SUCCESS;
}

static result_t process_default_key_input() {
  if (key_down.select) {
    tab_t new_tab = static_cast<tab_t>((static_cast<int>(tab) + 1) % NUM_TABS);
    change_tab(new_tab);
  } else if (key_down.up) {
    if (num_menu_items > 0) {
      selected_index = (selected_index + num_menu_items - 1) % num_menu_items;
      scroll_to(selected_index);
    }
  } else if (key_down.down) {
    if (num_menu_items > 0) {
      selected_index = (selected_index + 1) % num_menu_items;
      scroll_to(selected_index);
    }
  }
  return result_t::SUCCESS;
}

result_t overlay(int y, uint8_t *line_buff) {
  if (y == SCREEN_HEIGHT - 1) {
    vsync = true;
  }

  if (!shown) return result_t::SUCCESS;

  int iy = y / CHAR_HEIGHT;
  if (iy < 0 || iy >= BUFF_HEIGHT) {
    return result_t::SUCCESS;
  }

  for (int ix = 0; ix < BUFF_WIDTH; ix++) {
    int x = ix * CHAR_WIDTH;
    uint8_t c = text_buff[iy * BUFF_WIDTH + ix];
    uint8_t p = palette_buff[iy * BUFF_WIDTH + ix] % NUM_PALETTES;
    if (c < FONT8X16_CODE_FIRST || FONT8X16_CODE_LAST < c) {
      continue;
    }

    uint16_t word = FONT8X16_DATA[((int)c - FONT8X16_CODE_FIRST) * CHAR_HEIGHT +
                                  (y % CHAR_HEIGHT)];
    for (int ipix = 0; ipix < CHAR_WIDTH; ipix++) {
      int color_index = word & 0x03;
      word >>= 2;
      uint8_t color = PALETTE_FILE[p * 4 + color_index];
      if (color < 0x40) line_buff[x + ipix] = color;
    }
  }

  return result_t::SUCCESS;
}

static result_t change_tab(tab_t t, bool force) {
  if (tab == t && !force) return result_t::SUCCESS;

  tab = t;
  list_top = CLIENT_Y;
  list_height = CLIENT_HEIGHT;

  switch (tab) {
    case tab_t::NES_LIST:
      list_top++;
      list_height--;
      strncpy(status_text, "[A]Open", STATUS_TEXT_MAX_LENGTH);
      SHAPONES_TRY(load_file_list());
      break;
    case tab_t::SAVE_LIST:
      strncpy(status_text, "[A]Load [B]Save", STATUS_TEXT_MAX_LENGTH);
      SHAPONES_TRY(load_state_list());
      break;
  }

  return result_t::SUCCESS;
}

static void clear_menu_list() {
  for (int i = 0; i < num_menu_items; i++) {
    delete menu_items[i];
  }
  num_menu_items = 0;
}

static result_t load_file_list() {
  clear_menu_list();

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
    menu_items[0] = new MenuItem(icon_t::PARENT_DIR, "../");
    num_menu_items = 1;
  }

  result_t res = fs_enum_files(current_dir, [](const file_info_t &info) {
    char *name = (char *)info.name;
    if (info.is_dir) {
      // append '/' to directory names
      size_t len = strnlen(name, nes::MAX_FILENAME_LENGTH + 1);
      name = new char[len + 2];
      strcpy(name, info.name);
      name[len++] = '/';
      name[len] = '\0';
    }
    icon_t icon = info.is_dir ? icon_t::FOLDER : icon_t::FILE;
    menu_items[num_menu_items] = new MenuItem(icon, name);
    num_menu_items++;
    return (num_menu_items < MAX_MENU_ITEMS);
  });
  if (res != result_t::SUCCESS) {
    clear_menu_list();
    return res;
  }

  // sort by name
  for (int i = 0; i < num_menu_items - 1; i++) {
    for (int j = i + 1; j < num_menu_items; j++) {
      const MenuItem *mi = menu_items[i];
      const MenuItem *mj = menu_items[j];
      bool swap = false;
      if (mi->icon != mj->icon) {
        swap = mi->icon > mj->icon;
      } else {
        swap = strcmp(mi->label, mj->label) > 0;
      }
      if (swap) {
        MenuItem *temp = menu_items[i];
        menu_items[i] = menu_items[j];
        menu_items[j] = temp;
      }
    }
  }

  list_scroll_y = 0;
  selected_index = 0;

  draw_list();

  return res;
}

static result_t load_state_list() {
  clear_menu_list();

  char state_path[nes::MAX_PATH_LENGTH + 1];
  strncpy(state_path, get_ines_path(), nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(replace_ext(state_path, STATE_FILE_EXT));

  if (nes::fs_exists(state_path)) {
    enum_state_slots(state_path, [](const state_slot_entry_t &entry) {
      char label[nes::MAX_FILENAME_LENGTH + 1];
      snprintf(label, sizeof(label), "Slot #%d", entry.index);
      menu_items[num_menu_items++] = new MenuItem(icon_t::FILE, label);
      return (num_menu_items < MAX_MENU_ITEMS);
    });
  }

  if (num_menu_items < STATE_MAX_SLOTS) {
    menu_items[num_menu_items++] = new MenuItem(icon_t::ADD, "New Slot");
  }

  draw_list();
  return result_t::SUCCESS;
}

static void scroll_to(int index) {
  if (index < list_scroll_y) {
    list_scroll_y = index;
  } else if (index >= list_scroll_y + list_height) {
    list_scroll_y = index - list_height + 1;
  }
  draw_list();
}

static void draw_list() {
  if (tab == tab_t::NES_LIST) {
    // current directory
    fill_char(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1);
    print_text(CLIENT_X, CLIENT_Y, current_dir, CLIENT_WIDTH);
    fill_palette(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1, 2);
  }

  // file list area
  const int TEXT_END_X = CLIENT_X + CLIENT_WIDTH - 1;
  for (int iy = 0; iy < list_height; iy++) {
    const MenuItem *mi = menu_items[list_scroll_y + iy];
    int y = list_top + iy;
    int i = list_scroll_y + iy;
    fill_char(CLIENT_X, y, CLIENT_WIDTH - 1, 1);
    if (0 <= i && i < num_menu_items) {
      int x = CLIENT_X;
      const char *icon = nullptr;
      switch (mi->icon) {
        case icon_t::PARENT_DIR: icon = "\xB4\xB5"; break;
        case icon_t::FOLDER: icon = "\xB6\xB7"; break;
        case icon_t::FILE: icon = "\xB8\xB9"; break;
        case icon_t::ADD: icon = "\xBC\xBD"; break;
      }
      if (icon) print_text(x, y, icon);
      x += 3;
      if (mi->icon == icon_t::PARENT_DIR) {
        x += print_text(x, y, "(Parent)", TEXT_END_X - x);
      } else {
        x += print_text(x, y, mi->label, TEXT_END_X - x);
      }
    }
    if (i == selected_index && num_menu_items > 0) {
      fill_palette(CLIENT_X, y, CLIENT_WIDTH - 1, 1, 1);  // highlight color
    } else {
      fill_palette(CLIENT_X, y, CLIENT_WIDTH - 1, 1, 0);  // normal color
    }
  }

  // scroll bar
  int list_right = CLIENT_X + CLIENT_WIDTH - 1;
  int button_pos = 0;
  if (num_menu_items > list_height) {
    int n = (num_menu_items - list_height);
    button_pos = (list_scroll_y * (list_height - 3) + (n - 1)) / n;
  }
  fill_palette(list_right, list_top, list_height, 0);
  print_char(list_right, list_top, '\xB0');                    // up arrow
  print_char(list_right, list_top + list_height - 1, '\xB1');  // down arrow
  fill_char(list_right, list_top + 1, 1, list_height - 2, '\xB2');
  print_char(list_right, list_top + 1 + button_pos, '\xB3');  // scroll bar
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

static void fill_char(int x, int y, int w, int h, char c) {
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
  return find_parent_separator(path) < 0;
}

static int find_parent_separator(const char *path) {
  int n = strnlen(path, nes::MAX_PATH_LENGTH);
  if (n == 0) {
    return -1;
  }
  if (path[n - 1] == '/') {
    n--;
  }
  return find_char_rev(path, '/', n);
}

static int find_char_rev(const char *path, char c, int start_idx) {
  if (start_idx < 0) {
    start_idx = strnlen(path, nes::MAX_PATH_LENGTH);
  } else if (start_idx == 0) {
    return -1;
  }
  for (int i = start_idx - 1; i >= 0; i--) {
    if (path[i] == c) {
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

static result_t replace_ext(char *path, const char *new_ext) {
  int old_path_len = strnlen(path, nes::MAX_PATH_LENGTH);

  int sep_idx = find_char_rev(path, '/');
  int old_name_len = old_path_len;
  if (sep_idx >= 0) {
    old_name_len -= (sep_idx + 1);
  }

  int dot_idx = find_char_rev(path, '.');
  if (dot_idx < 0 || dot_idx < sep_idx) {
    dot_idx = old_path_len;
  }

  int old_ext_len = old_path_len - dot_idx - 1;
  int new_ext_len = strnlen(new_ext, nes::MAX_FILENAME_LENGTH);

  int new_path_len = old_path_len - old_ext_len + new_ext_len;
  if (new_path_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  int new_name_len = old_name_len - old_ext_len + new_ext_len;
  if (new_name_len >= nes::MAX_FILENAME_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  path[dot_idx] = '.';
  strcpy(&path[dot_idx + 1], new_ext);
  return result_t::SUCCESS;
}

}  // namespace nes::menu
