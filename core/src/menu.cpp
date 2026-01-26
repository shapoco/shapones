#include "shapones/menu.hpp"
#include "shapones/font8x16.hpp"
#include "shapones/fs.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/state.hpp"

namespace nes::menu {

static constexpr int CHAR_WIDTH = 8;
static constexpr int CHAR_HEIGHT = 16;
static constexpr int BUFF_WIDTH = SCREEN_WIDTH / CHAR_WIDTH;
static constexpr int BUFF_HEIGHT = SCREEN_HEIGHT / CHAR_HEIGHT;

static constexpr int CLIENT_X = 1;
static constexpr int CLIENT_Y = 2;
static constexpr int CLIENT_WIDTH = BUFF_WIDTH - 2;
static constexpr int CLIENT_HEIGHT = BUFF_HEIGHT - 3;

static constexpr int MENU_MAX_ITEMS = 256;

static constexpr int POPUP_MAX_ITEMS = 4;
static constexpr int POPUP_MAX_MESSAGE_LENGTH = 64;
static constexpr int POPUP_WIDTH = BUFF_WIDTH - 8;
static constexpr int POPUP_X = (BUFF_WIDTH - POPUP_WIDTH) / 2;
static constexpr int POPUP_Y = (BUFF_HEIGHT - POPUP_MAX_ITEMS - 3) / 2;

static constexpr int NUM_TABS = (int)tab_t::COUNT;

enum class icon_t {
  NONE,
  PARENT,
  FOLDER,
  FILE,
  ADD,
  INFO,
  ERROR,
  PLAY,
  SAVE,
};

enum class action_t {
  OPEN_DIR,
  LOAD_ROM,
  ADD_STATE_SLOT,
  STATE_SELECT,
  LOAD_STATE,
  SAVE_STATE,
  CLOSE_POPUP,
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

static void request_redraw();

class ListItem {
 public:
  const icon_t icon;
  const action_t action;
  const char *label;
  const int32_t tag;
  ListItem(icon_t icon, action_t action, const char *label, int32_t tag = 0)
      : icon(icon), action(action), label(strdup_safe(label)), tag(tag) {}
  ~ListItem() {
    if (label != nullptr) {
      delete[] label;
      label = nullptr;
    }
  }
};

class ListBox {
 public:
  const int capacity;
  ListItem **items;
  int num_items;
  int sel_index;
  int scroll_pos;
  int x, y, width, height;

  ListBox(int capacity)
      : capacity(capacity),
        items(new ListItem *[capacity]),
        num_items(0),
        sel_index(0),
        scroll_pos(0) {}

  ~ListBox() {
    for (int i = 0; i < num_items; i++) {
      delete items[i];
    }
    delete[] items;
  }

  void set_bounds(int x, int y, int w, int h) {
    this->x = x;
    this->y = y;
    this->width = w;
    this->height = h;
  }

  ListItem *get_selected_item() const {
    if (0 <= sel_index && sel_index < num_items) {
      return items[sel_index];
    } else {
      return nullptr;
    }
  }

  void clear() {
    for (int i = 0; i < num_items; i++) {
      delete items[i];
    }
    num_items = 0;
    sel_index = 0;
    scroll_pos = 0;
    request_redraw();
  }

  void add_item(icon_t icon, action_t action, const char *label,
                int32_t tag = 0) {
    if (num_items >= capacity) return;
    items[num_items++] = new ListItem(icon, action, label, tag);
    request_redraw();
  }

  void on_key_down(input::status_t key) {
    if (key.up) {
      if (num_items > 0) {
        sel_index = (sel_index + num_items - 1) % num_items;
        scroll_to(sel_index);
        request_redraw();
      }
    } else if (key.down) {
      if (num_items > 0) {
        sel_index = (sel_index + 1) % num_items;
        scroll_to(sel_index);
        request_redraw();
      }
    }
  }

  void scroll_to(int index) {
    if (index < scroll_pos) {
      scroll_pos = index;
      request_redraw();
    } else if (index >= scroll_pos + height) {
      scroll_pos = index - height + 1;
      request_redraw();
    }
  }
};

bool shown = false;
tab_t tab = tab_t::NES_LIST;
int last_menu_index[NUM_TABS] = {0};
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

bool redraw_requested = false;

ListBox menu(MENU_MAX_ITEMS);

icon_t popup_icon = icon_t::NONE;
char popup_message[POPUP_MAX_MESSAGE_LENGTH + 1] = "";
ListBox popup_list(POPUP_MAX_ITEMS);
bool popup_shown = false;

uint8_t ss_buff[state::SS_SIZE_BYTES];
bool ss_enable = false;

static result_t load_tab(tab_t tab, bool force = false);
static result_t load_file_list_tab();
static result_t load_state_list_tab();
static result_t load_state_screenshot();

static int find_empty_slot();

static result_t process_input();
static result_t on_menu_selected(ListItem *item);

static result_t on_open_dir(ListItem *mi);
static result_t on_load_rom(ListItem *mi);
static result_t on_add_state_slot();
static result_t on_state_select(ListItem *mi);
static result_t on_save_state(ListItem *mi);
static result_t on_load_state(ListItem *mi);

static result_t show_message(icon_t icon, const char *msg);

static void menu_scroll_to(int index);

static void popup_show(icon_t icon, const char *msg);
static void popup_close();

static void perform_redraw();
static int draw_text(int x, int y, const char *str, int max_len = 999999);
static void draw_char(int x, int y, char c);
static void draw_icon(int x, int y, icon_t icon);
static void draw_frame(int x, int y, int w, int h, char offset);
static void draw_list(const ListBox &list);
static void fill_char(int x, int y, int w, int h, char c = ' ');
static void fill_palette(int x, int y, int w, int h, uint8_t p = 0x00);
static void clip_rect(int *x, int *y, int *w, int *h);

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
void deinit() { menu.clear(); }

void show() {
  if (shown) return;
  shown = true;
  load_tab(tab, true);
}

void hide() {
  if (!shown) return;
  popup_close();
  menu.clear();
  shown = false;
}

static result_t load_tab(tab_t t, bool force) {
  if (tab == t && !force) return result_t::SUCCESS;

  tab = t;

  switch (tab) {
    case tab_t::NES_LIST:
      menu.set_bounds(CLIENT_X, CLIENT_Y + 1, CLIENT_WIDTH, CLIENT_HEIGHT - 1);
      SHAPONES_TRY(load_file_list_tab());
      break;
    case tab_t::SAVE_LIST:
      menu.set_bounds(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, CLIENT_HEIGHT);
      SHAPONES_TRY(load_state_list_tab());
      break;
  }

  int i = last_menu_index[static_cast<int>(tab)];
  if (0 <= i && i < menu.num_items) {
    menu.sel_index = i;
  }

  return result_t::SUCCESS;
}

static result_t load_file_list_tab() {
  menu.clear();

  if (!disk_mounted) {
    if (fs_mount() == result_t::SUCCESS) {
      disk_mounted = true;
    } else {
      return result_t::SUCCESS;
    }
  }

  if (current_dir[0] == '\0') {
    SHAPONES_TRY(fs_get_current_dir(current_dir));
    SHAPONES_TRY(fs::append_separator(current_dir));
  }

  if (!fs::is_root_dir(current_dir)) {
    // add parent directory entry
    menu.add_item(icon_t::PARENT, action_t::OPEN_DIR, "../");
  }

  result_t res = fs_enum_files(current_dir, [](const file_info_t &info) {
    char *name = (char *)info.name;
    if (info.is_dir) {
      // append '/' to directory names
      size_t len = strnlen(name, nes::MAX_FILENAME_LENGTH + 1);
      char name[nes::MAX_FILENAME_LENGTH + 2];
      strncpy(name, info.name, nes::MAX_FILENAME_LENGTH);
      name[len++] = '/';
      name[len] = '\0';
    }
    icon_t icon = info.is_dir ? icon_t::FOLDER : icon_t::FILE;
    action_t action = info.is_dir ? action_t::OPEN_DIR : action_t::LOAD_ROM;
    menu.add_item(icon, action, name);
    return (menu.num_items < menu.capacity);
  });
  if (res != result_t::SUCCESS) {
    menu.clear();
    return res;
  }

  // sort by name
  for (int i = 0; i < menu.num_items - 1; i++) {
    for (int j = i + 1; j < menu.num_items; j++) {
      const ListItem *mi = menu.items[i];
      const ListItem *mj = menu.items[j];
      bool swap = false;
      if (mi->icon != mj->icon) {
        swap = mi->icon > mj->icon;
      } else {
        swap = strcmp(mi->label, mj->label) > 0;
      }
      if (swap) {
        ListItem *temp = menu.items[i];
        menu.items[i] = menu.items[j];
        menu.items[j] = temp;
      }
    }
  }

  ss_enable = false;
  request_redraw();
  return res;
}

static result_t load_state_list_tab() {
  menu.clear();
  char state_path[nes::MAX_PATH_LENGTH + 1];
  strncpy(state_path, get_ines_path(), nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(fs::replace_ext(state_path, state::STATE_FILE_EXT));

  if (nes::fs_exists(state_path)) {
    state::enum_slots(state_path, [](const state::state_slot_entry_t &entry) {
      if (!entry.is_used()) return true;
      char label[nes::MAX_FILENAME_LENGTH + 1];
      uint32_t t = entry.play_time_sec;
      if (t < 60) {
        snprintf(label, sizeof(label), "#%02d %ds", entry.index, t);
      } else if (t < 3600) {
        snprintf(label, sizeof(label), "#%02d %dm", entry.index, t / 60);
      } else {
        snprintf(label, sizeof(label), "#%02d %.1fh", entry.index,
                 (float)t / 3600);
      }
      menu.add_item(icon_t::FILE, action_t::STATE_SELECT, label, entry.index);
      return (menu.num_items < menu.capacity);
    });
  }

  if (menu.num_items < state::MAX_SLOTS) {
    menu.add_item(icon_t::ADD, action_t::ADD_STATE_SLOT, "New Slot", -1);
  }

  load_state_screenshot();

  request_redraw();
  return result_t::SUCCESS;
}

static result_t load_state_screenshot() {
  char state_path[nes::MAX_PATH_LENGTH + 1];
  strncpy(state_path, get_ines_path(), nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(fs::replace_ext(state_path, state::STATE_FILE_EXT));

  ss_enable = false;
  auto *mi = menu.get_selected_item();
  if (mi && mi->tag >= 0) {
    result_t res = state::read_screenshot(state_path, mi->tag, ss_buff);
    ss_enable = (res == result_t::SUCCESS);
  }
  return result_t::SUCCESS;
}

result_t service() {
  if (!vsync) return result_t::SUCCESS;
  vsync = false;

  if (!shown) return result_t::SUCCESS;

  input::status_t prev_pressed = key_pressed;
  key_pressed = input::get_status(0);
  key_down.raw = key_pressed.raw & ~prev_pressed.raw;
  key_up.raw = ~key_pressed.raw & prev_pressed.raw;

  process_input();
  if (redraw_requested) {
    redraw_requested = false;
    perform_redraw();
  }

  return result_t::SUCCESS;
}

static int find_empty_slot() {
  uint64_t used_flags = 0;
  for (int i = 0; i < menu.num_items; i++) {
    if (menu.items[i]->tag >= 0) {
      used_flags |= (1 << menu.items[i]->tag);
    }
  }
  for (int i = 0; i < state::MAX_SLOTS; i++) {
    if ((used_flags & (1 << i)) == 0) {
      return i;
    }
  }
  return -1;
}

static result_t process_input() {
  if (key_up.A) {
    ListItem *mi = nullptr;
    if (popup_shown) {
      mi = popup_list.get_selected_item();
    } else {
      mi = menu.get_selected_item();
    }
    if (mi) {
      on_menu_selected(mi);
    }
  } else if (key_down.select) {
    if (!popup_shown) {
      tab_t new_tab =
          static_cast<tab_t>((static_cast<int>(tab) + 1) % NUM_TABS);
      load_tab(new_tab);
    }
  } else if (popup_shown) {
    popup_list.on_key_down(key_down);
  } else {
    int prev_sel = menu.sel_index;
    menu.on_key_down(key_down);
    int new_sel = menu.sel_index;
    if (tab == tab_t::SAVE_LIST && prev_sel != new_sel) {
      load_state_screenshot();
    }
    last_menu_index[static_cast<int>(tab)] = menu.sel_index;
  }
  return result_t::SUCCESS;
}

static result_t on_menu_selected(ListItem *mi) {
  result_t res = result_t::SUCCESS;
  switch (mi->action) {
    case action_t::OPEN_DIR: res = on_open_dir(mi); break;
    case action_t::LOAD_ROM: res = on_load_rom(mi); break;
    case action_t::ADD_STATE_SLOT: res = on_add_state_slot(); break;
    case action_t::STATE_SELECT: res = on_state_select(mi); break;
    case action_t::LOAD_STATE: res = on_load_state(mi); break;
    case action_t::SAVE_STATE: res = on_save_state(mi); break;
    case action_t::CLOSE_POPUP: popup_close(); break;
  }

  if (res != result_t::SUCCESS) {
    show_message(icon_t::ERROR, result_to_string(res));
  }

  return res;
}

static result_t on_open_dir(ListItem *mi) {
  if (strcmp(mi->label, "../") == 0) {
    // parent directory
    int sep_idx = fs::find_parent_separator(current_dir);
    if (sep_idx >= 0) {
      current_dir[sep_idx + 1] = '\0';
    }
  } else {
    // sub directory
    SHAPONES_TRY(fs::append_path(current_dir, mi->label));
  }
  SHAPONES_TRY(fs::append_separator(current_dir));
  load_file_list_tab();
  return result_t::SUCCESS;
}

static result_t on_load_rom(ListItem *mi) {
  char path[nes::MAX_PATH_LENGTH + 1];
  strncpy(path, current_dir, nes::MAX_PATH_LENGTH);
  SHAPONES_TRY(fs::append_path(path, mi->label));
  SHAPONES_TRY(request_load_nes_file(path));
  return result_t::SUCCESS;
}

static result_t on_add_state_slot() {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  int slot = find_empty_slot();
  if (slot < 0) return result_t::ERR_STATE_SLOT_FULL;
  state::save(path, slot);
  load_state_list_tab();
  return result_t::SUCCESS;
}

static result_t on_state_select(ListItem *mi) {
  popup_list.clear();
  popup_message[0] = '\0';
  popup_list.add_item(icon_t::PLAY, action_t::LOAD_STATE, "Load", mi->tag);
  popup_list.add_item(icon_t::SAVE, action_t::SAVE_STATE, "Save", mi->tag);
  popup_list.add_item(icon_t::NONE, action_t::CLOSE_POPUP, "Cancel");
  popup_show(icon_t::NONE, "Action?");
  return result_t::SUCCESS;
}

static result_t on_save_state(ListItem *mi) {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  SHAPONES_TRY(state::save(path, mi->tag));
  popup_close();
  return result_t::SUCCESS;
}

static result_t on_load_state(ListItem *mi) {
  char path[nes::MAX_PATH_LENGTH + 1];
  SHAPONES_TRY(state::get_state_path(path, nes::MAX_PATH_LENGTH));
  SHAPONES_TRY(state::load(path, mi->tag));
  popup_close();
  hide();
  return result_t::SUCCESS;
}

static result_t show_message(icon_t icon, const char *msg) {
  popup_list.clear();
  popup_list.add_item(icon_t::NONE, action_t::CLOSE_POPUP, "Close");
  popup_show(icon, msg);
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

  if (ss_enable && !popup_shown) {
    // scren shot
    int ss_x = SCREEN_WIDTH - CHAR_WIDTH * 2 - state::SS_WIDTH - 2;
    int ss_y = SCREEN_HEIGHT - CHAR_HEIGHT * 2 - state::SS_HEIGHT - 2;
    if (ss_y <= y && y < ss_y + state::SS_HEIGHT) {
      int sy = y - ss_y;
      memcpy(&line_buff[ss_x], &ss_buff[sy * state::SS_WIDTH], state::SS_WIDTH);
    }
  }

  return result_t::SUCCESS;
}

static void popup_show(icon_t icon, const char *msg) {
  popup_icon = icon;
  strncpy(popup_message, msg, POPUP_MAX_MESSAGE_LENGTH);

  popup_shown = true;
  int msg_w = strnlen(msg, POPUP_MAX_MESSAGE_LENGTH) + 4;
  int list_w = msg_w;
  for (int i = 0; i < popup_list.num_items; i++) {
    auto *mi = popup_list.items[i];
    int item_w = strnlen(mi->label, POPUP_MAX_MESSAGE_LENGTH) + 4;
    if (item_w > list_w) {
      list_w = item_w;
    }
  }
  int list_x = (BUFF_WIDTH - list_w) / 2;

  int frame_h = popup_list.num_items + 2;
  int message_h = 0;
  if (msg_w > 0) {
    message_h = 1;
  }
  frame_h += message_h;
  int frame_y = (BUFF_HEIGHT - frame_h) / 2;
  int list_y = frame_y + 1 + message_h;
  popup_list.set_bounds(list_x, list_y, list_w, popup_list.num_items);
  request_redraw();
}

static void popup_close() {
  popup_list.clear();
  popup_shown = false;
  request_redraw();
}

static void request_redraw() { redraw_requested = true; }

static void perform_redraw() {
  fill_char(0, 0, BUFF_WIDTH, 1, ' ');
  draw_frame(0, 1, BUFF_WIDTH, BUFF_HEIGHT - 1, '\x80');

  // tab bar
  draw_text(1, 1, "\x88\x89");
  for (int itab = 0; itab < NUM_TABS; itab++) {
    for (int i = 0; i < 3; i++) {
      char c = '\xA0' + itab * 3 + i;
      if (itab == (int)tab) {
        c += 0x10;
      }
      draw_char(3 + itab * 3 + i, 1, c);
    }
  }

  if (tab == tab_t::NES_LIST) {
    // current directory
    fill_char(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1);
    draw_text(CLIENT_X, CLIENT_Y, current_dir, CLIENT_WIDTH);
    fill_palette(CLIENT_X, CLIENT_Y, CLIENT_WIDTH, 1, 2);
  }

  // file list area
  draw_list(menu);

  // popup
  if (popup_shown) {
    int message_h = 0;
    if (popup_message[0] != '\0') {
      message_h = 1;
    }
    draw_frame(popup_list.x - 1, popup_list.y - 1 - message_h,
               popup_list.width + 2, popup_list.height + 2 + message_h, '\x90');
    if (message_h > 0) {
      fill_char(popup_list.x, popup_list.y - 1, popup_list.width, 1);
      int icon_w = 0;
      if (popup_icon != icon_t::NONE) {
        icon_w = 3;
        draw_icon(popup_list.x, popup_list.y - 1, popup_icon);
      }
      draw_text(popup_list.x + icon_w, popup_list.y - 1, popup_message,
                popup_list.width - icon_w);
      fill_palette(popup_list.x, popup_list.y - 1, popup_list.width, 1, 0);
    }
    draw_list(popup_list);
  }
}

static int draw_text(int x, int y, const char *str, int max_len) {
  int n = 0;
  char c;
  while (n < max_len && (c = str[n++]) != '\0') {
    draw_char(x++, y, c);
  }
  return n - 1;
}

static void draw_char(int x, int y, char c) {
  if (0 <= y && y < BUFF_HEIGHT && 0 <= x && x < BUFF_WIDTH) {
    text_buff[y * BUFF_WIDTH + x] = c;
  }
}

static void draw_icon(int x, int y, icon_t icon) {
  const char *text = nullptr;
  switch (icon) {
    case icon_t::PARENT: text = "\xC0\xC1"; break;
    case icon_t::FOLDER: text = "\xC2\xC3"; break;
    case icon_t::FILE: text = "\xC4\xC5"; break;
    case icon_t::ADD: text = "\xC8\xC9"; break;
    case icon_t::SAVE: text = "\xCA\xCB"; break;
    case icon_t::PLAY: text = "\xCC\xCD"; break;
  }
  if (text) draw_text(x, y, text);
}

static void draw_frame(int x, int y, int w, int h, char offset) {
  // top border
  draw_char(x, y, offset + 0);
  fill_char(x + 1, y, w - 2, 1, offset + 1);
  draw_char(x + w - 1, y, offset + 2);
  fill_palette(x, y, w, 2, 0);

  // side borders
  fill_char(x, y + 1, 1, h - 2, offset + 3);
  fill_palette(x, y + 1, 1, h - 2, 0);
  fill_char(x + w - 1, y + 1, 1, h - 2, offset + 4);
  fill_palette(x + w - 1, y + 1, 1, h - 2, 0);

  // bottom border
  draw_char(x, y + h - 1, offset + 5);
  fill_char(x + 1, y + h - 1, w - 2, 1, offset + 6);
  draw_char(x + w - 1, y + h - 1, offset + 7);
  fill_palette(x, y + h - 1, w, 1, 0);
}

static void draw_list(const ListBox &list) {
  int x = list.x;
  int y = list.y;
  int w = list.width;
  int h = list.height;
  int item_w = list.width;

  if (list.num_items > list.height) {
    // scroll bar
    item_w = list.width - 1;
    int list_right = CLIENT_X + CLIENT_WIDTH - 1;
    int button_pos = 0;
    if (list.num_items > h) {
      int n = (list.num_items - h);
      button_pos = (list.scroll_pos * (h - 3) + (n - 1)) / n;
    }
    fill_palette(list_right, y, h, 0);
    draw_char(list_right, y, '\x8C');          // up arrow
    draw_char(list_right, y + h - 1, '\x8D');  // down arrow
    fill_char(list_right, y + 1, 1, h - 2, '\x8E');
    draw_char(list_right, y + 1 + button_pos, '\x8F');  // scroll bar
  }

  for (int iy = 0; iy < h; iy++) {
    int i = iy + list.scroll_pos;
    fill_char(x, y + iy, item_w, 1, ' ');
    if (0 <= i && i < list.num_items) {
      auto *mi = list.items[i];
      if (mi->icon != icon_t::NONE) {
        draw_icon(x, y + iy, mi->icon);
      }
      draw_text(x + 3, y + iy, mi->label, item_w - 3);
    }
    if (i == list.sel_index) {
      fill_palette(x, y + iy, item_w, 1, 1);  // highlight color
    } else {
      fill_palette(x, y + iy, item_w, 1, 0);  // normal color
    }
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

}  // namespace nes::menu
