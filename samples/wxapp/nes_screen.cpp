#include <chrono>
#include "stdlib.h"

#include "shapones/shapones.hpp"

#include "nes_load.hpp"
#include "nes_screen.hpp"

static const uint32_t colors[] = {
    0x808080, 0x003DA6, 0x0012B0, 0x440096, 0xA1005E, 0xC70028, 0xBA0600,
    0x8C1700, 0x5C2F00, 0x104500, 0x054A00, 0x00472E, 0x004166, 0x000000,
    0x050505, 0x050505, 0xC7C7C7, 0x0077FF, 0x2155FF, 0x8237FA, 0xEB2FB5,
    0xFF2950, 0xFF2200, 0xD63200, 0xC46200, 0x358000, 0x058F00, 0x008A55,
    0x0099CC, 0x212121, 0x090909, 0x090909, 0xFFFFFF, 0x0FD7FF, 0x69A2FF,
    0xD480FF, 0xFF45F3, 0xFF618B, 0xFF8833, 0xFF9C12, 0xFABC20, 0x9FE30E,
    0x2BF035, 0x0CF0A4, 0x05FBFF, 0x5E5E5E, 0x0D0D0D, 0x0D0D0D, 0xFFFFFF,
    0xA6FCFF, 0xB3ECFF, 0xDAABEB, 0xFFA8F9, 0xFFABB3, 0xFFD2B0, 0xFFEFA6,
    0xFFF79C, 0xD7E895, 0xA6EDAF, 0xA2F2DA, 0x99FFFC, 0xDDDDDD, 0x111111,
    0x111111,
};

static double total_emu_time_ms = 0;
static int total_frames = 0;

static double get_time_ms() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return (double)ns / 1000000;
}

FcScreen::FcScreen(wxFrame *parent, wxWindowID id)
    : wxScrolledWindow(parent, id, wxDefaultPosition, wxDefaultSize,
                       wxNO_FULL_REPAINT_ON_RESIZE) {
  owner = parent;
  SetBackgroundColour(*wxBLACK);
  frame_buff = wxImage(nes::SCREEN_WIDTH * 2, nes::SCREEN_HEIGHT * 2);
}

BEGIN_EVENT_TABLE(FcScreen, wxScrolledWindow)
EVT_PAINT(FcScreen::OnPaint)
EVT_KEY_DOWN(FcScreen::OnKeyDown)
EVT_KEY_UP(FcScreen::OnKeyUp)
END_EVENT_TABLE();

void FcScreen::Render() {
  nes::result_t res;

  if (nes_path[0] != '\0') {
    res = load_nes_file(nes_path);
    nes_path[0] = '\0';
  }

  static int itvl = 0;
  bool upd = (((itvl++) & 0x3) == 0);
  uint8_t line_buff[nes::SCREEN_WIDTH];

  res = nes::vsync(line_buff);
  auto wr_ptr = frame_buff.GetData();
  for (int y = 0; y < nes::SCREEN_HEIGHT; y++) {
    auto t_start = get_time_ms();
    nes::render_next_line(line_buff);
    total_emu_time_ms += get_time_ms() - t_start;

    auto rd_ptr = line_buff;
    for (int x = 0; x < nes::SCREEN_WIDTH; x++) {
      int color_index = *(rd_ptr++) & 0x3f;
      uint32_t color = colors[color_index];
      *(wr_ptr++) = (color >> 16) & 0xff;
      *(wr_ptr++) = (color >> 8) & 0xff;
      *(wr_ptr++) = color & 0xff;
      *(wr_ptr++) = (color >> 16) & 0xff;
      *(wr_ptr++) = (color >> 8) & 0xff;
      *(wr_ptr++) = color & 0xff;
    }
    size_t line_length = 3 * 2 * nes::SCREEN_WIDTH;
    memcpy(wr_ptr, wr_ptr - line_length, line_length);
    wr_ptr += line_length;
  }

  total_frames++;

  Refresh();
  // if (upd) wxYield();
}

void FcScreen::OnPaint(wxPaintEvent &event) {
  wxPaintDC pdc(this);

  auto ptr = frame_buff.GetData();

  wxBitmap bmp = wxBitmap(frame_buff);
  pdc.DrawBitmap(bmp, 0, 0, false);

  char s[32];
  sprintf(s, "%6.3lfms / frame", total_emu_time_ms / total_frames);
  pdc.SetTextForeground(*wxWHITE);
  pdc.DrawText(wxString(s), 0, 0);
}

void FcScreen::OnKeyDown(wxKeyEvent &event) {
  auto input0 = nes::input::get_status(0);
  switch (event.GetKeyCode()) {
    case 'Z': input0.A = 1; break;
    case 'X': input0.B = 1; break;
    case 'C': input0.start = 1; break;
    case 'V': input0.select = 1; break;
    case wxKeyCode::WXK_UP: input0.up = 1; break;
    case wxKeyCode::WXK_DOWN: input0.down = 1; break;
    case wxKeyCode::WXK_LEFT: input0.left = 1; break;
    case wxKeyCode::WXK_RIGHT: input0.right = 1; break;
    case 'Q':
      if (nes::menu::is_shown()) {
        nes::menu::hide();
      } else {
        nes::menu::show();
      }
      break;
  }
  nes::input::set_status(0, input0);
}

void FcScreen::OnKeyUp(wxKeyEvent &event) {
  auto input0 = nes::input::get_status(0);
  switch (event.GetKeyCode()) {
    case 'Z': input0.A = 0; break;
    case 'X': input0.B = 0; break;
    case 'C': input0.start = 0; break;
    case 'V': input0.select = 0; break;
    case wxKeyCode::WXK_UP: input0.up = 0; break;
    case wxKeyCode::WXK_DOWN: input0.down = 0; break;
    case wxKeyCode::WXK_LEFT: input0.left = 0; break;
    case wxKeyCode::WXK_RIGHT: input0.right = 0; break;
    case wxKeyCode::WXK_ESCAPE: owner->Close(); break;
  }
  nes::input::set_status(0, input0);
}
