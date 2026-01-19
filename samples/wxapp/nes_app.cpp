#include <wx/wx.h>

#include "nes_audio.hpp"
#include "nes_screen.hpp"
#include "shapones/shapones.hpp"
#include "shapones/utils_std.hpp"

enum { ID_FCFRAME = wxID_HIGHEST, ID_FCSCREEN, ID_TIMER };

class FcFrame : public wxFrame {
 public:
  FcFrame(const wxString &title);
  void OnMenuQuit(wxCommandEvent &event);
  void OnTimer(wxTimerEvent &event);
  void OnClose(wxCloseEvent &event);

 private:
  wxMenuBar *menubar;
  FcScreen *screen;
  wxTimer *timer;
  DECLARE_EVENT_TABLE()
};

FcFrame::FcFrame(const wxString &title) : wxFrame(NULL, ID_FCFRAME, title) {
  menubar = new wxMenuBar();
  wxMenu *mFile = new wxMenu();
  mFile->Append(wxID_EXIT, wxT("Quit"));
  menubar->Append(mFile, wxT("File"));
  SetMenuBar(menubar);

  timer = new wxTimer(this, ID_TIMER);
  timer->Start(16);

  screen = new FcScreen(this, ID_FCSCREEN);

  SetTitle(title);
  SetClientSize(wxSize(nes::SCREEN_WIDTH * 2, nes::SCREEN_HEIGHT * 2));
  CenterOnScreen();

  nes_audio::play();
}

BEGIN_EVENT_TABLE(FcFrame, wxFrame)
EVT_MENU(wxID_EXIT, FcFrame::OnMenuQuit)
EVT_TIMER(ID_TIMER, FcFrame::OnTimer)
EVT_CLOSE(FcFrame::OnClose)
END_EVENT_TABLE()

void FcFrame::OnMenuQuit(wxCommandEvent &event) { Close(); }

void FcFrame::OnTimer(wxTimerEvent &event) { screen->Render(); }

void FcFrame::OnClose(wxCloseEvent &event) {
  nes_audio::stop();
  event.Skip();
}

class FcApp : public wxApp {
 public:
  FcFrame *frame;
  virtual bool OnInit();
};

bool FcApp::OnInit() {
  frame = new FcFrame(wxT("ShapoNES"));

  nes::load_ines_file(wxApp::argv[1]);

  auto cfg = nes::get_default_config();
  cfg.apu_sampling_rate = nes_audio::FREQ_HZ;
  nes::init(cfg);

  frame->Show(true);

  return (true);
}

DECLARE_APP(FcApp)
IMPLEMENT_APP(FcApp)

// Exclusive control is not required because it is single-threaded
void nes::lock_init(int id) {}
void nes::lock_deinit(int id) {}
void nes::lock_get(int id) {}
void nes::lock_release(int id) {}
