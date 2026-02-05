#include <string.h>
#include <chrono>
#include <filesystem>
#include <fstream>

#include <wx/wx.h>

#include "shapones/shapones.hpp"

#include "nes_audio.hpp"
#include "nes_screen.hpp"

enum { ID_FCFRAME = wxID_HIGHEST, ID_FCSCREEN, ID_TIMER };

std::vector<uint8_t> ines_image;

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
  nes::deinit();
  event.Skip();
}

class FcApp : public wxApp {
 public:
  FcFrame *frame;
  virtual bool OnInit();
};

bool FcApp::OnInit() {
  nes::result_t res = nes::result_t::SUCCESS;

  frame = new FcFrame(wxT("ShapoNES"));

  auto cfg = nes::get_default_config();
  cfg.apu_sampling_rate = nes_audio::FREQ_HZ;
  nes::init(cfg);

  bool loaded = false;
  if (wxApp::argc >= 2) {
    do {
      const uint8_t *ines_data = nullptr;
      size_t ines_size = 0;
      res = nes::load_ines(wxApp::argv[1], &ines_data, &ines_size);
      if (res != nes::result_t::SUCCESS) {
        break;
      }

      res = nes::map_ines(ines_data, wxApp::argv[1]);
      if (res != nes::result_t::SUCCESS) {
        break;
      }

      loaded = true;
    } while (0);
  }

  if (!loaded) {
    nes::menu::show();
  }

  frame->Show(true);

  return (true);
}

DECLARE_APP(FcApp)
IMPLEMENT_APP(FcApp)

nes::result_t nes::ram_alloc(size_t size, void **out_ptr) {
  void *ptr = malloc(size);
  if (!ptr) {
    return nes::result_t::ERR_RAM_ALLOC_FAILED;
  }
  *out_ptr = ptr;
  return nes::result_t::SUCCESS;
}

void nes::ram_free(void *ptr) { free(ptr); }

// Exclusive control is not required because it is single-threaded
nes::result_t nes::spinlock_init(int id) { return nes::result_t::SUCCESS; }
void nes::spinlock_deinit(int id) {}
void nes::spinlock_get(int id) {}
void nes::spinlock_release(int id) {}

nes::result_t nes::semaphore_init(int id) { return nes::result_t::SUCCESS; }
void nes::semaphore_deinit(int id) {}
void nes::semaphore_take(int id) {}
bool nes::semaphore_try_take(int id) { return true; }
void nes::semaphore_give(int id) {}

nes::result_t nes::load_ines(const char *path, const uint8_t **out_ines,
                             size_t *out_size) {
  try {
    std::ifstream ifs(path, std::ios::binary);

    ifs.seekg(0, std::ios::end);
    auto size = ifs.tellg();
    ifs.seekg(0);
    SHAPONES_PRINTF("Loading iNES file: %s (size: %zu bytes)\n", path,
                    (size_t)size);

    std::vector<uint8_t> vec(size);
    ifs.read((char *)&vec[0], size);
    ines_image = std::move(vec);

    *out_ines = &ines_image[0];
    *out_size = ines_image.size();

    SHAPONES_PRINTF("iNES file loaded\n");
  } catch (...) {
    SHAPONES_PRINTF("Failed to load iNES file\n");
    return nes::result_t::ERR_FS_READ_FAILED;
  }
  return nes::result_t::SUCCESS;
}

void nes::unload_ines() {
  SHAPONES_PRINTF("Unloading iNES file\n");
  ines_image.clear();
}

uint64_t nes::get_time_us() {
  auto now = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count();
  return static_cast<uint64_t>(us);
}
