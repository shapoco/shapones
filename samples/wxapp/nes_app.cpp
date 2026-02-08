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
  SetClientSize(wxSize(shapones::SCREEN_WIDTH * 2, shapones::SCREEN_HEIGHT * 2));
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
  shapones::deinit();
  event.Skip();
}

class FcApp : public wxApp {
 public:
  FcFrame *frame;
  virtual bool OnInit();
};

bool FcApp::OnInit() {
  shapones::result_t res = shapones::result_t::SUCCESS;

  frame = new FcFrame(wxT("ShapoNES"));

  auto cfg = shapones::get_default_config();
  cfg.apu_sampling_rate = nes_audio::FREQ_HZ;
  shapones::init(cfg);

  bool loaded = false;
  if (wxApp::argc >= 2) {
    do {
      const uint8_t *ines_data = nullptr;
      size_t ines_size = 0;
      res = shapones::load_ines(wxApp::argv[1], &ines_data, &ines_size);
      if (res != shapones::result_t::SUCCESS) {
        break;
      }

      res = shapones::map_ines(ines_data, wxApp::argv[1]);
      if (res != shapones::result_t::SUCCESS) {
        break;
      }

      loaded = true;
    } while (0);
  }

  if (!loaded) {
    shapones::menu::show();
  }

  frame->Show(true);

  return (true);
}

DECLARE_APP(FcApp)
IMPLEMENT_APP(FcApp)

shapones::result_t shapones::ram_alloc(size_t size, void **out_ptr) {
  void *ptr = malloc(size);
  if (!ptr) {
    return shapones::result_t::ERR_RAM_ALLOC_FAILED;
  }
  *out_ptr = ptr;
  return shapones::result_t::SUCCESS;
}

void shapones::ram_free(void *ptr) { free(ptr); }

// Exclusive control is not required because it is single-threaded
shapones::result_t shapones::spinlock_init(int id) { return shapones::result_t::SUCCESS; }
void shapones::spinlock_deinit(int id) {}
void shapones::spinlock_get(int id) {}
void shapones::spinlock_release(int id) {}

shapones::result_t shapones::semaphore_init(int id) { return shapones::result_t::SUCCESS; }
void shapones::semaphore_deinit(int id) {}
void shapones::semaphore_take(int id) {}
bool shapones::semaphore_try_take(int id) { return true; }
void shapones::semaphore_give(int id) {}

shapones::result_t shapones::load_ines(const char *path, const uint8_t **out_ines,
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
    return shapones::result_t::ERR_FS_READ_FAILED;
  }
  return shapones::result_t::SUCCESS;
}

void shapones::unload_ines() {
  SHAPONES_PRINTF("Unloading iNES file\n");
  ines_image.clear();
}

uint64_t shapones::get_time_us() {
  auto now = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count();
  return static_cast<uint64_t>(us);
}
