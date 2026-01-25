#include <string.h>
#include <filesystem>
#include <fstream>

#include <wx/wx.h>

#include "shapones/shapones.hpp"

#include "nes_audio.hpp"
#include "nes_load.hpp"
#include "nes_screen.hpp"

namespace fs = std::filesystem;

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
  nes::deinit();
  event.Skip();
}

class FcApp : public wxApp {
 public:
  FcFrame *frame;
  virtual bool OnInit();
};

bool FcApp::OnInit() {
  frame = new FcFrame(wxT("ShapoNES"));

  auto cfg = nes::get_default_config();
  cfg.apu_sampling_rate = nes_audio::FREQ_HZ;
  nes::init(cfg);

  if (wxApp::argc >= 2) {
    nes::result_t res = load_nes_file(wxApp::argv[1]);
    if (res != nes::result_t::SUCCESS) {
      wxMessageBox("Failed to load NES file.", "Error", wxOK | wxICON_ERROR);
    }
  } else {
    nes::menu::show();
  }

  frame->Show(true);

  return (true);
}

DECLARE_APP(FcApp)
IMPLEMENT_APP(FcApp)

// Exclusive control is not required because it is single-threaded
nes::result_t nes::lock_init(int id) { return nes::result_t::SUCCESS; }
void nes::lock_deinit(int id) {}
void nes::lock_get(int id) {}
void nes::lock_release(int id) {}

nes::result_t nes::fs_mount() { return nes::result_t::SUCCESS; }
void nes::fs_unmount() {}

nes::result_t nes::fs_get_current_dir(char *out_path) {
  try {
    std::string path = fs::current_path().string();
    strncpy(out_path, path.c_str(), nes::MAX_PATH_LENGTH);
    out_path[nes::MAX_PATH_LENGTH] = '\0';
  } catch (...) {
    return nes::result_t::ERR_DIR_NOT_FOUND;
  }
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_enum_files(const char *path,
                                 nes::fs_enum_files_cb_t callback) {
  try {
    for (const auto &entry : fs::directory_iterator(path)) {
      nes::file_info_t info;
      info.is_dir = entry.is_directory();
      std::string filename = entry.path().filename().string();
      info.name = filename.c_str();
      if (!callback(info)) break;
    }
  } catch (...) {
    return nes::result_t::ERR_DIR_NOT_FOUND;
  }
  return nes::result_t::SUCCESS;
}

bool nes::fs_exists(const char *path) { return fs::exists(path); }

nes::result_t nes::fs_open(const char *path, bool write, void **handle) {
  bool create = !fs_exists(path);

  std::fstream *fs = new std::fstream();
  std::ios::openmode mode = std::ios::binary;
  if (write) {
    mode |= std::ios::in | std::ios::out;
    if (create) mode |= std::ios::trunc;
  } else {
    mode |= std::ios::in;
  }
  fs->open(path, mode);
  if (!fs->is_open()) {
    delete fs;
    return nes::result_t::ERR_FAILED_TO_OPEN_FILE;
  }
  *handle = static_cast<void *>(fs);
  return nes::result_t::SUCCESS;
}

void nes::fs_close(void *handle) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (fs) {
    fs->close();
    delete fs;
  }
}

nes::result_t nes::fs_seek(void *handle, int offset) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return nes::result_t::ERR_FAILED_TO_OPEN_FILE;
  }
  fs->seekg(offset, std::ios::beg);
  fs->seekp(offset, std::ios::beg);
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_size(void *handle, size_t *out_size) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return nes::result_t::ERR_FILE_NOT_OPEN;
  }
  std::streampos current_pos = fs->tellg();
  fs->seekg(0, std::ios::end);
  std::streampos end_pos = fs->tellg();
  fs->seekg(current_pos, std::ios::beg);
  *out_size = static_cast<size_t>(end_pos);
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_read(void *handle, uint8_t *buff, int size) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return nes::result_t::ERR_FILE_NOT_OPEN;
  }
  fs->read(reinterpret_cast<char *>(buff), size);
  return nes::result_t::SUCCESS;
}

nes::result_t nes::fs_write(void *handle, const uint8_t *buff, int size) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return nes::result_t::ERR_FILE_NOT_OPEN;
  }
  fs->write(reinterpret_cast<const char *>(buff), size);
  if (fs->bad()) {
    return nes::result_t::ERR_FAILED_TO_WRITE_FILE;
  }
  return nes::result_t::SUCCESS;
}

nes::result_t nes::request_load_nes_file(const char *path) {
  strncpy(nes_path, path, nes::MAX_PATH_LENGTH);
  nes_path[nes::MAX_PATH_LENGTH] = '\0';
  return nes::result_t::SUCCESS;
}
