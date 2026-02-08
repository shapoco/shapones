#include <string.h>
#include <filesystem>
#include <fstream>
#include "shapones/shapones.hpp"

namespace fs = std::filesystem;

namespace shapones::fsys {

result_t mount() { return result_t::SUCCESS; }
void unmount() {}

void get_ines_dir(char *out_path) {
  std::string path = "/";
  try {
    path = fs::current_path().string();
  } catch (...) {
  }
  strncpy(out_path, path.c_str(), MAX_PATH_LENGTH);
  out_path[MAX_PATH_LENGTH] = '\0';
}

void get_config_dir(char *out_path) {
  std::string path = "/";
  try {
    path = fs::current_path().string();
  } catch (...) {
  }
  snprintf(out_path, MAX_PATH_LENGTH, "%s/.shapones", path.c_str());
  out_path[MAX_PATH_LENGTH] = '\0';
}

result_t enum_files(const char *path,
                                    enum_files_cb_t callback) {
  try {
    for (const auto &entry : fs::directory_iterator(path)) {
      file_info_t info;
      info.is_dir = entry.is_directory();
      std::string filename = entry.path().filename().string();
      info.name = filename.c_str();
      if (!callback(info)) break;
    }
  } catch (...) {
    return result_t::ERR_FS_DIR_NOT_FOUND;
  }
  return result_t::SUCCESS;
}

bool exists(const char *path) { return fs::exists(path); }

result_t open(const char *path, bool write, void **handle) {
  bool create = !fs::exists(path);

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
    return result_t::ERR_FS_OPEN_FAILED;
  }
  *handle = static_cast<void *>(fs);
  return result_t::SUCCESS;
}

void close(void *handle) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (fs) {
    fs->close();
    delete fs;
  }
}

result_t seek(void *handle, size_t offset) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return result_t::ERR_FS_OPEN_FAILED;
  }
  fs->seekg(offset, std::ios::beg);
  fs->seekp(offset, std::ios::beg);
  return result_t::SUCCESS;
}

bool eof(void *handle) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return true;
  }
  return fs->eof();
}

result_t size(void *handle, size_t *out_size) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return result_t::ERR_FS_FILE_NOT_OPEN;
  }
  std::streampos current_pos = fs->tellg();
  fs->seekg(0, std::ios::end);
  std::streampos end_pos = fs->tellg();
  fs->seekg(current_pos, std::ios::beg);
  *out_size = static_cast<size_t>(end_pos);
  return result_t::SUCCESS;
}

result_t read(void *handle, uint8_t *buff, size_t size) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return result_t::ERR_FS_FILE_NOT_OPEN;
  }
  fs->read(reinterpret_cast<char *>(buff), size);
  return result_t::SUCCESS;
}

result_t write(void *handle, const uint8_t *buff, size_t size) {
  std::fstream *fs = static_cast<std::fstream *>(handle);
  if (!fs || !fs->is_open()) {
    return result_t::ERR_FS_FILE_NOT_OPEN;
  }
  fs->write(reinterpret_cast<const char *>(buff), size);
  if (fs->bad()) {
    return result_t::ERR_FS_WRITE_FAILED;
  }
  return result_t::SUCCESS;
}

result_t remove(const char *path) {
  try {
    fs::remove(path);
  } catch (...) {
    return result_t::ERR_FS_DELETE_FAILED;
  }
  return result_t::SUCCESS;
}

}
