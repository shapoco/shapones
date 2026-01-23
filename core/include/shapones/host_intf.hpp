#ifndef SHAPONES_HOST_INTF_HPP
#define SHAPONES_HOST_INTF_HPP

#include "shapones/common.hpp"

namespace nes {

struct file_info_t {
  bool is_dir = false;
  char *name = nullptr;
};

using fs_enum_files_cb_t = bool (*)(const file_info_t &info);

result_t lock_init(int id);
void lock_deinit(int id);
void lock_get(int id);
void lock_release(int id);

result_t fs_mount();
void fs_unmount();

result_t fs_get_current_dir(char *out_path);
result_t fs_enum_files(const char *path, fs_enum_files_cb_t callback);

result_t request_load_nes_file(const char *path);

}  // namespace nes

#endif
