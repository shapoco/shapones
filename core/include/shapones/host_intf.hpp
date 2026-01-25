#ifndef SHAPONES_HOST_INTF_HPP
#define SHAPONES_HOST_INTF_HPP

#include "shapones/common.hpp"

namespace nes {

struct file_info_t {
  bool is_dir;
  const char *name;
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
bool fs_exists(const char *path);
result_t fs_open(const char *path, bool write, void **handle);
void fs_close(void *handle);
result_t fs_seek(void *handle, int offset);
result_t fs_read(void *handle, uint8_t *buff, int size);
result_t fs_write(void *handle, const uint8_t *buff, int size);
result_t fs_size(void *handle, size_t *out_size);

result_t request_load_nes_file(const char *path);

}  // namespace nes

#endif
