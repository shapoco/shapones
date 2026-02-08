#ifndef SHAPONES_FS_HPP
#define SHAPONES_FS_HPP

#include "shapones/common.hpp"

namespace shapones::fsys {

struct file_info_t {
  bool is_dir;
  const char *name;
};

using enum_files_cb_t = bool (*)(const file_info_t &info);

bool is_root_dir(const char *path);
int find_parent_separator(const char *path);
int find_char_rev(const char *path, char c, int start_idx = -1);
result_t append_separator(char *path);
result_t append_path(char *path, const char *name);
result_t replace_ext(char *path, const char *new_ext);

}  // namespace shapones::fsys

#endif
