#include "shapones/fs.hpp"

namespace nes::fs {

bool is_root_dir(const char *path) { return find_parent_separator(path) < 0; }

int find_parent_separator(const char *path) {
  int n = strnlen(path, nes::MAX_PATH_LENGTH);
  if (n == 0) {
    return -1;
  }
  if (path[n - 1] == '/') {
    n--;
  }
  return find_char_rev(path, '/', n);
}

int find_char_rev(const char *path, char c, int start_idx) {
  if (start_idx < 0) {
    start_idx = strnlen(path, nes::MAX_PATH_LENGTH);
  } else if (start_idx == 0) {
    return -1;
  }
  for (int i = start_idx - 1; i >= 0; i--) {
    if (path[i] == c) {
      return i;
    }
  }
  return -1;
}

result_t append_separator(char *path) {
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  if (path[len - 1] == '/') {
    return result_t::SUCCESS;
  }
  if (len + 1 >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }
  path[len++] = '/';
  path[len] = '\0';
  return result_t::SUCCESS;
}

result_t append_path(char *path, const char *name) {
  SHAPONES_TRY(append_separator(path));
  int len = strnlen(path, nes::MAX_PATH_LENGTH);
  int name_len = strnlen(name, nes::MAX_PATH_LENGTH);
  if (len + name_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  strcat(path, name);
  return result_t::SUCCESS;
}

result_t replace_ext(char *path, const char *new_ext) {
  int old_path_len = strnlen(path, nes::MAX_PATH_LENGTH);

  int sep_idx = find_char_rev(path, '/');
  int old_name_len = old_path_len;
  if (sep_idx >= 0) {
    old_name_len -= (sep_idx + 1);
  }

  int dot_idx = find_char_rev(path, '.');
  if (dot_idx < 0 || dot_idx < sep_idx) {
    dot_idx = old_path_len;
  }

  int old_ext_len = old_path_len - dot_idx - 1;
  int new_ext_len = strnlen(new_ext, nes::MAX_FILENAME_LENGTH);

  int new_path_len = old_path_len - old_ext_len + new_ext_len;
  if (new_path_len >= nes::MAX_PATH_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  int new_name_len = old_name_len - old_ext_len + new_ext_len;
  if (new_name_len >= nes::MAX_FILENAME_LENGTH) {
    return result_t::ERR_PATH_TOO_LONG;
  }

  path[dot_idx] = '.';
  strcpy(&path[dot_idx + 1], new_ext);
  return result_t::SUCCESS;
}

}  // namespace nes::fs