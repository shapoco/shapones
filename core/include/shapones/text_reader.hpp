#ifndef SHAPONES_TEXT_READER_HPP
#define SHAPONES_TEXT_READER_HPP

#include "shapones/common.hpp"
#include "shapones/fsys.hpp"
#include "shapones/host_intf.hpp"

namespace shapones::fsys {

class TextReader {
 private:
  void *handle;
  char peeked_char;
  result_t error = result_t::SUCCESS;

 public:
  TextReader(void *file_handle) : handle(file_handle) { read(); }

  bool eof() { return peeked_char == '\0'; }

  char peek() { return peeked_char; }

  char read() {
    char ret = peeked_char;
    if (fsys::eof(handle)) {
      peeked_char = '\0';
    } else {
      char c;
      error = fsys::read(handle, (uint8_t *)&c, 1);
      if (error == result_t::SUCCESS) {
        peeked_char = c;
      } else {
        peeked_char = '\0';
      }
    }
    return ret;
  }

  bool read_if(char c) {
    if (peeked_char == c) {
      read();
      return true;
    } else {
      return false;
    }
  }

  result_t expect(char c) {
    if (read_if(c)) {
      return result_t::SUCCESS;
    } else {
      return result_t::ERR_INI_PARSE_FAILED;
    }
  }

  void skip_whitespace() {
    while (read_if(' ') || read_if('\t')) {
    }
  }

  bool read_if_newline() {
    if (read_if('\n') || read_if('\r')) {
      skip_newline();
      return true;
    } else {
      return false;
    }
  }

  result_t expect_newline() {
    if (read_if_newline()) {
      return result_t::SUCCESS;
    } else {
      return result_t::ERR_INI_PARSE_FAILED;
    }
  }

  void skip_newline() {
    while (read_if_newline()) {
    }
  }
};
}  // namespace shapones::fsys

#endif