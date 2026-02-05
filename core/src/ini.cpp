#include "shapones/ini.hpp"
#include "shapones/text_reader.hpp"

namespace nes::ini {

enum class state_t {
  LINE_START,
  SECTION_NAME,
  SECTION_END,
  KEY_NAME,
  KEY_VALUE,
};

static char section[MAX_SECTION_LENGTH + 1] = {0};
static char key[MAX_KEY_LENGTH + 1] = {0};
static char value[MAX_VALUE_LENGTH + 1] = {0};

static result_t read_line(fsys::TextReader &reader, enum_keys_cb_t callback);
static result_t read_section_following(fsys::TextReader &reader);
static result_t read_key(fsys::TextReader &reader, enum_keys_cb_t callback);
static bool is_id_char(char c);

result_t read(const char *path, enum_keys_cb_t callback) {
  result_t res = result_t::SUCCESS;

  void *handle = nullptr;
  SHAPONES_TRY(fsys::open(path, false, &handle));

  fsys::TextReader reader(handle);

  state_t state = state_t::LINE_START;
  section[0] = '\0';

  while (!reader.eof()) {
    res = read_line(reader, callback);
    if (res != result_t::SUCCESS) {
      break;
    }
  }

  fsys::close(handle);
  return res;
}

static result_t read_line(fsys::TextReader &reader, enum_keys_cb_t callback) {
  reader.skip_whitespace();

  if (reader.read_if(';') || reader.read_if('#')) {
    while (!reader.eof() || !reader.read_if_newline()) {
      reader.read();
    }
    return result_t::SUCCESS;
  } else if (reader.read_if('[')) {
    return read_section_following(reader);
  } else if (is_id_char(reader.peek())) {
    return read_key(reader, callback);
  } else if (reader.read_if_newline() || reader.eof()) {
    return result_t::SUCCESS;
  } else {
    return result_t::ERR_INI_PARSE_FAILED;
  }
}

static result_t read_section_following(fsys::TextReader &reader) {
  int len = 0;
  while (is_id_char(reader.peek())) {
    char c = reader.read();
    if (len >= MAX_SECTION_LENGTH) {
      return result_t::ERR_INI_PARSE_FAILED;
    }
    section[len++] = c;
  }
  section[len] = '\0';
  if (len == 0) {
    return result_t::ERR_INI_PARSE_FAILED;
  }
  SHAPONES_TRY(reader.expect(']'));
  reader.skip_whitespace();
  SHAPONES_TRY(reader.expect_newline());
  return result_t::SUCCESS;
}

static result_t read_key(fsys::TextReader &reader, enum_keys_cb_t callback) {
  char key[MAX_KEY_LENGTH + 1] = {0};
  int key_len = 0;
  while (is_id_char(reader.peek())) {
    char c = reader.read();
    if (key_len >= MAX_KEY_LENGTH) {
      return result_t::ERR_INI_PARSE_FAILED;
    }
    key[key_len++] = c;
  }
  key[key_len] = '\0';
  if (key_len == 0) {
    return result_t::ERR_INI_PARSE_FAILED;
  }

  reader.skip_whitespace();
  SHAPONES_TRY(reader.expect('='));
  reader.skip_whitespace();

  char value[MAX_VALUE_LENGTH + 1] = {0};
  int value_len = 0;
  while (!reader.eof() && !reader.read_if_newline()) {
    char c = reader.read();
    if (value_len >= MAX_VALUE_LENGTH) {
      return result_t::ERR_INI_PARSE_FAILED;
    }
    value[value_len++] = c;
  }
  value[value_len] = '\0';

  callback(section, key, value);
  return result_t::SUCCESS;
}

static bool is_id_char(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') || c == '_';
}

}  // namespace nes::ini