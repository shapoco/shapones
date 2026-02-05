#ifndef SHAPONES_INI_HPP
#define SHAPONES_INI_HPP

#include "shapones/common.hpp"
#include "shapones/fsys.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/text_reader.hpp"

namespace nes::ini {

static constexpr uint32_t MAX_SECTION_LENGTH = 16;
static constexpr uint32_t MAX_KEY_LENGTH = 16;
static constexpr uint32_t MAX_VALUE_LENGTH = 256;

using enum_keys_cb_t = bool (*)(const char *section, const char *key,
                                const char *value);

result_t read(const char *path, enum_keys_cb_t callback);

}  // namespace nes::ini

#endif
