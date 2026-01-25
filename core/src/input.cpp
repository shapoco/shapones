#include "shapones/input.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/menu.hpp"

namespace nes::input {

static constexpr uint32_t STATE_SIZE = 16;

static control_t reg;
static status_t raw[2];
static uint8_t shift_reg[2];

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  reg.raw = 0;
  raw[0].raw = 0;
  raw[1].raw = 0;
  shift_reg[0] = 0;
  shift_reg[1] = 0;
  return result_t::SUCCESS;
}

status_t get_status(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  return raw[player];
}

void set_status(int player, status_t s) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  raw[player] = s;
}

void update() {
  if (reg.strobe) {
    if (menu::is_shown()) {
      shift_reg[0] = 0;
      shift_reg[1] = 0;
    } else {
      shift_reg[0] = raw[0].raw;
      shift_reg[1] = raw[1].raw;
    }
  }
}

uint8_t read_latched(int player) {
  if (player < 0) {
    player = 0;
  } else if (player > 1) {
    player = 1;
  }
  uint8_t retval = shift_reg[player] & 1;
  shift_reg[player] >>= 1;
  return retval;
}

void write_control(uint8_t data) {
  reg.raw = data;
  update();
}

uint32_t state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  memset(buffer, 0, STATE_SIZE);
  uint8_t *p = buffer;
  BufferWriter writer(p);
  writer.u8(reg.raw);
  writer.u8(raw[0].raw);
  writer.u8(raw[1].raw);
  writer.u8(shift_reg[0]);
  writer.u8(shift_reg[1]);
  return fs_write(file_handle, buffer, STATE_SIZE);
}

result_t load_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buffer, STATE_SIZE));
  const uint8_t *p = buffer;
  BufferReader reader(p);
  reg.raw = reader.u8();
  raw[0].raw = reader.u8();
  raw[1].raw = reader.u8();
  shift_reg[0] = reader.u8();
  shift_reg[1] = reader.u8();
  return result_t::SUCCESS;
}

}  // namespace nes::input
