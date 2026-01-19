#include "common.hpp"

uint8_t frame_buff[FRAME_BUFF_STRIDE * nes::SCREEN_HEIGHT];

const int input_pins[] = {PIN_PAD_A,     PIN_PAD_B,    PIN_PAD_SELECT,
                          PIN_PAD_START, PIN_PAD_UP,   PIN_PAD_DOWN,
                          PIN_PAD_LEFT,  PIN_PAD_RIGHT};

int wait_key() {
  // wait all button released
  int num_pushed;
  do {
    sleep_ms(10);
    num_pushed = 0;
    for (int i = 0; i < 8; i++) {
      if (!gpio_get(input_pins[i])) {
        num_pushed++;
      }
    }
  } while (num_pushed != 0);

  // wait any button pushed
  for (;;) {
    sleep_ms(10);
    for (int i = 0; i < 8; i++) {
      if (!gpio_get(input_pins[i])) {
        return i;
      }
    }
  }
}

void draw_string(int x, int y, const char *str) {
  mono8x16::draw_string_rgb444(frame_buff, FRAME_BUFF_STRIDE, FRAME_BUFF_WIDTH,
                               FRAME_BUFF_HEIGHT, x, y, str, 0xfff);
}

void clear_frame_buff() {
  for (int i = 0; i < FRAME_BUFF_STRIDE * FRAME_BUFF_HEIGHT; i++) {
    frame_buff[i] = 0;
  }
}

void update_lcd() {
  ws19804::start_write_data(25, 0, FRAME_BUFF_WIDTH, FRAME_BUFF_HEIGHT,
                            frame_buff);
  ws19804::finish_write_data();
}
