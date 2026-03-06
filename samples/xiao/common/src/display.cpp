#include "shapones/xiao/display.hpp"
#include "shapones/xiao/gpio.h"
#include "shapones/xiao/timer.h"
#include "shapones/xiao/spi.h"
#include "shapones/xiao/pins.h"
#include "shapones/xiao/ioex.hpp"

namespace shapones::xiao::display {

enum class command_t : uint8_t {
  NOP = 0x00,
  SOFTWARE_RESET = 0x01,
  READ_DISP_ID = 0x04,
  READ_ERROR_DSI = 0x05,
  READ_DISP_STATUS = 0x09,
  READ_DISP_POWER_MODE = 0x0A,
  READ_DISP_MADCTRL = 0x0B,
  READ_DISP_PIXEL_FORMAT = 0x0C,
  READ_DISP_IMAGE_MODE = 0x0D,
  READ_DISP_SIGNAL_MODE = 0x0E,
  READ_DISP_SELF_DIAGNOSTIC = 0x0F,
  ENTER_SLEEP_MODE = 0x10,
  SLEEP_OUT = 0x11,
  PARTIAL_MODE_ON = 0x12,
  NORMAL_DISP_MODE_ON = 0x13,
  DISP_INVERSION_OFF = 0x20,
  DISP_INVERSION_ON = 0x21,
  PIXEL_OFF = 0x22,
  PIXEL_ON = 0x23,
  DISPLAY_OFF = 0x28,
  DISPLAY_ON = 0x29,
  COLUMN_ADDRESS_SET = 0x2A,
  PAGE_ADDRESS_SET = 0x2B,
  MEMORY_WRITE = 0x2C,
  MEMORY_READ = 0x2E,
  PARTIAL_AREA = 0x30,
  VERT_SCROLL_DEFINITION = 0x33,
  TEARING_EFFECT_LINE_OFF = 0x34,
  TEARING_EFFECT_LINE_ON = 0x35,
  MEMORY_ACCESS_CONTROL = 0x36,
  VERT_SCROLL_START_ADDRESS = 0x37,
  IDLE_MODE_OFF = 0x38,
  IDLE_MODE_ON = 0x39,
  INTERFACE_PIXEL_FORMAT = 0x3A,
  MEMORY_WRITE_CONTINUE = 0x3C,
  MEMORY_READ_CONTINUE = 0x3E,
  SET_TEAR_SCANLINE = 0x44,
  GET_SCANLINE = 0x45,
  WRITE_DISPLAY_BRIGHTNESS = 0x51,
  READ_DISPLAY_BRIGHTNESS = 0x52,
  WRITE_CTRL_DISPLAY = 0x53,
  READ_CTRL_DISPLAY = 0x54,
  WRITE_CONTENT_ADAPT_BRIGHTNESS = 0x55,
  READ_CONTENT_ADAPT_BRIGHTNESS = 0x56,
  WRITE_MIN_CAB_LEVEL = 0x5E,
  READ_MIN_CAB_LEVEL = 0x5F,
  READ_ABC_SELF_DIAG_RES = 0x68,
  READ_ID1 = 0xDA,
  READ_ID2 = 0xDB,
  READ_ID3 = 0xDC,
  INTERFACE_MODE_CONTROL = 0xB0,
  FRAME_RATE_CONTROL_NORMAL = 0xB1,
  FRAME_RATE_CONTROL_IDLE_8COLOR = 0xB2,
  FRAME_RATE_CONTROL_PARTIAL = 0xB3,
  DISPLAY_INVERSION_CONTROL = 0xB4,
  BLANKING_PORCH_CONTROL = 0xB5,
  DISPLAY_FUNCTION_CONTROL = 0xB6,
  ENTRY_MODE_SET = 0xB7,
  BACKLIGHT_CONTROL_1 = 0xB9,
  BACKLIGHT_CONTROL_2 = 0xBA,
  HS_LANES_CONTROL = 0xBE,
  POWER_CONTROL_1 = 0xC0,
  POWER_CONTROL_2 = 0xC1,
  POWER_CONTROL_NORMAL_3 = 0xC2,
  POWER_CONTROL_IDEL_4 = 0xC3,
  POWER_CONTROL_PARTIAL_5 = 0xC4,
  VCOM_CONTROL_1 = 0xC5,
  CABC_CONTROL_1 = 0xC6,
  CABC_CONTROL_2 = 0xC8,
  CABC_CONTROL_3 = 0xC9,
  CABC_CONTROL_4 = 0xCA,
  CABC_CONTROL_5 = 0xCB,
  CABC_CONTROL_6 = 0xCC,
  CABC_CONTROL_7 = 0xCD,
  CABC_CONTROL_8 = 0xCE,
  CABC_CONTROL_9 = 0xCF,
  NVMEM_WRITE = 0xD0,
  NVMEM_PROTECTION_KEY = 0xD1,
  NVMEM_STATUS_READ = 0xD2,
  READ_ID4 = 0xD3,
  ADJUST_CONTROL_1 = 0xD7,
  READ_ID_VERSION = 0xD8,
  POSITIVE_GAMMA_CORRECTION = 0xE0,
  NEGATIVE_GAMMA_CORRECTION = 0xE1,
  DIGITAL_GAMMA_CONTROL_1 = 0xE2,
  DIGITAL_GAMMA_CONTROL_2 = 0xE3,
  SET_IMAGE_FUNCTION = 0xE9,
  ADJUST_CONTROL_2 = 0xF2,
  ADJUST_CONTROL_3 = 0xF7,
  ADJUST_CONTROL_4 = 0xF8,
  ADJUST_CONTROL_5 = 0xF9,
  SPI_READ_COMMAND_SETTING = 0xFB,
  ADJUST_CONTROL_6 = 0xFC,
  ADJUST_CONTROL_7 = 0xFF,
};

static uint8_t frame_buffer[STRIDE * HEIGHT];

static void start_command(command_t cmd);
static void end_command();
static void send_data(const uint8_t *data, uint32_t size);
static void write_command(command_t cmd, const uint8_t *data, uint32_t size);
static void write_command(command_t cmd);
static void write_command_1(command_t cmd, uint8_t data0);
static void write_command_2(command_t cmd, uint8_t data0, uint8_t data1);
static void set_window(int x, int y, int w, int h);
static void clip_rect(int *x, int *y, int *w, int *h);

void init() {
  xiao_gpio_init(XIAO_DISPLAY_CS_PIN, true);
  xiao_gpio_init(XIAO_DISPLAY_DC_PIN, true);
  xiao_gpio_write(XIAO_DISPLAY_CS_PIN, 1);
  xiao_gpio_write(XIAO_DISPLAY_DC_PIN, 1);

  xiao_ioex_set_dir(XIAO_IOEX_LCD_RST_PIN, true);
  xiao_ioex_write(XIAO_IOEX_LCD_RST_PIN, 0);
  xiao_sleep_ms(100);
  xiao_ioex_write(XIAO_IOEX_LCD_RST_PIN, 1);
  xiao_sleep_ms(100);

  write_command(command_t::SOFTWARE_RESET);
  xiao_sleep_ms(200);

  write_command(command_t::SLEEP_OUT);
  xiao_sleep_ms(200);

  write_command_1(command_t::INTERFACE_PIXEL_FORMAT, 0x53);
  // write_command_1(Command::INTERFACE_PIXEL_FORMAT, 0x55); // RGB565
  // write_command_1(Command::INTERFACE_PIXEL_FORMAT, 0x56); // RGB666

  // #if 0
  //     writeCommand(Command::MEMORY_ACCESS_CONTROL, 0x48);
  // #else
  //     // writeCommand(Command::MEMORY_ACCESS_CONTROL, 0xE8);
  //     writeCommand(Command::MEMORY_ACCESS_CONTROL, 0x28);
  // #endif

  write_command(command_t::DISP_INVERSION_ON);

  fill_rect(0, 0, WIDTH, HEIGHT, 0x0000);
  refresh();

  write_command(command_t::DISPLAY_ON);
  xiao_sleep_ms(25);
}

uint8_t *get_pixel_pointer(int x, int y) {
  if (x < 0 || WIDTH <= x || y < 0 || HEIGHT <= y) return nullptr;
  int i = y * STRIDE + x / 2 * 3;
  if ((x & 1) == 0) {
    return &frame_buffer[i + 0];
  } else {
    return &frame_buffer[i + 1];
  }
}

void set_pixel(int x, int y, uint16_t color) {
  if (x < 0 || WIDTH <= x || y < 0 || HEIGHT <= y) return;
  int i = y * STRIDE + x / 2 * 3;
  if ((x & 1) == 0) {
    // even pixel
    frame_buffer[i + 0] = (color >> 4) & 0xFF;
    frame_buffer[i + 1] = (frame_buffer[i + 1] & 0x0F) | ((color & 0x0F) << 4);
  } else {
    // odd pixel
    frame_buffer[i + 1] = (frame_buffer[i + 1] & 0xF0) | ((color >> 8) & 0x0F);
    frame_buffer[i + 2] = color & 0xFF;
  }
}

void clear(uint16_t color) { fill_rect(0, 0, WIDTH, HEIGHT, color); }

void fill_rect(int x, int y, int w, int h, uint16_t color) {
  clip_rect(&x, &y, &w, &h);
  if (w <= 0 || h <= 0) return;
  for (int iy = 0; iy < h; iy++) {
    for (int ix = 0; ix < w; ix++) {
      set_pixel(x + ix, y + iy, color);
    }
  }
}

void draw_rect(int x, int y, int w, int h, uint16_t color) {
  fill_rect(x, y, w + 1, 1, color);
  fill_rect(x, y + h, w + 1, 1, color);
  fill_rect(x, y + 1, 1, h - 1, color);
  fill_rect(x + w, y + 1, 1, h - 1, color);
}

void refresh() {
  set_window(0, 0, WIDTH, HEIGHT);
  start_command(command_t::MEMORY_WRITE);
  xiao_spi_dma_write_start(XIAO_DISPLAY_CS_PIN, frame_buffer, STRIDE * HEIGHT);
}

static void start_command(command_t cmd) {
  xiao_spi_dma_complete();

  xiao_spi_set_baudrate(xiao_spi_display_frequency());

  xiao_gpio_write(XIAO_DISPLAY_CS_PIN, 0);
  xiao_gpio_write(XIAO_DISPLAY_DC_PIN, 0);
  xiao_spi_write_blocking((const uint8_t *)&cmd, 1);
  xiao_gpio_write(XIAO_DISPLAY_DC_PIN, 1);
}

static void end_command() {
  xiao_gpio_write(XIAO_DISPLAY_CS_PIN, 1);
  xiao_gpio_write(XIAO_DISPLAY_DC_PIN, 1);
}

static void send_data(const uint8_t *data, uint32_t size) {
  xiao_spi_write_blocking(data, size);
}

static void write_command(command_t cmd, const uint8_t *data, uint32_t size) {
  start_command(cmd);
  send_data(data, size);
  end_command();
}

static void write_command(command_t cmd) {
  start_command(cmd);
  end_command();
}

static void write_command_1(command_t cmd, uint8_t data0) {
  start_command(cmd);
  send_data(&data0, 1);
  end_command();
}

static void write_command_2(command_t cmd, uint8_t data0, uint8_t data1) {
  uint8_t data[2] = {data0, data1};
  start_command(cmd);
  send_data(data, 2);
  end_command();
}

static void set_window(int x, int y, int w, int h) {
  int x_end = x + w - 1;
  int y_end = y + h - 1;
  uint8_t tmp[4];
  tmp[0] = x >> 8;
  tmp[1] = x & 0xFF;
  tmp[2] = x_end >> 8;
  tmp[3] = x_end & 0xFF;
  write_command(command_t::COLUMN_ADDRESS_SET, tmp, 4);
  tmp[0] = y >> 8;
  tmp[1] = y & 0xFF;
  tmp[2] = y_end >> 8;
  tmp[3] = y_end & 0xFF;
  write_command(command_t::PAGE_ADDRESS_SET, tmp, 4);
}

static void clip_rect(int *x, int *y, int *w, int *h) {
  if (*x < 0) {
    *w += *x;
    *x = 0;
  }
  if (*x + *w > WIDTH) {
    *w = WIDTH - *x;
  }
  if (*y < 0) {
    *h += *y;
    *y = 0;
  }
  if (*y + *h > HEIGHT) {
    *h = HEIGHT - *y;
  }
}

}  // namespace shapones::xiao::display
