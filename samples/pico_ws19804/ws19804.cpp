
#include "ws19804.hpp"
#include "ws19804_c.h"

#if SHAPONES_USE_PIO
#include "ws19804.pio.h"
#endif

namespace ws19804 {

static int sys_clock_hz;
static direction_t curr_dir = EMPTY;
static int curr_speed = 10 * MHZ;

#if SHAPONES_USE_PIO
static PIO spi_pio;
static uint spi_sm;
static uint spi_offset;
#else
static spi_inst_t *spi_inst = spi1;
#endif
static uint dma_tx;

#if SHAPONES_USE_PIO
// SPI push byte
static void pio_push(uint8_t data);

// SPI pop byte
static uint8_t pio_pop();

// wait for SPI to idle
static void pio_wait_idle();

// setup SPI direction and speed
static void setup_pio(direction_t new_dir, int new_speed);

// set SPI direction
static void set_spi_direction(direction_t new_dir);
#endif

void init(int sys_clk_hz) {
  sys_clock_hz = sys_clk_hz;

#if SHAPONES_USE_PIO
  spi_pio = pio0;
  spi_sm = 0;

  spi_offset = pio_add_program(spi_pio, &ws19804_program);
  pio_gpio_init(spi_pio, PIN_MISO);
  pio_gpio_init(spi_pio, PIN_MOSI);
  pio_gpio_init(spi_pio, PIN_SCK);
  pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, PIN_MISO, 1, false);
  pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, PIN_MOSI, 1, true);
  pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, PIN_SCK, 1, true);
#else
  spi_init(spi_inst, curr_speed);  // max speed
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
#endif

  dma_tx = dma_claim_unused_channel(true);

  gpio_init(PIN_RST);
  gpio_init(PIN_DC);
  gpio_init(PIN_BL);
  gpio_init(PIN_LCD_CS);
  gpio_init(PIN_TP_CS);
  gpio_set_dir(PIN_RST, GPIO_OUT);
  gpio_set_dir(PIN_DC, GPIO_OUT);
  gpio_set_dir(PIN_BL, GPIO_OUT);
  gpio_set_dir(PIN_LCD_CS, GPIO_OUT);
  gpio_set_dir(PIN_TP_CS, GPIO_OUT);
  gpio_put(PIN_RST, 1);
  gpio_put(PIN_LCD_CS, 1);
  gpio_put(PIN_TP_CS, 1);
  gpio_put(PIN_BL, 0);

  // hardware reset
  gpio_put(PIN_RST, 1);
  sleep_ms(1);
  gpio_put(PIN_RST, 0);
  sleep_ms(10);
  gpio_put(PIN_RST, 1);
  sleep_ms(10);

  write_command(0x11);
  sleep_ms(120);

  // MX, MY, RGB mode
  {
    uint8_t data[] = {0xa0};
    write_command(0x36, data, sizeof(data));
  }

  {
    // uint8_t data[] = { 0x05 }; // RGB565
    uint8_t data[] = {0x03};  // RGB444
    write_command(0x3a, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    write_command(0xb2, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x72};
    write_command(0xb7, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x3d};
    write_command(0xbb, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x01};
    write_command(0xc2, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x19};
    write_command(0xc3, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x20};
    write_command(0xc4, data, sizeof(data));
  }

  {
    uint8_t data[] = {0x0f};
    write_command(0xc6, data, sizeof(data));
  }

  {
    uint8_t data[] = {0xa4, 0xa1};
    write_command(0xd0, data, sizeof(data));
  }

  {
    uint8_t data[] = {
        0x70, 0x04, 0x08, 0x09, 0x09, 0x05, 0x2a,
        0x33, 0x41, 0x07, 0x13, 0x13, 0x29, 0x2f,
    };
    write_command(0xe0, data, sizeof(data));
  }
  {
    uint8_t data[] = {
        0x70, 0x03, 0x09, 0x0a, 0x09, 0x06, 0x2b,
        0x34, 0x41, 0x07, 0x12, 0x14, 0x28, 0x2e,
    };
    write_command(0xe1, data, sizeof(data));
  }

  write_command(0x21);

  clear(0);

  write_command(0x29);

  gpio_put(PIN_BL, 1);
}

#if SHAPONES_USE_PIO
void setup_pio(direction_t new_dir, int new_speed) {
  if (new_dir == curr_dir && new_speed == curr_speed) return;

  float div = sys_clock_hz / 2.f / new_speed;

  pio_sm_set_enabled(spi_pio, spi_sm, false);

  // load new PIO
  if (new_dir == TX) {
    pio_sm_config pio_cfg = ws19804_program_get_default_config(spi_offset);
    sm_config_set_out_pins(&pio_cfg, PIN_MOSI, 1);
    sm_config_set_sideset_pins(&pio_cfg, PIN_SCK);
    sm_config_set_fifo_join(&pio_cfg, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&pio_cfg, false, true, 8);
    sm_config_set_clkdiv(&pio_cfg, div);
    pio_sm_init(spi_pio, spi_sm, spi_offset, &pio_cfg);
  } else if (new_dir == RX) {
    pio_sm_config pio_cfg = ws19804_program_get_default_config(spi_offset);
    sm_config_set_out_pins(&pio_cfg, PIN_MOSI, 1);
    sm_config_set_in_pins(&pio_cfg, PIN_MISO);
    sm_config_set_sideset_pins(&pio_cfg, PIN_SCK);
    sm_config_set_out_shift(&pio_cfg, false, true, 8);
    sm_config_set_in_shift(&pio_cfg, false, true, 8);
    sm_config_set_clkdiv(&pio_cfg, div);
    pio_sm_init(spi_pio, spi_sm, spi_offset, &pio_cfg);
  }

  hw_set_bits(&spi_pio->input_sync_bypass, 1u << PIN_MISO);
  pio_sm_set_enabled(spi_pio, spi_sm, true);

  curr_dir = new_dir;
  curr_speed = new_speed;
}
#endif

#if SHAPONES_USE_PIO
void set_spi_direction(direction_t new_dir) { setup_pio(new_dir, curr_speed); }
#endif

void set_spi_speed(int new_speed) {
#if SHAPONES_USE_PIO
  setup_pio(curr_dir, new_speed);
#else
  spi_set_baudrate(spi_inst, new_speed);
  curr_speed = new_speed;
#endif
}

void clear(uint16_t color) {
  uint8_t data[WIDTH * 3 / 2];
  for (int x = 0; x < WIDTH * 3 / 2; x++) {
    data[x] = color;
  }
  for (int y = 0; y < HEIGHT; y++) {
    start_write_data(0, y, WIDTH, 1, data);
    finish_write_data();
  }
}

void start_write_data(int x0, int y0, int w, int h, uint8_t *data) {
  int x1 = x0 + w - 1;
  int y1 = y0 + h - 1;
  {
    uint8_t data[] = {(uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xff),
                      (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xff)};
    write_command(0x2a, data, sizeof(data));
  }
  {
    uint8_t data[] = {(uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xff),
                      (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xff)};
    write_command(0x2b, data, sizeof(data));
  }
  // write_command(0x2c, data, w * h * 3 / 2);
  {
    gpio_put(PIN_DC, 0);
    gpio_put(PIN_LCD_CS, 0);
    uint8_t cmd = 0x2c;
    write_blocking(&cmd, 1);
    gpio_put(PIN_DC, 1);

    dma_channel_config dma_cfg = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
#if SHAPONES_USE_PIO
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(spi_pio, spi_sm, true));
    dma_channel_configure(dma_tx, &dma_cfg,
                          &spi_pio->txf[spi_sm],  // write address
                          data,                   // read address
                          w * h * 3 / 2,  // element count (each element is
                                          // of size transfer_data_size)
                          true);          // don't start yet
#else
    channel_config_set_dreq(&dma_cfg, spi_get_dreq(spi_inst, true));
    dma_channel_configure(dma_tx, &dma_cfg,
                          &spi_get_hw(spi_inst)->dr,  // write address
                          data,                       // read address
                          w * h * 3 / 2,  // element count (each element is
                                          // of size transfer_data_size)
                          true);          // don't start yet
#endif

    // dma_start_channel_mask(1u << dma_tx);
  }
}

void finish_write_data() {
  dma_channel_wait_for_finish_blocking(dma_tx);
#if SHAPONES_USE_PIO
  pio_wait_idle();
#else
  while (spi_get_hw(spi_inst)->sr & SPI_SSPSR_BSY_BITS) {
  }
#endif
  gpio_put(PIN_LCD_CS, 1);
}

void write_command(uint8_t cmd, const uint8_t *data, int len) {
#if SHAPONES_USE_PIO
  set_spi_direction(TX);
#endif
  gpio_put(PIN_DC, 0);  // command mode
  gpio_put(PIN_LCD_CS, 0);
  write_blocking(&cmd, 1);
  if (data) {
    gpio_put(PIN_DC, 1);  // data mode
    write_blocking(data, len);
  }
  gpio_put(PIN_LCD_CS, 1);
}

void write_command(uint8_t cmd) {
#if SHAPONES_USE_PIO
  set_spi_direction(TX);
#endif
  write_command(cmd, nullptr, 0);
}

void write_blocking(const uint8_t *data, int len) {
#if SHAPONES_USE_PIO
  set_spi_direction(TX);
  for (int i = 0; i < len; i++) {
    pio_push(data[i]);
  }
  pio_wait_idle();
#else
  spi_write_blocking(spi_inst, data, len);
#endif
}

void read_blocking(uint8_t tx_repeat, uint8_t *buff, int len) {
#if SHAPONES_USE_PIO
  set_spi_direction(RX);
  int tx_remain = len, rx_remain = len;
  io_rw_8 *txfifo = (io_rw_8 *)&spi_pio->txf[spi_sm];
  io_rw_8 *rxfifo = (io_rw_8 *)&spi_pio->rxf[spi_sm];
  while (tx_remain || rx_remain) {
    if (tx_remain && !pio_sm_is_tx_fifo_full(spi_pio, spi_sm)) {
      *txfifo = tx_repeat;
      --tx_remain;
    }
    if (rx_remain && !pio_sm_is_rx_fifo_empty(spi_pio, spi_sm)) {
      *buff++ = *rxfifo;
      --rx_remain;
    }
  }
#else
  spi_read_blocking(spi_inst, tx_repeat, buff, len);
#endif
}

#if SHAPONES_USE_PIO
static void pio_push(uint8_t data) {
  while (pio_sm_is_tx_fifo_full(spi_pio, spi_sm)) {
  }
  *(volatile uint8_t *)&spi_pio->txf[spi_sm] = data;
}

static uint8_t pio_pop() {
  while (pio_sm_is_rx_fifo_empty(spi_pio, spi_sm)) {
  }
  return *(volatile uint8_t *)&spi_pio->rxf[spi_sm];
}

static void pio_wait_idle() {
  uint32_t stall_mask = 1u << (spi_sm + PIO_FDEBUG_TXSTALL_LSB);
  spi_pio->fdebug = stall_mask;
  while (!(spi_pio->fdebug & stall_mask)) {
  }
}
#endif

extern "C" {

void ws19804_write_blocking(const uint8_t *buff, int len) {
  ws19804::write_blocking(buff, len);
}
void ws19804_read_blocking(uint8_t tx_repeat, uint8_t *buff, int len) {
  ws19804::read_blocking(tx_repeat, buff, len);
}
}

}  // namespace ws19804
