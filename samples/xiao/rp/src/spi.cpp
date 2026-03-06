#include "shapones/xiao/spi.h"
#include "shapones/xiao/gpio.h"
#include "shapones/xiao/pins.h"

#include <hardware/dma.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>

static uint dma_tx;
static dma_channel_config dma_cfg;

static spi_inst_t *const spibus_inst = spi0;

static int cs_pin = -1;
static uint32_t baudrate = 10'000'000;

static bool is_spi_shift_busy();
static void abort_dma();

extern "C" {

uint32_t xiao_spi_display_frequency() { return 62'500'000; }
uint32_t xiao_spi_tfcard_frequency() { return 10'000'000; }

void xiao_spi_init() {
  // SPI init
  spi_init(spibus_inst, baudrate);
  gpio_set_function(XIAO_SPI_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(XIAO_SPI_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(XIAO_SPI_SCK_PIN, GPIO_FUNC_SPI);

  // LCD control pins
  gpio_init(XIAO_DISPLAY_CS_PIN);
  gpio_put(XIAO_DISPLAY_CS_PIN, 1);
  gpio_set_dir(XIAO_DISPLAY_CS_PIN, GPIO_OUT);

  gpio_init(XIAO_DISPLAY_DC_PIN);
  gpio_put(XIAO_DISPLAY_DC_PIN, 1);
  gpio_set_dir(XIAO_DISPLAY_DC_PIN, GPIO_OUT);

  // TF card CS pin
  gpio_init(XIAO_TFCARD_CS_PIN);
  gpio_put(XIAO_TFCARD_CS_PIN, 1);
  gpio_set_dir(XIAO_TFCARD_CS_PIN, GPIO_OUT);

  dma_tx = dma_claim_unused_channel(true);
  dma_cfg = dma_channel_get_default_config(dma_tx);
  channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&dma_cfg, true);
  channel_config_set_write_increment(&dma_cfg, false);
  channel_config_set_dreq(&dma_cfg, spi_get_dreq(spibus_inst, true));
}

void xiao_spi_deinit() {
  xiao_spi_dma_complete();
  dma_channel_unclaim(dma_tx);
  spi_deinit(spibus_inst);

  const int pins[] = {
      XIAO_DISPLAY_CS_PIN, XIAO_DISPLAY_DC_PIN, XIAO_TFCARD_CS_PIN,
      XIAO_SPI_MISO_PIN,   XIAO_SPI_MOSI_PIN,   XIAO_SPI_SCK_PIN,
  };
  const int num_pins = sizeof(pins) / sizeof(pins[0]);
  // drain charge from pins
  for (int i = 0; i < num_pins; i++) {
    gpio_init(pins[i]);
    gpio_set_dir(pins[i], GPIO_OUT);
    gpio_put(pins[i], 0);
  }
  // disable pins
  for (int i = 0; i < num_pins; i++) {
    gpio_deinit(pins[i]);
  }
}

void xiao_spi_set_baudrate(uint32_t baud) {
  if (baud == baudrate) return;
  baudrate = baud;
  spi_set_baudrate(spibus_inst, baudrate);
}

void xiao_spi_write_blocking(const uint8_t *data, uint32_t size) {
  xiao_spi_dma_complete();
  spi_write_blocking(spibus_inst, data, size);
}

void xiao_spi_read_blocking(uint8_t repeated_byte, uint8_t *data,
                            uint32_t size) {
  xiao_spi_dma_complete();
  spi_read_blocking(spibus_inst, repeated_byte, data, size);
}

void xiao_spi_dma_write_start(int cs, const uint8_t *data, uint32_t size) {
  xiao_spi_dma_complete();
  if (cs >= 0) {
    xiao_gpio_write(cs, 0);
  }
  cs_pin = cs;
  dma_channel_configure(dma_tx, &dma_cfg, &spi_get_hw(spibus_inst)->dr, data,
                        size, true);
}

void xiao_spi_dma_complete() {
  while (xiao_spi_dma_is_busy()) {
    tight_loop_contents();
  }
  if (cs_pin >= 0) {
    xiao_gpio_write(cs_pin, 1);
    cs_pin = -1;
  }
}

bool xiao_spi_dma_is_busy() {
  return dma_channel_is_busy(dma_tx) || is_spi_shift_busy();
}
}

static bool is_spi_shift_busy() {
  return (spi_get_hw(spibus_inst)->sr & SPI_SSPSR_BSY_BITS) != 0;
}

static void abort_dma() {
  dma_hw->ints0 = 1u << dma_tx;
  dma_hw->ints1 = 1u << dma_tx;
  if (cs_pin >= 0) {
    xiao_gpio_write(cs_pin, 1);
    cs_pin = -1;
  }
}
