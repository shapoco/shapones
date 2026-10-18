// SD card SPI for the shared FatFs disk driver (xiao/common diskio.c):
// the xiao_spi_* calls on SPI0, which only the card uses on the PicoCalc,
// so no DMA or bus sharing is needed.
#include "shapones/xiao/spi.h"
#include "shapones/xiao/pins.h"

#include <hardware/spi.h>
#include <pico/stdlib.h>

static spi_inst_t *const sd_spi = spi0;
static uint32_t baudrate = 400'000;

extern "C" {

uint32_t xiao_spi_display_frequency() { return 0; }
uint32_t xiao_spi_tfcard_frequency() { return 400'000; }

void xiao_spi_init() {
  spi_init(sd_spi, baudrate);
  gpio_set_function(XIAO_SPI_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(XIAO_SPI_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(XIAO_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_pull_up(XIAO_SPI_MISO_PIN);
  gpio_init(XIAO_TFCARD_CS_PIN);
  gpio_put(XIAO_TFCARD_CS_PIN, 1);
  gpio_set_dir(XIAO_TFCARD_CS_PIN, GPIO_OUT);
}

void xiao_spi_deinit() { spi_deinit(sd_spi); }

void xiao_spi_set_baudrate(uint32_t baud) {
  if (baud == baudrate) return;
  baudrate = baud;
  spi_set_baudrate(sd_spi, baudrate);
}

void xiao_spi_write_blocking(const uint8_t *data, uint32_t size) {
  spi_write_blocking(sd_spi, data, size);
}

void xiao_spi_read_blocking(uint8_t repeated_byte, uint8_t *data,
                            uint32_t size) {
  spi_read_blocking(sd_spi, repeated_byte, data, size);
}

void xiao_spi_dma_write_start(int cs, const uint8_t *data, uint32_t size) {
  if (cs >= 0) gpio_put(cs, 0);
  spi_write_blocking(sd_spi, data, size);
  if (cs >= 0) gpio_put(cs, 1);
}

void xiao_spi_dma_complete() {}
bool xiao_spi_dma_is_busy() { return false; }
}
