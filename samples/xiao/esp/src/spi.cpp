#include "shapones/xiao/spi.h"
#include "shapones/xiao/gpio.h"
#include "shapones/xiao/pins.h"

#include <Arduino.h>

#include <ESP32DMASPIMaster.h>
ESP32DMASPI::Master spi_dma;

extern "C" {

static constexpr size_t TRANSFER_SIZE = 4096;

static constexpr size_t QUEUE_SIZE = (2 * 256 * 256) / TRANSFER_SIZE;
int cs_pin = -1;
uint32_t baudrate = 10000000;
bool dma_inited = false;

uint8_t *tx_buff = nullptr;
uint8_t *rx_buff = nullptr;

static void init_dma();
static void deinit_dma();

uint32_t xiao_spi_display_frequency() { return 60000000; }
uint32_t xiao_spi_tfcard_frequency() { return 10000000; }

void xiao_spi_init() {
  cs_pin = -1;
  tx_buff = spi_dma.allocDMABuffer(TRANSFER_SIZE);
  rx_buff = spi_dma.allocDMABuffer(TRANSFER_SIZE);
  init_dma();
}

void xiao_spi_deinit() {
  deinit_dma();
  if (tx_buff) free(tx_buff);
  if (rx_buff) free(rx_buff);
}

void xiao_spi_set_baudrate(uint32_t baud) {
  if (baud == baudrate) return;
  baudrate = baud;
  deinit_dma();
  init_dma();
}

void xiao_spi_write_blocking(const uint8_t *data, uint32_t size) {
  xiao_spi_dma_write_start(-1, data, size);
  xiao_spi_dma_complete();
}

void xiao_spi_read_blocking(uint8_t repeated_byte, uint8_t *data,
                            uint32_t size) {
  xiao_spi_dma_complete();
  init_dma();
  while (size > 0) {
    size_t chunk_size = size < TRANSFER_SIZE ? size : TRANSFER_SIZE;
    memset(tx_buff, repeated_byte, chunk_size);
    spi_dma.queue(tx_buff, rx_buff, chunk_size);
    spi_dma.wait();
    memcpy(data, rx_buff, chunk_size);
    data += chunk_size;
    size -= chunk_size;
  }
}

void xiao_spi_dma_write_start(int cs, const uint8_t *data, uint32_t size) {
  xiao_spi_dma_complete();
  init_dma();
  if (cs >= 0) {
    xiao_gpio_put(cs, 0);
  }
  cs_pin = cs;
  while (size > 0) {
    size_t chunk_size = size < TRANSFER_SIZE ? size : TRANSFER_SIZE;
    spi_dma.queue(data, nullptr, chunk_size);
    data += chunk_size;
    size -= chunk_size;
  }
  spi_dma.trigger();
}

void xiao_spi_dma_complete() {
  if (!dma_inited) return;
  while (spi_dma.numTransactionsInFlight() > 0) {
  }
  if (cs_pin >= 0) {
    xiao_gpio_put(cs_pin, 1);
    cs_pin = -1;
  }
}

bool xiao_spi_dma_is_busy() {
  return dma_inited && spi_dma.numTransactionsInFlight() > 0;
}

static void init_dma() {
  if (dma_inited) return;
  spi_dma.setDataMode(SPI_MODE0);
  spi_dma.setFrequency(baudrate);
  spi_dma.setMaxTransferSize(TRANSFER_SIZE);
  spi_dma.setQueueSize(QUEUE_SIZE);
  spi_dma.begin(FSPI, XIAO_SPI_SCK_PIN, XIAO_SPI_MISO_PIN, XIAO_SPI_MOSI_PIN,
                -1);
  dma_inited = true;
}

static void deinit_dma() {
  if (!dma_inited) return;
  xiao_spi_dma_complete();
  spi_dma.end();
  dma_inited = false;
}
}
