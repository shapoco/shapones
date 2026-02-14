#include <hardware/dma.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>

#include "spibus.h"

uint dma_tx;
dma_channel_config dma_cfg;

spi_inst_t *const spibus_inst = spi0;

void spibus_init() {
  // SPI init
  spi_init(spibus_inst, 10 * 1000 * 1000);
  gpio_set_function(SPIBUS_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPIBUS_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPIBUS_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_drive_strength(SPIBUS_MOSI_PIN, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(SPIBUS_SCK_PIN, GPIO_DRIVE_STRENGTH_12MA);

  // LCD control pins
  gpio_init(SPIBUS_DISPLAY_CS_PIN);
  gpio_put(SPIBUS_DISPLAY_CS_PIN, 1);
  gpio_set_dir(SPIBUS_DISPLAY_CS_PIN, GPIO_OUT);

  gpio_init(SPIBUS_DISPLAY_DC_PIN);
  gpio_put(SPIBUS_DISPLAY_DC_PIN, 1);
  gpio_set_dir(SPIBUS_DISPLAY_DC_PIN, GPIO_OUT);

  // TF card CS pin
  gpio_init(SPIBUS_TFCARD_CS_PIN);
  gpio_put(SPIBUS_TFCARD_CS_PIN, 1);
  gpio_set_dir(SPIBUS_TFCARD_CS_PIN, GPIO_OUT);

  dma_tx = dma_claim_unused_channel(true);
  dma_cfg = dma_channel_get_default_config(dma_tx);
  channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&dma_cfg, spi_get_dreq(spibus_inst, true));
}

void spibus_deinit() {
  spibus_dma_complete();
  dma_channel_unclaim(dma_tx);
  spi_deinit(spibus_inst);

  const int pins[] = {
      SPIBUS_DISPLAY_CS_PIN, SPIBUS_DISPLAY_DC_PIN, SPIBUS_TFCARD_CS_PIN,
      SPIBUS_MISO_PIN,       SPIBUS_MOSI_PIN,       SPIBUS_SCK_PIN,
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

void spibus_dma_write_start(const uint8_t *data, uint32_t size) {
  dma_channel_configure(dma_tx, &dma_cfg, &spi_get_hw(spibus_inst)->dr, data,
                        size, true);
}

bool spibus_dma_is_busy() {
  return dma_channel_is_busy(dma_tx) ||
         (spi_get_hw(spibus_inst)->sr & SPI_SSPSR_BSY_BITS);
}

void spibus_dma_complete() {
  while (spibus_dma_is_busy()) {
    tight_loop_contents();
  }
  gpio_put(SPIBUS_DISPLAY_CS_PIN, 1);
}
