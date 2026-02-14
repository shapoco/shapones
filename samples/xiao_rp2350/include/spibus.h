#ifndef SHAPONES_XIAO_RP2350_SPIBUS_HPP
#define SHAPONES_XIAO_RP2350_SPIBUS_HPP

#include <stdint.h>

#include <hardware/spi.h>
#include <pico/stdlib.h>

#if defined(__cplusplus)
extern "C" {
#endif

static const int SPIBUS_MOSI_PIN = 3;
static const int SPIBUS_MISO_PIN = 4;
static const int SPIBUS_SCK_PIN = 2;
static const int SPIBUS_DISPLAY_CS_PIN = 5;
static const int SPIBUS_DISPLAY_DC_PIN = 28;
static const int SPIBUS_TFCARD_CS_PIN = 0;

extern spi_inst_t *const spibus_inst;

void spibus_init();
void spibus_deinit();

void spibus_dma_write_start(const uint8_t *data, uint32_t size);
bool spibus_dma_is_busy();
void spibus_dma_complete();

#if defined(__cplusplus)
}
#endif

#endif
