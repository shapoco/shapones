#ifndef SHAPONES_XIAO_SPI_HPP
#define SHAPONES_XIAO_SPI_HPP

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

uint32_t xiao_spi_display_frequency();
uint32_t xiao_spi_tfcard_frequency();
void xiao_spi_init();
void xiao_spi_deinit();
void xiao_spi_set_baudrate(uint32_t baudrate);
void xiao_spi_write_blocking(const uint8_t *data, uint32_t size);
void xiao_spi_read_blocking(uint8_t repeated_byte, uint8_t *data,
                            uint32_t size);
void xiao_spi_dma_write_start(int cs_pin, const uint8_t *data, uint32_t size);
void xiao_spi_dma_complete();
bool xiao_spi_dma_is_busy();

#if defined(__cplusplus)
}
#endif

#endif
