#ifndef WS19804_C_H
#define WS19804_C_H

#include "stdint.h"

// SPI write function for FatFS
void ws19804_write_blocking(const uint8_t *data, int len);

// SPI read function for FatFS
void ws19804_read_blocking(uint8_t tx_repeat, uint8_t *buff, int len);

#endif
