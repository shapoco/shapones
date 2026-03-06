#ifndef SHAPONES_XIAO_RAM_H
#define SHAPONES_XIAO_RAM_H

#include <stdbool.h>
#include <stdlib.h>

#if defined(__cplusplus)
extern "C" {
#endif

void *xiao_malloc(size_t size, bool for_dma);
void xiao_free(void *ptr);

#if defined(__cplusplus)
}
#endif

#endif
