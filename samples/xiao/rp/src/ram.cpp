#include "shapones/xiao/ram.h"

#include <pico/stdlib.h>

extern "C" {

void *xiao_malloc(size_t size, bool for_dma) { return malloc(size); }

void xiao_free(void *ptr) { free(ptr); }
}