#include "shapones/xiao/ram.h"

#include <Arduino.h>
#include <esp_heap_caps.h>

extern "C" {

void *xiao_malloc(size_t size, bool for_dma) {
  if (for_dma) {
    return heap_caps_malloc(size, MALLOC_CAP_DMA);
  } else {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  }
}

void xiao_free(void *ptr) { heap_caps_free(ptr); }
}