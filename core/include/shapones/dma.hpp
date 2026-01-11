#ifndef SHAPONES_DMA_HPP
#define SHAPONES_DMA_HPP

#include "shapones/common.hpp"

namespace nes::dma {

static constexpr int TRANSFER_SIZE = 256;

bool is_running();
void start(int src_page);
int exec_next_cycle();

}

#endif
