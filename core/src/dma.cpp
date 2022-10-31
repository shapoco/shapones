#include "shapones/shapones.hpp"

namespace nes::dma {

static bool running = false;
static int next_cycle = 0;
static addr_t next_src_addr = 0;

bool is_running() {
    return running;    
}

void start(int src_page) {
    running = true;
    next_cycle = 0;
    next_src_addr = (addr_t)src_page * 0x100;
}

int exec_next_cycle() {
    if ( ! running) return 0;

    ppu::oam_dma_write(next_cycle++, cpu::bus_read(next_src_addr++));
    
    if (next_cycle >= TRANSFER_SIZE) {
        running = false;
    }
    return 2;
}

}