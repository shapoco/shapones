#ifndef SHAPONES_HPP
#define SHAPONES_HPP

#include "stdio.h"
#include "stdlib.h"

//#include "pico/stdlib.h"
//#include "hardware/gpio.h"

#define NES_ALWAYS_INLINE __attribute__((always_inline)) static inline

#if 1

#define NES_PRINTF(fmt, ...) \
    do { \
        printf((fmt), ##__VA_ARGS__); \
        fflush(stdout); \
    } while(0)

#define NES_ERRORF(fmt, ...) \
    do { \
        printf("*ERROR: "); \
        NES_PRINTF(fmt, ##__VA_ARGS__); \
        nes::cpu::stop(); \
    } while(0)

#else 

#define NES_PRINTF(fmt, ...) \
    do { } while(0)

#define NES_ERRORF(fmt, ...) \
    do { \
        nes::cpu::stop(); \
    } while(0)

#endif

#include "shapones/common.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/memory.hpp"
#include "shapones/cpu.hpp"
#include "shapones/ppu.hpp"
#include "shapones/dma.hpp"
#include "shapones/apu.hpp"

namespace nes {

void reset();
void render_next_line(uint8_t *line_buff);
void vsync(uint8_t *line_buff);

}

#endif
