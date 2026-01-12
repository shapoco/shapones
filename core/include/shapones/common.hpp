#ifndef SHAPONES_BASIC_HPP
#define SHAPONES_BASIC_HPP

#if !(SHAPONES_NO_STDLIB)
#include <stdint.h>
#include <stdio.h>
#endif

#if SHAPONES_ENABLE_LOG

#if !(SHAPONES_NO_STDLIB)
#include <stdlib.h>
#endif

#define NES_PRINTF(fmt, ...) \
    do { \
        printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
        printf((fmt), ##__VA_ARGS__); \
        fflush(stdout); \
    } while(0)

#define NES_ERRORF(fmt, ...) \
    do { \
        printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
        printf("*ERROR: "); \
        printf(fmt, ##__VA_ARGS__); \
        fflush(stdout); \
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

#define NES_ALWAYS_INLINE __attribute__((always_inline))  inline

namespace nes {

using addr_t = uint16_t;
using cycle_t = unsigned int;

static constexpr int SCREEN_WIDTH = 256;
static constexpr int SCREEN_HEIGHT = 240;

static constexpr addr_t WRAM_SIZE = 2 * 1024;
static constexpr addr_t VRAM_SIZE = 4 * 1024;
static constexpr addr_t PRGROM_RANGE = 32 * 1024;
static constexpr addr_t PRGRAM_RANGE = 8 * 1024;
static constexpr addr_t CHRROM_RANGE = 8 * 1024;

static constexpr int NUM_LOCKS = 3;
static constexpr int LOCK_INTERRUPTS = 0;
static constexpr int LOCK_PPU = 1;
static constexpr int LOCK_APU = 2;

struct Config {
    uint32_t apu_sampling_rate;
};

void lock_init(int id);
void lock_deinit(int id);
void lock_get(int id);
void lock_release(int id);

class Exclusive {
public:
    const int id;
    Exclusive(int id) : id(id) {
        lock_get(id);
    }
    ~Exclusive() {
        lock_release(id);
    }
};

}  // namespace nes

#endif
