#ifndef SHAPONES_COMMON_HPP
#define SHAPONES_COMMON_HPP

#if !(SHAPONES_NO_STDLIB)
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#endif

#ifndef SHAPONES_DEFINE_FAST_INT
#define SHAPONES_DEFINE_FAST_INT (0)
#endif

#if SHAPONES_DEFINE_FAST_INT
using uint_fast8_t = uint8_t;
using uint_fast16_t = uint16_t;
using uint_fast32_t = uint32_t;
using int_fast8_t = int8_t;
using int_fast16_t = int16_t;
using int_fast32_t = int32_t;
#endif

#if SHAPONES_ENABLE_LOG

#if !(SHAPONES_NO_STDLIB)
#include <stdlib.h>
#endif

#define SHAPONES_PRINTF(fmt, ...) \
    do { \
        printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
        printf((fmt), ##__VA_ARGS__); \
        fflush(stdout); \
    } while(0)

#define SHAPONES_ERRORF(fmt, ...) \
    do { \
        printf("[%s:%d] ", __FILE_NAME__, __LINE__); \
        printf("*ERROR: "); \
        printf(fmt, ##__VA_ARGS__); \
        fflush(stdout); \
        nes::cpu::stop(); \
    } while(0)

#else 

#define SHAPONES_PRINTF(fmt, ...) \
    do { } while(0)

#define SHAPONES_ERRORF(fmt, ...) \
    do { \
        nes::cpu::stop(); \
    } while(0)

#endif

#define SHAPONES_INLINE inline __attribute__((always_inline))
#define SHAPONES_NOINLINE __attribute__((noinline))

#ifndef SHAPONES_ENABLE_CHROM_CACHE
#define SHAPONES_ENABLE_CHROM_CACHE (0)
#endif

namespace nes {

using addr_t = uint_fast16_t;
using cycle_t = uint32_t;

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

enum class NametableArrangement : uint8_t {
    HORIZONTAL = 0,
    VERTICAL = 1,
    SINGLE_LOWER = 2,
    SINGLE_UPPER = 3,
    FOUR_SCREEN = 4,
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
