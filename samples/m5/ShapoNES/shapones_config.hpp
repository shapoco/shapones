#ifndef SHAPONES_CONFIG_HPP
#define SHAPONES_CONFIG_HPP

#include <M5Unified.h>
#include "shapones_core.h"

#if defined(ARDUINO_M5STACK_ATOMS3)

#define SHAPONES_ENABLE_PDM_AUDIO (1)
#define SHAPONES_ENABLE_HALF_SCREEN (1)

static constexpr int TF_CS_PIN = -1;
static constexpr int TF_SCK_PIN = 7;
static constexpr int TF_MISO_PIN = 8;
static constexpr int TF_MOSI_PIN = 6;

static const uint8_t BUTTON_ADC_PINS[] = { 5, 2, 1 };
static constexpr int DISPLAY_BUTTON_PIN = 41;

static constexpr int AUDIO_DOUT_PIN = 38;

#elif defined(ARDUINO_M5STACK_STICKS3)

#define SHAPONES_ENABLE_I2S_AUDIO (1)
#define SHAPONES_ENABLE_HALF_SCREEN (1)

static constexpr int TF_CS_PIN = -1;
static constexpr int TF_SCK_PIN = 1;
static constexpr int TF_MISO_PIN = 2;
static constexpr int TF_MOSI_PIN = 3;

static const uint8_t BUTTON_ADC_PINS[] = { 4, 5, 6 };
static constexpr int DISPLAY_BUTTON_PIN = 11;

static constexpr int AUDIO_MCLK_PIN = 18;
static constexpr int AUDIO_BCLK_PIN = 17;
static constexpr int AUDIO_LRCK_PIN = 15;
static constexpr int AUDIO_DOUT_PIN = 14;
static constexpr int AUDIO_DIN_PIN = 16;

#else

static constexpr int TF_CS_PIN = 4;
static const uint8_t BUTTON_ADC_PINS[] = { 5, 2, 1 };
static constexpr int DISPLAY_BUTTON_PIN = 11;

#endif

#ifndef SHAPONES_ENABLE_PDM_AUDIO
#define SHAPONES_ENABLE_PDM_AUDIO (0)
#endif

#ifndef SHAPONES_ENABLE_I2S_AUDIO
#define SHAPONES_ENABLE_I2S_AUDIO (0)
#endif

#ifndef SHAPONES_ENABLE_HALF_SCREEN
#define SHAPONES_ENABLE_HALF_SCREEN (0)
#endif

#ifndef SHAPONES_MAX_INES_IN_SRAM
#define SHAPONES_MAX_INES_IN_SRAM ((64 + 1) * 1024)
#endif

#if SHAPONES_ENABLE_HALF_SCREEN
static constexpr int BUFF_W = shapones::SCREEN_WIDTH / 2;
static constexpr int BUFF_H = shapones::SCREEN_HEIGHT / 2;
constexpr int DMA_HEIGHT = 120;
#else
static constexpr int BUFF_W = shapones::SCREEN_WIDTH;
static constexpr int BUFF_H = shapones::SCREEN_HEIGHT;
constexpr int DMA_HEIGHT = 60;
#endif

#endif
