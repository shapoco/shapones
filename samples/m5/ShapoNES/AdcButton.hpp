#ifndef ADC_BUTTON_HPP
#define ADC_BUTTON_HPP

#include <stdint.h>

namespace adc_button {

static constexpr int NUM_BUTTONS = 3;
static constexpr int NUM_LEVELS = 1 << NUM_BUTTONS;

static constexpr float R1 = 10000;
static constexpr float R2_C = 5000;
static constexpr float R2_B = 10000;
static constexpr float R2_A = 20000;

static constexpr uint32_t ADC_MAX = 4095;
static constexpr float R2_ABC =
    1.0f / (1.0f / R2_A + 1.0f / R2_B + 1.0f / R2_C);
static constexpr float R2_AB = 1.0f / (1.0f / R2_A + 1.0f / R2_B);
static constexpr float R2_AC = 1.0f / (1.0f / R2_A + 1.0f / R2_C);
static constexpr float R2_BC = 1.0f / (1.0f / R2_B + 1.0f / R2_C);

static constexpr float ADC_C = 256 * (R1 + R2_C) / R2_C;
static constexpr float ADC_B = 256 * (R1 + R2_B) / R2_B;
static constexpr float ADC_BC = 256 * (R1 + R2_BC) / R2_BC;
static constexpr float ADC_A = 256 * (R1 + R2_A) / R2_A;
static constexpr float ADC_AC = 256 * (R1 + R2_AC) / R2_AC;
static constexpr float ADC_AB = 256 * (R1 + R2_AB) / R2_AB;
static constexpr float ADC_ABC = 256 * (R1 + R2_ABC) / R2_ABC;

const uint16_t THRESHOLDS[] = {
    0x0000u,
    (uint16_t)(ADC_A - (ADC_B - ADC_A) / 2),
    (uint16_t)((ADC_A + ADC_B) / 2),
    (uint16_t)((ADC_B + ADC_AB) / 2),
    (uint16_t)((ADC_AB + ADC_C) / 2),
    (uint16_t)((ADC_C + ADC_AC) / 2),
    (uint16_t)((ADC_AC + ADC_BC) / 2),
    (uint16_t)((ADC_BC + ADC_ABC) / 2),
    0xFFFFu,
};

const uint32_t CODES[] = {
    0b000,  // none
    0b001,  // A
    0b010,  // B
    0b011,  // AB
    0b100,  // C
    0b101,  // AC
    0b110,  // BC
    0b111,  // ABC
};

class Pin {
 private:
  uint32_t raw_level = 0;
  uint32_t stable_count = 0;
  uint32_t current_code = 0;

 public:
  Pin() {}

  void update_value(uint16_t value) {
    if (value == 0) value = 1;
    uint32_t inv = 256 * ADC_MAX / value;
    int last_raw_level = raw_level;
    int i0 = 0, i1 = NUM_LEVELS;
    while (i0 + 1 < i1) {
      int imid = (i0 + i1) / 2;
      if (inv < THRESHOLDS[imid]) {
        i1 = imid;
      } else {
        i0 = imid;
      }
    }
    raw_level = i0;

    if (raw_level == last_raw_level) {
      stable_count++;
    } else {
      stable_count = 0;
    }

    if (stable_count >= 3) {
      current_code = CODES[raw_level];
    }
  }

  uint32_t read_current() { return current_code; }
};

};  // namespace adc_button

#endif
