#ifndef ADC_BUTTON_HPP
#define ADC_BUTTON_HPP

#include <stdint.h>

namespace adc_button {

static constexpr int NUM_BUTTONS = 3;
static constexpr int NUM_LEVELS = 1 << NUM_BUTTONS;

static constexpr float R1 = 10000;
static constexpr float R2_A = 4700;
static constexpr float R2_B = 10000;
static constexpr float R2_C = 22000;

static constexpr int ADC_MAX = 4096;
static constexpr float R2_ABC = 1.0f / (1.0f / R2_A + 1.0f / R2_B + 1.0f / R2_C);
static constexpr float R2_AB = 1.0f / (1.0f / R2_A + 1.0f / R2_B);
static constexpr float R2_AC = 1.0f / (1.0f / R2_A + 1.0f / R2_C);
static constexpr float R2_BC = 1.0f / (1.0f / R2_B + 1.0f / R2_C);

static constexpr float ADC_ABC = ADC_MAX * R2_ABC / (R1 + R2_ABC);
static constexpr float ADC_AB = ADC_MAX * R2_AB / (R1 + R2_AB);
static constexpr float ADC_AC = ADC_MAX * R2_AC / (R1 + R2_AC);
static constexpr float ADC_A = ADC_MAX * R2_A / (R1 + R2_A);
static constexpr float ADC_BC = ADC_MAX * R2_BC / (R1 + R2_BC);
static constexpr float ADC_B = ADC_MAX * R2_B / (R1 + R2_B);
static constexpr float ADC_C = ADC_MAX * R2_C / (R1 + R2_C);

const uint16_t THRESHOLDS[] = {
  0,
  (uint16_t)((ADC_ABC + ADC_AB) / 2),
  (uint16_t)((ADC_AB + ADC_AC) / 2),
  (uint16_t)((ADC_AC + ADC_A) / 2),
  (uint16_t)((ADC_A + ADC_BC) / 2),
  (uint16_t)((ADC_BC + ADC_B) / 2),
  (uint16_t)((ADC_B + ADC_C) / 2),
  (uint16_t)((ADC_C + ADC_MAX) / 2),
  ADC_MAX,
};

const uint32_t CODES[] ={
  0b111, // ABC
  0b011, // AB
  0b101, // AC
  0b001, // A
  0b110, // BC
  0b010, // B
  0b100, // C
  0b000, // none
};

class Pin {
 private:
  uint32_t raw_level = 0;
  uint32_t stable_count = 0;
  uint32_t current_code = 0;
  
 public:
  Pin() {}

  void update_value(uint16_t value) {
    int last_raw_level = raw_level;
    int i0 = 0, i1 = NUM_LEVELS;
    while (i0 + 1 < i1) {
      int imid = (i0 + i1) / 2;
      if (value < THRESHOLDS[imid]) {
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
    
    if (stable_count >= 5) {
      current_code = CODES[raw_level];
    }
  }

  uint32_t read_current() {
    return current_code;
  }
};

};

#endif
