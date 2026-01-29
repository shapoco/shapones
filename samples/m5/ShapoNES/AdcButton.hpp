#ifndef ADC_BUTTON_HPP
#define ADC_BUTTON_HPP

#include <stdint.h>

namespace adc_button {

static constexpr int NUM_BUTTONS = 3;
static constexpr int NUM_LEVELS = 1 << NUM_BUTTONS;

//static constexpr uint32_t ADC_MAX = 4096 * 0.93f / 1.1f;  // 3.3V * -11dB = 0.93V
static constexpr uint32_t VCC_MV = 3300;

static constexpr float R1 = 4700;
static constexpr float R2_A = 22000;
static constexpr float R2_B = 10000;
static constexpr float R2_C = 4700;

static constexpr float R2_AB = 1.0f / (1.0f / R2_A + 1.0f / R2_B);
static constexpr float R2_AC = 1.0f / (1.0f / R2_A + 1.0f / R2_C);
static constexpr float R2_BC = 1.0f / (1.0f / R2_B + 1.0f / R2_C);
static constexpr float R2_ABC = 1.0f / (1.0f / R2_A + 1.0f / R2_B + 1.0f / R2_C);

static constexpr float ADC_C = VCC_MV * R2_C / (R1 + R2_C);
static constexpr float ADC_B = VCC_MV * R2_B / (R1 + R2_B);
static constexpr float ADC_BC = VCC_MV * R2_BC / (R1 + R2_BC);
static constexpr float ADC_A = VCC_MV * R2_A / (R1 + R2_A);
static constexpr float ADC_AC = VCC_MV * R2_AC / (R1 + R2_AC);
static constexpr float ADC_AB = VCC_MV * R2_AB / (R1 + R2_AB);
static constexpr float ADC_ABC = VCC_MV * R2_ABC / (R1 + R2_ABC);

const uint16_t THRESHOLDS[] = {
  0x0000u,
  (uint16_t)((ADC_BC + ADC_ABC) / 2),
  (uint16_t)((ADC_AC + ADC_BC) / 2),
  (uint16_t)((ADC_C + ADC_AC) / 2),
  (uint16_t)((ADC_AB + ADC_C) / 2),
  (uint16_t)((ADC_B + ADC_AB) / 2),
  (uint16_t)((ADC_A + ADC_B) / 2),
  (uint16_t)(ADC_A + (ADC_A - ADC_B) / 2),
  0xFFFFu,
};

const uint32_t CODES[] = {
  0b111,  // ABC
  0b110,  // BC
  0b101,  // AC
  0b100,  // C
  0b011,  // AB
  0b010,  // B
  0b001,  // A
  0b000,  // none
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
      if (value >= THRESHOLDS[imid]) {
        i0 = imid;
      } else {
        i1 = imid;
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

  uint32_t read_current() {
    return current_code;
  }
};

};  // namespace adc_button

#endif
