#include "shapones/xiao/power.hpp"
#include "shapones/xiao/i2c.h"
#include "shapones/xiao/ioex.hpp"
#include "shapones/xiao/pins.h"

#include <Arduino.h>
#include <driver/rtc_io.h>

namespace shapones::xiao::power {

void init() {}

void service() {}

void deep_sleep() {
  Serial.end();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)XIAO_POWER_BUTTON_PIN, 1);
  esp_deep_sleep_start();

  esp_restart();
}

}  // namespace shapones::xiao::power
