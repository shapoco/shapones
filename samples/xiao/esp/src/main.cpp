
#include "shapones/xiao/app.hpp"
#include "shapones/xiao/display.hpp"
#include "shapones/xiao/spi.h"
#include "shapones/xiao/timer.h"

#include <Arduino.h>

namespace shapones::xiao::esp {

static void ppu_loop(void *arg);

void setup() {
  app_init();
  xTaskCreatePinnedToCore(ppu_loop, "PPULoop", 8192, NULL, 10, NULL,
                          PRO_CPU_NUM);
}

void loop() { cpu_service(); }

static void ppu_loop(void *arg) {
  uint64_t next_wdt_reset_ms = 0;
  while (true) {
    ppu_service();

    uint64_t now_ms = millis();
    if (now_ms > next_wdt_reset_ms) {
      next_wdt_reset_ms = now_ms + 1000;
      vTaskDelay(1);
    }
  }
}

}  // namespace shapones::xiao::esp

void setup() { shapones::xiao::esp::setup(); }

void loop() { shapones::xiao::esp::loop(); }
