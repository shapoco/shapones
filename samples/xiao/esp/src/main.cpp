
#include "shapones/xiao/app.hpp"
#include "shapones/xiao/display.hpp"
#include "shapones/xiao/spi.h"
#include "shapones/xiao/timer.h"

#include <Arduino.h>
#include <soc/timer_group_reg.h>
#include <soc/timer_group_struct.h>
#include <soc/wdt_periph.h>

namespace shapones::xiao::esp {

static void ppu_loop(void *arg);
static void feed_watchdog();

void setup() {
  app_init();
  xTaskCreatePinnedToCore(ppu_loop, "PPULoop", 8192, NULL, 10, NULL,
                          PRO_CPU_NUM);
}

void loop() {
  cpu_service();
  feed_watchdog();
}

static void ppu_loop(void *arg) {
  uint64_t next_wdt_reset_ms = 0;
  while (true) {
    ppu_service();

    uint64_t now_ms = millis();
    if (now_ms > next_wdt_reset_ms) {
      next_wdt_reset_ms = now_ms + 1000;
      feed_watchdog();
    }
  }
}

static void feed_watchdog() {
  TIMERG0.wdtwprotect.val = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdtfeed.val = 1;
  TIMERG0.wdtwprotect.val = 0;
  TIMERG1.wdtwprotect.val = TIMG_WDT_WKEY_VALUE;
  TIMERG1.wdtfeed.val = 1;
  TIMERG1.wdtwprotect.val = 0;
}

}  // namespace shapones::xiao::esp

void setup() { shapones::xiao::esp::setup(); }

void loop() { shapones::xiao::esp::loop(); }
