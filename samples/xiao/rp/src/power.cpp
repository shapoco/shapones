#include "shapones/xiao/power.hpp"
#include "shapones/xiao/i2c.h"
#include "shapones/xiao/ioex.hpp"
#include "shapones/xiao/pins.h"

#include <hardware/clocks.h>
#include <hardware/powman.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/sleep.h>
#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

namespace shapones::xiao::power {

static constexpr int XIAO_BAT_ADC_EN_PIN = 19;
static constexpr int XIAO_NEO_PWR_PIN = 23;
static constexpr int XIAO_LED_Y_PIN = 25;

void init() {}

void service() {}

void deep_sleep() {
  multicore_reset_core1();

  stdio_flush();
  stdio_usb_deinit();

  gpio_set_pulls(PICO_DEFAULT_UART_TX_PIN, false, false);
  gpio_set_pulls(PICO_DEFAULT_UART_RX_PIN, false, false);
  gpio_deinit(PICO_DEFAULT_UART_TX_PIN);
  gpio_deinit(PICO_DEFAULT_UART_RX_PIN);

  gpio_deinit(XIAO_LED_Y_PIN);
  gpio_pull_up(XIAO_LED_Y_PIN);

  gpio_deinit(XIAO_BAT_ADC_EN_PIN);
  gpio_pull_down(XIAO_BAT_ADC_EN_PIN);

  gpio_deinit(XIAO_NEO_PWR_PIN);
  gpio_pull_down(XIAO_NEO_PWR_PIN);

#if 1
  gpio_init(XIAO_POWER_BUTTON_PIN);
  gpio_set_dir(XIAO_POWER_BUTTON_PIN, false);
  powman_enable_gpio_wakeup(0, XIAO_POWER_BUTTON_PIN, false, true);

  powman_set_debug_power_request_ignored(true);

  powman_power_state P1_7 = POWMAN_POWER_STATE_NONE;
  powman_power_state P0_3 = POWMAN_POWER_STATE_NONE;
  P0_3 = powman_power_state_with_domain_on(P0_3,
                                           POWMAN_POWER_DOMAIN_SWITCHED_CORE);
  P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_XIP_CACHE);
  powman_power_state off_state = P1_7;
  powman_power_state on_state = P0_3;

  powman_configure_wakeup_state(off_state, on_state);

  // reboot to main
  powman_hw->boot[0] = 0;
  powman_hw->boot[1] = 0;
  powman_hw->boot[2] = 0;
  powman_hw->boot[3] = 0;

  // Switch to required power state
  powman_set_power_state(off_state);

  // Power down
  while (true) __wfi();
#else
  sleep_run_from_rosc();

  gpio_deinit(PICO_DEFAULT_UART_TX_PIN);

  sleep_goto_dormant_until_pin(POWER_SW_PIN, true, true);
#endif

  watchdog_reboot(0, 0, 0);
  while (true) {
    tight_loop_contents();
  }
}

}  // namespace shapones::xiao::power
