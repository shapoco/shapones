#include <hardware/clocks.h>
#include <hardware/powman.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/sleep.h>
#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "power.hpp"
#include "pwm_audio.hpp"
#include "spibus.h"

namespace shapones::xiao::rp::power {

static constexpr int POWER_SW_PIN = 27;
static constexpr int I2C_SDA_PIN = 6;
static constexpr int I2C_SCL_PIN = 7;
static constexpr int XIAO_BAT_ADC_EN_PIN = 19;
static constexpr int XIAO_NEO_PWR_PIN = 23;
static constexpr int XIAO_LED_Y_PIN = 25;

static constexpr int IOEX_PERI_EN_PIN = 7;

mcp23017::Driver *ioex;

static uint64_t next_sample_time_us = 0;
static bool power_sw_pressed = false;

void init(mcp23017::Driver *ie) {
  ioex = ie;
  gpio_init(POWER_SW_PIN);
  gpio_set_dir(POWER_SW_PIN, GPIO_IN);

  i2c_init(i2c1, 400'000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);
  ioex->set_dir(0, 0xFF, 0x7F);
  ioex->set_dir(1, 0x03, 0x03);
  ioex->set_pullup(0, 0x7F, 0x7F);
  ioex->set_pullup(1, 0x03, 0x03);

  // power-on peripherals
  ioex->write_port(0, 1 << IOEX_PERI_EN_PIN, 0);
}

void service() {
  uint64_t now_us = time_us_64();
  if (now_us >= next_sample_time_us) {
    next_sample_time_us = now_us + 100'000;  // sample every 100ms
    bool pressed = gpio_get(POWER_SW_PIN);
    if (!pressed && power_sw_pressed) {
      shutdown();
    }
    power_sw_pressed = pressed;
  }
}

void shutdown() {
  multicore_reset_core1();

  stdio_flush();
  stdio_usb_deinit();

  gpio_set_pulls(PICO_DEFAULT_UART_TX_PIN, false, false);
  gpio_set_pulls(PICO_DEFAULT_UART_RX_PIN, false, false);
  gpio_deinit(PICO_DEFAULT_UART_TX_PIN);
  gpio_deinit(PICO_DEFAULT_UART_RX_PIN);

  // power-off peripherals
  ioex->write_port(0, 1 << IOEX_PERI_EN_PIN, 1 << IOEX_PERI_EN_PIN);
  ioex->deinit();

  pwm_audio::deinit();
  spibus_deinit();

  i2c_deinit(i2c1);
  gpio_deinit(I2C_SDA_PIN);
  gpio_deinit(I2C_SCL_PIN);

  gpio_deinit(XIAO_LED_Y_PIN);
  gpio_pull_up(XIAO_LED_Y_PIN);

  gpio_deinit(XIAO_BAT_ADC_EN_PIN);
  gpio_pull_down(XIAO_BAT_ADC_EN_PIN);

  gpio_deinit(XIAO_NEO_PWR_PIN);
  gpio_pull_down(XIAO_NEO_PWR_PIN);

#if 1
  gpio_init(POWER_SW_PIN);
  gpio_set_dir(POWER_SW_PIN, false);
  powman_enable_gpio_wakeup(0, POWER_SW_PIN, false, true);

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

}  // namespace shapones::xiao::rp::power
