
#include "shapones/xiao/app.hpp"
#include "shapones/xiao/display.hpp"
#include "shapones/xiao/spi.h"
#include "shapones/xiao/timer.h"

#include <hardware/clocks.h>
#include <hardware/vreg.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

namespace shapones::xiao::rp {

static constexpr uint32_t SYS_CLK_FREQ = 250'000'000;

void core0_main();
void core1_main();

void core0_main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(100);
  set_sys_clock_khz(SYS_CLK_FREQ / 1000, true);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                  SYS_CLK_FREQ, SYS_CLK_FREQ);

  app_init();
  multicore_launch_core1(core1_main);

  while (true) {
    cpu_service();
  }
}

void core1_main() {
  while (true) {
    ppu_service();
  }
}

}  // namespace shapones::xiao::rp

int main() { shapones::xiao::rp::core0_main(); }
