#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <hardware/vreg.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/util/queue.h>

#include <stdio.h>

#include "ws19804.hpp"

#include "pwm_audio.hpp"

#include "shapones/shapones.hpp"

#include "boot_menu.hpp"
#include "common.hpp"

#define SHAPONES_DISP_DMA_CORE (0)

// monitor pin for debugging
static constexpr int PIN_MONITOR = 1;

// speaker PWM out
static constexpr int PIN_SPEAKER = 28;

// APU configuration
static constexpr int SPK_LATENCY = 256;
static constexpr int SPK_PWM_FREQ = 22050;

// NES color table
static const uint16_t COLOR_TABLE[] = {
    0x555, 0x027, 0x019, 0x308, 0x406, 0x603, 0x500, 0x410, 0x230, 0x040, 0x040,
    0x040, 0x034, 0x000, 0x000, 0x000, 0x999, 0x05C, 0x33F, 0x62E, 0x81B, 0xA16,
    0x922, 0x740, 0x560, 0x270, 0x080, 0x072, 0x067, 0x000, 0x000, 0x000, 0xFFF,
    0x5AF, 0x78F, 0xB6F, 0xE5F, 0xF5B, 0xF76, 0xD82, 0xAB0, 0x7C0, 0x5D2, 0x3D7,
    0x3BD, 0x444, 0x000, 0x000, 0xFFF, 0xADF, 0xCCF, 0xDBF, 0xFBF, 0xFBD, 0xFBB,
    0xEC9, 0xDD7, 0xBE7, 0xAE9, 0x9EB, 0xADE, 0xAAA, 0x000, 0x000,
};

// line buffer FIFO between core1 --> core0
// <------------- STRIDE ------------>
// +---------+-----------------------+    A
// | y coord | color numbers (256px) |    |
// | 1 Byte  | 256 Byte              |    |
// +---------+-----------------------+  DEPTH
// |    :    |           :           |    |
// |    :    |           :           |    |
// +---------+-----------------------+    V
static constexpr int LINE_FIFO_DEPTH = 8;
static constexpr int LINE_FIFO_STRIDE = nes::SCREEN_WIDTH + 1;
static uint8_t line_buff[LINE_FIFO_DEPTH * LINE_FIFO_STRIDE];
static volatile int line_fifo_wptr = 0;
static volatile int line_fifo_rptr = 0;

static spin_lock_t *lock_hws[nes::NUM_LOCKS];
static uint32_t lock_irqs[nes::NUM_LOCKS];

// sound buffer for DMA
static uint8_t spk_buff[SPK_LATENCY];

// next vsync time in microseconds
static uint64_t next_vsync_us = 0;

// start game
static void boot_nes();

// core0 main loop
static void cpu_loop();

// core1 main loop
static void ppu_loop();

// wait for next vsync
static bool wait_vsync();

// APU DMA finish IRQ handler
static void apu_dma_handler();

// let APU fill the sound buffer
static void apu_fill_buffer(PwmAudio::sample_t *buff);

// PWM audio driver
PwmAudio speaker(PIN_SPEAKER, SPK_LATENCY, 8,
                 (float)SYS_CLK_FREQ / SPK_PWM_FREQ, apu_dma_handler);

int main() {
  // setup clocks
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(100);
  stdio_init_all();
  set_sys_clock_khz(SYS_CLK_FREQ / 1000, true);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                  SYS_CLK_FREQ, SYS_CLK_FREQ);
  setup_default_uart();

  // setup monitor pin
  gpio_init(PIN_MONITOR);
  gpio_set_dir(PIN_MONITOR, GPIO_OUT);
  gpio_put(PIN_MONITOR, 0);

  // setup input pins
  for (int i = 0; i < 8; i++) {
    int pin = input_pins[i];
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
  }

  // setup WAVESHARE-19804
  ws19804::init(SYS_CLK_FREQ);

  // show boot menu
  if (!boot_menu()) {
    for (;;) sleep_ms(100);
  }

  // boot game
  boot_nes();

  return 0;
}

static void boot_nes() {
  // reset
  auto cfg = nes::get_default_config();
  cfg.apu_sampling_rate = SPK_PWM_FREQ;
  nes::init(cfg);

  // start APU loop
  apu_fill_buffer(speaker.get_buffer(0));
  apu_fill_buffer(speaker.get_buffer(1));
  speaker.play();

  // start PPU loop
  multicore_launch_core1(ppu_loop);

  // start CPU loop
  cpu_loop();
}

static void cpu_loop() {
#if SHAPONES_DISP_DMA_CORE == 0
  auto t_last_frame = get_absolute_time();
  int frame_count = 0;
  char fps_str[16];
#endif
  for (;;) {
    // update input status
    nes::input::InputStatus input_status;
    input_status.raw = 0;
    for (int i = 0; i < 8; i++) {
      if (!gpio_get(input_pins[i])) {
        input_status.raw |= (1 << i);
      }
    }
    nes::input::set_raw(0, input_status);

    // run CPU
    nes::cpu::service();

#if SHAPONES_DISP_DMA_CORE == 0
    // check line buffer FIFO state
    int fifo_rptr = line_fifo_rptr;
    if (line_fifo_wptr != fifo_rptr) {
      // convert color number to RGB444
      int y = line_buff[fifo_rptr * LINE_FIFO_STRIDE];
      int x0 = (nes::SCREEN_WIDTH - FRAME_BUFF_WIDTH) / 2;
      uint8_t *rd_ptr = line_buff + (fifo_rptr * LINE_FIFO_STRIDE + x0 + 1);
      uint8_t *wr_ptr = frame_buff + (y * FRAME_BUFF_STRIDE);
      for (int x = 0; x < FRAME_BUFF_WIDTH; x += 2) {
        uint_fast16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
        uint_fast16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
        *(wr_ptr++) = (c0 >> 4) & 0xff;
        *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
        *(wr_ptr++) = c1 & 0xff;
      }
      line_fifo_rptr = (fifo_rptr + 1) % LINE_FIFO_DEPTH;

      if (y == nes::SCREEN_HEIGHT - 1) {
        // fps measurement
        auto t_now = get_absolute_time();
        if (frame_count < 60 - 1) {
          frame_count++;
        } else {
          auto t_diff = absolute_time_diff_us(t_last_frame, t_now);
          float fps = (60.0f * 1000000) / t_diff;
          sprintf(fps_str, "%5.2ffps", fps);
          t_last_frame = t_now;
          frame_count = 0;
        }

        // DMA transfer
        ws19804::finish_write_data();
        draw_string(0, 0, fps_str);
        ws19804::start_write_data(25, 0, FRAME_BUFF_WIDTH, FRAME_BUFF_HEIGHT,
                                  frame_buff);
        static int tmp = 0;
        gpio_put(PIN_MONITOR, tmp);
        tmp ^= 1;
      }
    }
#endif
  }
}

static void ppu_loop() {
#if SHAPONES_DISP_DMA_CORE == 1
  auto t_last_frame = get_absolute_time();
  int frame_count = 0;
  char fps_str[16];
#endif

  bool skip_frame = false;

  for (;;) {
#if SHAPONES_DISP_DMA_CORE == 0
    int wptr = line_fifo_wptr;
    int y;
    uint32_t timing = nes::ppu::service(&line_buff[wptr * LINE_FIFO_STRIDE + 1],
                                        skip_frame, &y);
    if ((timing & nes::ppu::END_OF_VISIBLE_LINE) && !skip_frame) {
      // push new line
      line_buff[wptr * LINE_FIFO_STRIDE] = y;
      line_fifo_wptr = (wptr + 1) % LINE_FIFO_DEPTH;
    }
    if (timing & nes::ppu::END_OF_VISIBLE_AREA) {
      skip_frame = wait_vsync();
    }
#else
    int y;
    uint32_t timing = nes::ppu::service(line_buff, skip_frame, &y);
    if ((timing & nes::ppu::END_OF_VISIBLE_LINE) && !skip_frame) {
      int x0 = (nes::SCREEN_WIDTH - FRAME_BUFF_WIDTH) / 2;
      uint8_t *rd_ptr = line_buff;
      uint8_t *wr_ptr = frame_buff + (y * FRAME_BUFF_STRIDE);
      for (int x = 0; x < FRAME_BUFF_WIDTH; x += 2) {
        uint16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
        uint16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
        *(wr_ptr++) = (c0 >> 4) & 0xff;
        *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
        *(wr_ptr++) = c1 & 0xff;
      }
    }
    if (timing & nes::ppu::END_OF_VISIBLE_AREA) {
      if (!skip_frame) {
        // fps measurement
        auto t_now = get_absolute_time();
        if (frame_count < 60 - 1) {
          frame_count++;
        } else {
          auto t_diff = absolute_time_diff_us(t_last_frame, t_now);
          float fps = (60.0f * 1000000) / t_diff;
          sprintf(fps_str, "%5.2ffps", fps);
          t_last_frame = t_now;
          frame_count = 0;
        }

        // DMA transfer
        ws19804::finish_write_data();
        draw_string(0, 0, fps_str);
        ws19804::start_write_data(25, 0, FRAME_BUFF_WIDTH, FRAME_BUFF_HEIGHT,
                                  frame_buff);
      }
      skip_frame = wait_vsync() && !skip_frame;
    }
#endif
  }
}

static bool wait_vsync() {
  constexpr int FRAME_DELAY_US = 1000000 / 60;
  uint64_t now_us = to_us_since_boot(get_absolute_time());
  int64_t wait_us = next_vsync_us - now_us;
  bool delaying = (wait_us <= 0);
  if (!delaying) {
    sleep_us(wait_us);
  }
  next_vsync_us += FRAME_DELAY_US;
  if (now_us > next_vsync_us) {
    next_vsync_us = now_us;
  }
  return delaying;
}

static void apu_dma_handler() {
  speaker.flip_buffer();
  apu_fill_buffer(speaker.get_next_buffer());
}

static void apu_fill_buffer(PwmAudio::sample_t *buff) {
  nes::apu::service(spk_buff, speaker.LATENCY);
  for (int i = 0; i < speaker.LATENCY; i++) {
    buff[i] = spk_buff[i];
  }
}

void nes::lock_init(int id) {
  lock_hws[id] = spin_lock_init(id);
  spin_lock_claim(id);
}
void nes::lock_deinit(int id) { spin_lock_unclaim(id); }
void nes::lock_get(int id) { lock_irqs[id] = spin_lock_blocking(lock_hws[id]); }
void nes::lock_release(int id) { spin_unlock(lock_hws[id], lock_irqs[id]); }
