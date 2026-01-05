#include "../include.h"

#include "../../../_sdk/inc/sdk_dma.h"
#include "../../../_sdk/inc/sdk_gpio.h"
#include "../../../_sdk/inc/sdk_spi.h"

#include "mono8x16.hpp"
#include "shapones/shapones.hpp"
#include "tmp.rom.hpp"

static constexpr uint16_t NUM_WORKERS = 2;

static int feed_index = 0;

static constexpr int FRAME_BUFF_STRIDE = nes::SCREEN_WIDTH * 3 / 2;

static uint8_t line_buff[nes::SCREEN_WIDTH];
static uint8_t frame_buff[FRAME_BUFF_STRIDE * nes::SCREEN_HEIGHT];

static int fps_frame_count = 0;
static uint64_t fps_last_meas_ms = 0;
static char fps_str[16];

static volatile bool disp_update_req = false;

// NES color table
static const uint16_t COLOR_TABLE[] = {
    0x555, 0x027, 0x019, 0x308, 0x406, 0x603, 0x500, 0x410, 0x230, 0x040, 0x040,
    0x040, 0x034, 0x000, 0x000, 0x000, 0x999, 0x05C, 0x33F, 0x62E, 0x81B, 0xA16,
    0x922, 0x740, 0x560, 0x270, 0x080, 0x072, 0x067, 0x000, 0x000, 0x000, 0xFFF,
    0x5AF, 0x78F, 0xB6F, 0xE5F, 0xF5B, 0xF76, 0xD82, 0xAB0, 0x7C0, 0x5D2, 0x3D7,
    0x3BD, 0x444, 0x000, 0x000, 0xFFF, 0xADF, 0xCCF, 0xDBF, 0xFBF, 0xFBD, 0xFBB,
    0xEC9, 0xDD7, 0xBE7, 0xAE9, 0x9EB, 0xADE, 0xAAA, 0x000, 0x000,
};

static void core1_main();

static void disp_enable_rgb444();
static void disp_dma_start();
static void disp_dma_complete();
static void disp_wait_vsync();

static int disp_dma_ch;
static spi_hw_t *disp_spi_hw = nullptr;
static dma_channel_hw_t *disp_dma_hw = nullptr;

uint64_t disp_next_vsync_us = 0;

int main() {
#if USE_PICOPAD10 || USE_PICOPAD20
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    WaitMs(100);
    set_sys_clock_khz(250000, true);
#endif

    disp_enable_rgb444();

    disp_spi_hw = SPI_GetHw(DISP_SPI);

    nes::memory::map_ines(nes_rom);
    nes::apu::set_sampling_rate(22050);
    nes::reset();

    Core1Exec(core1_main);

    while (True) {
        uint64_t now_ms = Time64() / 1000;
        nes::cpu::service();
    }
}

static void core1_main() {
    while (true) {
        uint64_t now_ms = Time64() / 1000;

        bool eol = nes::ppu::service(line_buff);

        int y = nes::ppu::current_focus_y();

        // end of visible line
        if (eol && y < nes::SCREEN_HEIGHT) {
            // Convert palette index to RGB444
            uint8_t *rd_ptr = line_buff;
            uint8_t *wr_ptr = frame_buff + (y * FRAME_BUFF_STRIDE);
            for (int x = 0; x < nes::SCREEN_WIDTH; x += 2) {
                uint16_t c0 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                uint16_t c1 = COLOR_TABLE[*(rd_ptr++) & 0x3f];
                *(wr_ptr++) = (c0 >> 4) & 0xff;
                *(wr_ptr++) = ((c0 << 4) & 0xf0) | (c1 >> 8) & 0xf;
                *(wr_ptr++) = c1 & 0xff;
            }

            // end of frame
            if (y == 0) {
                uint64_t now_ms = Time64() / 1000;

                // measure FPS
                fps_frame_count += 1;
                uint32_t t_elapsed = now_ms - fps_last_meas_ms;
                if (t_elapsed >= 1000) {
                    float fps = (float)(fps_frame_count * 1000) / t_elapsed;
                    snprintf(fps_str, sizeof(fps_str), "%5.2f FPS", fps);
                    fps_last_meas_ms = now_ms;
                    fps_frame_count = 0;
                }

                mono8x16::draw_string_rgb444(
                    frame_buff, FRAME_BUFF_STRIDE, nes::SCREEN_WIDTH,
                    nes::SCREEN_HEIGHT, 0, 0, fps_str, 0x000);

                // complete previous DMA
                disp_dma_complete();
                GPIO_Out1(DISP_CS_PIN);

                // select window
                DispWindow(0, nes::SCREEN_WIDTH, 0, nes::SCREEN_HEIGHT);

                // kick new DMA
                GPIO_Out0(DISP_CS_PIN);
                GPIO_Out1(DISP_DC_PIN);
                disp_dma_start();

                disp_wait_vsync();
            }
        }
    }
}

static void disp_enable_rgb444() {
    uint8_t cmd = 0x3A;
    uint8_t data = 0x03;
    GPIO_Out0(DISP_CS_PIN);
    GPIO_Out0(DISP_DC_PIN);
    SPI_Send8(DISP_SPI, &cmd, 1);
    GPIO_Out1(DISP_DC_PIN);
    SPI_Send8(DISP_SPI, &data, 1);
    GPIO_Out1(DISP_CS_PIN);
}

static void disp_dma_start() {
    disp_dma_ch = DMA_TEMP_CHAN();
    disp_dma_hw = DMA_GetHw(disp_dma_ch);

    // configure DMA
    DMA_Abort(disp_dma_ch);
    DMA_ClearError_hw(disp_dma_hw);
    DMA_SetRead_hw(disp_dma_hw, frame_buff);
    DMA_SetWrite_hw(disp_dma_hw, &disp_spi_hw->dr);
    DMA_SetCount_hw(disp_dma_hw, FRAME_BUFF_STRIDE * nes::SCREEN_HEIGHT);
    cb();
    DMA_SetCtrlTrig_hw(disp_dma_hw,
                       DMA_CTRL_TREQ(SPI_GetDreq_hw(disp_spi_hw, True)) |
                           DMA_CTRL_CHAIN(disp_dma_ch) | DMA_CTRL_INC_READ |
                           DMA_CTRL_SIZE(DMA_SIZE_8) | DMA_CTRL_EN);
}

static void disp_dma_complete() {
    if (!disp_dma_hw) return;

    DMA_Wait_hw(disp_dma_hw);
    DMA_Disable_hw(disp_dma_hw);

    while (SPI_IsBusy_hw(disp_spi_hw)) {
        SPI_RxFlush_hw(disp_spi_hw);
    }

    SPI_RxFlush_hw(disp_spi_hw);
    SPI_RxOverClear_hw(disp_spi_hw);
}

static void disp_wait_vsync() {
    constexpr uint32_t FRAME_INTERVAL_US = 1000000 / 60;
    uint64_t now_us;
    do {
        now_us = Time64();
    } while (now_us < disp_next_vsync_us);
    if (now_us < disp_next_vsync_us + FRAME_INTERVAL_US) {
        disp_next_vsync_us += FRAME_INTERVAL_US;
    } else {
        disp_next_vsync_us = now_us;
    }
}