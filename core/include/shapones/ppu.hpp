#ifndef SHAPONES_PPU_HPP
#define SHAPONES_PPU_HPP

#include "shapones/shapones.hpp"

namespace nes::ppu {

static constexpr cycle_t MAX_DELAY_CYCLES = 128;

static constexpr int TILE_SIZE = 8;
static constexpr int NUM_TILE_X = SCREEN_WIDTH / TILE_SIZE;
static constexpr int NUM_TILE_Y = SCREEN_HEIGHT / TILE_SIZE;
static constexpr int BLOCK_SIZE = TILE_SIZE * 2;

static constexpr int REG_SIZE = 8;
static constexpr addr_t REG_PPUCTRL     = 0x2000;
static constexpr addr_t REG_PPUMASK     = 0x2001;
static constexpr addr_t REG_PPUSTATUS   = 0x2002;
static constexpr addr_t REG_OAMADDR     = 0x2003;
static constexpr addr_t REG_OAMDATA     = 0x2004;
static constexpr addr_t REG_PPUSCROLL   = 0x2005;
static constexpr addr_t REG_PPUADDR     = 0x2006;
static constexpr addr_t REG_PPUDATA     = 0x2007;

static constexpr int PALETTE_SIZE = 4;
static constexpr int PALETTE_NUM_BANK = 8;
static constexpr int PALETTE_FILE_SIZE = PALETTE_SIZE * PALETTE_NUM_BANK;
static constexpr int PALETTE_FILE_SIZE_WITH_MIRROR = 0x100;

static constexpr uint8_t OPAQUE_FLAG = 0x80;
static constexpr uint8_t BEHIND_FLAG = 0x40;

static constexpr addr_t CHRROM_BASE = 0x0000;
static constexpr addr_t VRAM_BASE = 0x2000;
static constexpr addr_t VRAM_MIRROR_BASE = 0x3000;
static constexpr addr_t VRAM_MIRROR_SIZE = 0xf00;
static constexpr addr_t PALETTE_FILE_BASE = 0x3f00;

static constexpr addr_t OAM_DMA_SRC_ADDR = 0x200;

static constexpr int MAX_VISIBLE_SPRITES = 8;
static constexpr int SPRITE_Y_OFFSET = 1;

static constexpr int LINE_CYCLES = 341;
static constexpr int SCAN_LINES = 262;

static constexpr int NAME_LINE_STRIDE = SCREEN_WIDTH / 8;
static constexpr int NAME_PAGE_STRIDE = 0x400;

static constexpr int MAX_SPRITE_COUNT = 64;
static constexpr int OAM_SIZE = MAX_SPRITE_COUNT * 4;

static constexpr uint16_t SCROLL_MASK_COARSE_X = 0x001fu;
static constexpr uint16_t SCROLL_MASK_COARSE_Y = 0x03e0u;
static constexpr uint16_t SCROLL_MASK_NAME_SEL = 0x0c00u;
static constexpr uint16_t SCROLL_MASK_FINE_Y   = 0x7000u;
static constexpr uint16_t SCROLL_MASK_PPU_ADDR = 0x3fffu;

struct Registers {
public:
    
    // Control Register
    union {
        uint8_t raw;
        struct {
            uint8_t unused : 2; // [1:0]
            uint8_t incr_stride : 1; // [2] addr increment (0:+1, 1:+32)
            uint8_t sprite_name_sel : 1; // [3] sprite pattern sel (0:0x0000, 1:0x1000)
            uint8_t bg_name_sel : 1; // [4] BG pattern sel (0:0x0000, 1:0x1000)
            uint8_t sprite_size : 1; // [5] sprite size (0:8x8, 1:8x16)
            uint8_t ppu_master : 1; // [6] ppu master/slave
            uint8_t vblank_irq_enable : 1; // [7] vblank interrupt enable
        };
    } control;

    // Mask Register
    union {
        uint8_t raw;
        struct {
            uint8_t color : 1; // [0] 0:color, 1:mono
            uint8_t bg_clip : 1; // [1] BG clipping
            uint8_t sprite_clip : 1; // [2] sprite clipping
            uint8_t bg_enable : 1; // [3] BG enable
            uint8_t sprite_enable : 1; // [4] sprite enable
            uint8_t emphasis_r : 1; // [5] emphasis R
            uint8_t emphasis_g : 1; // [6] emphasis G
            uint8_t emphasis_b : 1; // [7] emphasis B
        };
    } mask;

    // Status Register
    union {
        uint8_t raw;
        struct {
            uint8_t reserved : 5; // [4:0]
            uint8_t overflow : 1; // [5] overflow (unused)
            uint8_t sprite0_hit : 1; // [6] sprite 0 hit
            uint8_t vblank_flag : 1; // [7] vblank flag
        };
    } status;

    // OAM Indirect Address
    uint8_t oam_addr;

    // Scroll Registers
    uint8_t fine_x;
    uint16_t scroll;
};

static constexpr uint8_t OAM_ATTR_PALETTE = 0x3;
static constexpr uint8_t OAM_ATTR_PRIORITY = 0x20;
static constexpr uint8_t OAM_ATTR_INVERT_H = 0x40;
static constexpr uint8_t OAM_ATTR_INVERT_V = 0x80;

union Palette {
    uint32_t packed;
    uint8_t color[4];
};

struct OamEntry {
    uint8_t y;
    uint8_t tile;
    uint8_t attr;
    uint8_t x;
};

static constexpr uint8_t SL_ATTR_BEHIND = 0x1;
static constexpr uint8_t SL_ATTR_ZERO   = 0x2;

struct SpriteLine {
    uint8_t x;
    uint8_t attr;
    uint16_t chr;
    Palette palette;
};

static constexpr int FOCUS_HBLANK = 1;
static constexpr int FOCUS_VBLANK = 2;
static constexpr int FOCUS_1STLINE = 3;

void reset();
bool is_in_hblank();
int current_focus_y();

uint8_t reg_read(addr_t addr);
void reg_write(addr_t addr, uint8_t data);

void oam_dma_write(addr_t offset, uint8_t data);

bool service(uint8_t *line_buff);

cycle_t cycle_following();

}

#endif
