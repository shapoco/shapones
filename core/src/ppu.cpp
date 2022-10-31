#include "shapones/shapones.hpp"

namespace nes::ppu {

static Registers reg;
//static RenderContext ctx;
static int lead_x;
static int lead_y;
static int focus_x;
static int focus_y;
static int shift_x;

//static ScrollRegister scroll;
static uint16_t scroll;

static int startup_delay_counter;

static uint8_t bus_read_data_latest = 0;
static uint8_t bus_read_data_delayed = 0;

static bool scroll_ppuaddr_high_stored = false;

//static uint8_t line_buff[SCREEN_WIDTH];

static Palette palette_file[PALETTE_NUM_BANK];

static OamEntry oam[MAX_SPRITE_COUNT];
static SpriteLine visible_sprites[MAX_VISIBLE_SPRITES];
static int num_visible_sprites;

static void hold_reset_state();

static uint8_t bus_read(addr_t addr);
static void bus_write(addr_t addr, uint8_t data);
static uint8_t oam_read(addr_t addr);
static void oam_write(addr_t addr, uint8_t data);
static uint8_t palette_read(addr_t addr);
static void palette_write(addr_t addr, uint8_t data);

static void render_bg_block(uint8_t *line_buff, int x0, int x1);
static void render_sprite_block(uint8_t *line_buff, int x0, int x1);
static void update_scroll_counter(int x0, int x1);
static void enum_visible_sprites();

void reset() {
    hold_reset_state();
    startup_delay_counter = STARTUP_DELAY_CYCLES;
}

// https://www.nesdev.org/wiki/PPU_power_up_state
static void hold_reset_state() {
    reg.control.raw = 0;
    reg.mask.raw = 0;
    reg.oam_addr = 0;
    reg.scroll = 0;
    reg.status.raw = 0;
    reg.fine_x = 0;

    bus_read_data_latest = 0;
    bus_read_data_delayed = 0;

    lead_x = 0;
    lead_y = 0;
    focus_x = 0;
    focus_y = 0;

    num_visible_sprites = 0;

    scroll_ppuaddr_high_stored = false;
}

bool is_in_hblank() {
    return focus_x >= SCREEN_WIDTH;
}

int current_focus_y() {
    return focus_y; 
}

uint8_t reg_read(addr_t addr) {
    uint8_t retval;
    switch(addr) {
    case REG_PPUSTATUS:
        {
            retval = reg.status.raw;
            reg.status.vblank_flag = 0;
            scroll_ppuaddr_high_stored = false;
        }
        break;

    case REG_OAMDATA:
        retval = oam_read(reg.oam_addr);
        break;

    case REG_PPUDATA:
        {
            addr_t addr = scroll & SCROLL_MASK_PPU_ADDR;
            bus_read(addr);
            addr += reg.control.incr_stride ? 32 : 1;
            addr &= SCROLL_MASK_PPU_ADDR;
            scroll &= ~SCROLL_MASK_PPU_ADDR;
            scroll |= addr;
            retval = bus_read_data_delayed;
        }
        break;

    default:
        NES_PRINTF("*Warning: Invalid PPU register read addr: 0x%x\n", addr);
        retval = 0;
        break;
    }
    return retval;
}

void reg_write(addr_t addr, uint8_t data) {
    switch(addr) {
    case REG_PPUCTRL:
        // name sel bits
        reg.scroll &= 0xf3ff;
        reg.scroll |= (uint16_t)(data & 0x3) << 10;
        // other bits
        reg.control.raw = data;
        break;

    case REG_PPUMASK:
        reg.mask.raw = data;
        break;

    case REG_OAMADDR:
        reg.oam_addr = data;
        break;

    case REG_OAMDATA:
        oam_write(reg.oam_addr, data);
        break;

    case REG_PPUSCROLL:
        if ( ! scroll_ppuaddr_high_stored) {
            reg.scroll &= ~SCROLL_MASK_COARSE_X;
            reg.scroll |= (data >> 3) & SCROLL_MASK_COARSE_X;
            reg.fine_x = data & 0x7;
            scroll_ppuaddr_high_stored = true;
        }
        else {
            reg.scroll &= ~(SCROLL_MASK_COARSE_Y | SCROLL_MASK_FINE_Y);
            reg.scroll |= ((uint16_t)data << 2) & SCROLL_MASK_COARSE_Y;
            reg.scroll |= ((uint16_t)data << 12) & SCROLL_MASK_FINE_Y;
            scroll_ppuaddr_high_stored = false;
        }
        break;

    case REG_PPUADDR:
        if ( ! scroll_ppuaddr_high_stored) {
            reg.scroll &= 0x00ffu;
            reg.scroll |= ((uint16_t)data << 8) & 0x3f00u;
            scroll_ppuaddr_high_stored = true;
        }
        else {
            reg.scroll &= 0xff00u;
            reg.scroll |= (uint16_t)data;
            scroll = reg.scroll;
            scroll_ppuaddr_high_stored = false;
        }
        break;

    case REG_PPUDATA:
        {
            addr_t addr = scroll & SCROLL_MASK_PPU_ADDR;
            bus_write(addr, data);
            addr += reg.control.incr_stride ? 32 : 1;
            addr &= SCROLL_MASK_PPU_ADDR;
            scroll &= ~SCROLL_MASK_PPU_ADDR;
            scroll |= addr;
        }
        break;

    default:
        NES_PRINTF("*Warning: Invalid PPU register write addr: 0x%x\n", addr);
    }
}

void oam_dma_write(addr_t offset, uint8_t data) {
    oam_write((reg.oam_addr + offset) % OAM_SIZE, data);
}

static uint8_t bus_read(addr_t addr) {
    if (CHRROM_BASE <= addr && addr < CHRROM_BASE + CHRROM_SIZE) {
        bus_read_data_delayed = bus_read_data_latest;
        bus_read_data_latest = memory::chrrom[addr - CHRROM_BASE];
    }
    else if (VRAM_BASE <= addr && addr < VRAM_BASE + memory::VRAM_SIZE) {
        bus_read_data_delayed = bus_read_data_latest;
        bus_read_data_latest = memory::vram_read(addr - VRAM_BASE);
    }
    else if (PALETTE_FILE_BASE <= addr && addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
        bus_read_data_delayed = palette_read(addr - PALETTE_FILE_BASE);
        bus_read_data_latest = bus_read_data_delayed;
    }
    else if (VRAM_MIRROR_BASE <= addr && addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
        bus_read_data_delayed = bus_read_data_latest;
        bus_read_data_latest = memory::vram_read(addr - VRAM_MIRROR_BASE);
    }
    else {
        bus_read_data_delayed = 0;
        bus_read_data_latest = 0;
    }
    return bus_read_data_latest;
}

static void bus_write(addr_t addr, uint8_t data) {
    if (VRAM_BASE <= addr && addr < VRAM_BASE + memory::VRAM_SIZE) {
        memory::vram_write(addr - VRAM_BASE, data);
    }
    else if (PALETTE_FILE_BASE <= addr && addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
        palette_write(addr - PALETTE_FILE_BASE, data);
    }
    else if (VRAM_MIRROR_BASE <= addr && addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
        memory::vram_write(addr - VRAM_MIRROR_BASE, data);
    }
    else {
        //NES_PRINTF("*Warning: Invalid PPU bus write addr: 0x%x\n", addr);
    }
}

static uint8_t oam_read(addr_t addr) {
    switch (addr & 0x3) {
    case 0: return oam[addr / 4].y;
    case 1: return oam[addr / 4].tile;
    case 2: return oam[addr / 4].attr;
    default: return oam[addr / 4].x;
    }
}

static void oam_write(addr_t addr, uint8_t data) {
    switch (addr & 0x3) {
    case 0: oam[addr / 4].y = data; break;
    case 1: oam[addr / 4].tile = data; break;
    case 2: oam[addr / 4].attr = data; break;
    default: oam[addr / 4].x = data; break;
    }
}

static uint8_t palette_read(addr_t addr) {
    addr %= PALETTE_FILE_SIZE;
    switch (addr) {
    case 0x10: return palette_file[0].color[0];
    case 0x14: return palette_file[1].color[0];
    case 0x18: return palette_file[2].color[0];
    case 0x1c: return palette_file[3].color[0];
    default: return palette_file[addr / 4].color[addr & 0x3];
    }
}

static void palette_write(addr_t addr, uint8_t data) {
    addr %= PALETTE_FILE_SIZE;
    switch (addr) {
    case 0x10: palette_file[0].color[0] = data; break;
    case 0x14: palette_file[1].color[0] = data; break;
    case 0x18: palette_file[2].color[0] = data; break;
    case 0x1c: palette_file[3].color[0] = data; break;
    default: palette_file[addr / 4].color[addr & 0x3] = data; break;
    }
}

void render_next_block(uint8_t *line_buff, int num_cycles) {
    if (startup_delay_counter > 0) {
        startup_delay_counter--;
        hold_reset_state();
        return;
    }

    // step lead counter
    lead_x += num_cycles;
    if (lead_x >= LINE_CYCLES) {
        lead_x -= LINE_CYCLES;
        lead_y++;
        if (lead_y == SCREEN_HEIGHT + 1) {
            // vblank flag/interrupt
            reg.status.vblank_flag = 1;
            if (reg.control.vblank_irq_enable) {
                interrupt::assert_nmi();
            }
        }
        else if (lead_y == SCAN_LINES - 1) {
            // clear flags
            reg.status.vblank_flag = 0;
            reg.status.sprite0_hit = 0;
        }
        else if (lead_y >= SCAN_LINES) {
            // vertical recovery
            lead_y = 0;
        }
    }

    while (true) {

        // determine rendering block area
        int next_focus_x;
        if (focus_x == 0 && focus_y < SCREEN_HEIGHT) {
            // head of line
            int coarse_x = reg.scroll & SCROLL_MASK_COARSE_X;
            next_focus_x = BLOCK_SIZE - (((coarse_x * TILE_SIZE) + reg.fine_x) & (BLOCK_SIZE - 1));
        }
        else if (focus_x < SCREEN_WIDTH && focus_y < SCREEN_HEIGHT) {
            // visible area
            next_focus_x = focus_x + BLOCK_SIZE;
            if (next_focus_x > SCREEN_WIDTH) {
                next_focus_x = SCREEN_WIDTH;
            }
        }
        else {
            // blanking area
            next_focus_x = focus_x + BLOCK_SIZE;
            if (next_focus_x > LINE_CYCLES) {
                next_focus_x = LINE_CYCLES;
            }
        }

        // if the new focus does not overtake the lead counter, render the area
        if (focus_y != lead_y || next_focus_x <= lead_x) {
            if (focus_x < SCREEN_WIDTH && focus_y < SCREEN_HEIGHT) {
                if (reg.mask.bg_enable) {
                    // render background
                    render_bg_block(line_buff, focus_x, next_focus_x);
                }
                else {
                    // blank background
                    uint8_t bg_color = palette_file[0].color[0];
                    for (int x = focus_x; x < next_focus_x; x++) {
                        line_buff[x] = bg_color;
                    }
                }

                if (reg.mask.sprite_enable) {
                    // render sprite
                    if (focus_x == 0) {
                        enum_visible_sprites();
                    }
                    render_sprite_block(line_buff, focus_x, next_focus_x);
                }
            }
            
            update_scroll_counter(focus_x, next_focus_x);

            // update focus counter
            focus_x = next_focus_x;
            if (focus_x >= LINE_CYCLES) {
                focus_x = 0;
                focus_y = lead_y;
            }
        }
        else {
            break;
        }
    }
}

static void render_bg_block(uint8_t *line_buff, int x0, int x1) {
    // read name table for two tiles
    addr_t name_addr0 = scroll & 0xffeu;
    addr_t name_addr1 = name_addr0 + 1;
    uint8_t name0 = memory::vram_read(name_addr0);
    uint8_t name1 = memory::vram_read(name_addr1);

    int fine_y = (scroll & SCROLL_MASK_FINE_Y) >> 12;

    // read CHRROM
    int chrrom_index0 = name0 * 8 + fine_y;
    int chrrom_index1 = name1 * 8 + fine_y;
    if (reg.control.bg_name_sel) {
        chrrom_index1 += 0x800;
        chrrom_index0 += 0x800;
    }
    uint16_t chr0 = memory::chrrom_reordered[chrrom_index0];
    uint16_t chr1 = memory::chrrom_reordered[chrrom_index1];

    uint32_t chr = (uint32_t)chr0 | ((uint32_t)chr1 << 16);
    
    // calc attr index
    int attr_index = (scroll & SCROLL_MASK_NAME_SEL) + 0x3c0;
    int coarse_y = (scroll & SCROLL_MASK_COARSE_Y) >> 5;
    int coarse_x = scroll & SCROLL_MASK_COARSE_X;
    attr_index += (coarse_y / 4) * (NUM_TILE_X / 4);
    attr_index += coarse_x / 4;
    int attr_shift_size = ((coarse_y & 0x2) << 1) | (coarse_x & 0x2);

    // read attr table
    uint8_t attr = memory::vram_read(attr_index);
    attr = (attr >> attr_shift_size) & 0x3;
    Palette palette = palette_file[attr];

    // adjust CHR bit pos
    int chr_shift_size = ((coarse_x & 1) << 4) | (shift_x << 1);
    chr >>= chr_shift_size;

    // render BG block
    uint8_t bg_color = palette_file[0].color[0];
    for (int x = x0; x < x1; x++) {
        int palette_index = chr & 0x3;
        chr >>= 2;
        if (palette_index == 0) {
            line_buff[x] = bg_color;
        }
        else {
            line_buff[x] = palette.color[palette_index] | OPAQUE_FLAG;
        }
    }
}

static void enum_visible_sprites() {
    num_visible_sprites = 0;
    uint8_t h = reg.control.sprite_size ? 16 : 8;
    for (int i = 0; i < MAX_SPRITE_COUNT; i++) {
        auto s = oam[i];

        // vertical hit test
        if (s.y + SPRITE_Y_OFFSET <= focus_y && focus_y < s.y + h + SPRITE_Y_OFFSET) {
            int src_y = focus_y - (s.y + SPRITE_Y_OFFSET);
            
            if (s.attr & OAM_ATTR_INVERT_V) {
                // vertical inversion
                if (reg.control.sprite_size) {
                    src_y ^= 0xf;
                }
                else {
                    src_y ^= 0x7;
                }
            }
            
            // tile index calculation
            int tile_index;
            if (reg.control.sprite_size) {
                // 8x16 sprite
                if (src_y < 8) {
                    tile_index = s.tile & 0xfe;
                }
                else {
                    tile_index = s.tile | 0x01;
                }
            
                if (s.tile & 0x1) {
                    tile_index += 0x1000 / 16;
                }
            }
            else {
                // 8x8 sprite
                tile_index = s.tile;
                if (reg.control.sprite_name_sel) {
                    tile_index += 0x1000 / 16;
                }
            }
            
            // read CHRROM
            int chrrom_index = tile_index * 8 + (src_y & 0x7);
            uint16_t chr = memory::chrrom_reordered[chrrom_index];

            if (s.attr & OAM_ATTR_INVERT_H) {
                // horizontal inversion
                chr = 
                    ((chr << 14) & 0xc000) |
                    ((chr << 10) & 0x3000) |
                    ((chr <<  6) & 0x0c00) |
                    ((chr <<  2) & 0x0300) |
                    ((chr >>  2) & 0x00c0) |
                    ((chr >>  6) & 0x0030) |
                    ((chr >> 10) & 0x000c) |
                    ((chr >> 14) & 0x0003);
            }

            // store sprite information
            SpriteLine vs;
            vs.chr = chr;
            vs.x = s.x;
            vs.behind_of_bg = (s.attr & OAM_ATTR_PRIORITY) != 0;
            vs.palette = palette_file[4 + (s.attr & OAM_ATTR_PALETTE)];
            visible_sprites[num_visible_sprites++] = vs;

            if (num_visible_sprites >= MAX_VISIBLE_SPRITES) {
                break;
            }
        }
    }

    for (int i = num_visible_sprites; i < MAX_VISIBLE_SPRITES; i++) {
        visible_sprites[i].x = 255;
    }
}

static void render_sprite_block(uint8_t *line_buff, int x0_block, int x1_block) {
    // sprite height
    int h = reg.control.sprite_size ? 16 : 8;
    for (int i = 0; i < num_visible_sprites; i++) {
        auto s = visible_sprites[i];
        int x0 = x0_block > s.x ? x0_block : s.x;
        int x1 = x1_block < s.x + TILE_SIZE ? x1_block : s.x + TILE_SIZE;
        uint32_t chr = s.chr >> (2 * (x0 - s.x));
        for (int x = x0; x < x1; x++) {
            int palette_index = chr & 0x3;
            chr >>= 2;
            bool sprite_opaque = (palette_index != 0);
            bool bg_opaque = (line_buff[x] & OPAQUE_FLAG) != 0;
            if (sprite_opaque && (!bg_opaque || !s.behind_of_bg)) {
                line_buff[x] = s.palette.color[palette_index];
            }
            if (sprite_opaque && bg_opaque) {
                reg.status.sprite0_hit = 1;
            }
        }
    }
}

static void update_scroll_counter(int x0, int x1) {
    if (!reg.mask.bg_enable && !reg.mask.sprite_enable) {
        return;
    }

    int n = x1 - x0;
    if (focus_y < SCREEN_HEIGHT) {
        if (focus_x < SCREEN_WIDTH) {
            shift_x += n;
            while (shift_x >= TILE_SIZE) {
                shift_x -= TILE_SIZE;
                // if coarse_x < 31
                if ((scroll & SCROLL_MASK_COARSE_X) < SCROLL_MASK_COARSE_X) {
                    scroll++; // coarse_x++
                }
                else {
                    // right edge of name table
                    scroll &= ~SCROLL_MASK_COARSE_X; // coarse_x = 0
                    scroll ^= 0x0400u; // switch name table horizontally
                }
            }
        }
        else if (focus_x == SCREEN_WIDTH) {
            // if fine_y < 7
            if ((scroll & SCROLL_MASK_FINE_Y) < SCROLL_MASK_FINE_Y) {
                scroll += 0x1000u; // fine_y++
            }
            else {
                // bottom edge of tile
                scroll &= ~SCROLL_MASK_FINE_Y; // fine_y = 0
                // if coarse_y == 29
                if ((scroll & SCROLL_MASK_COARSE_Y) == ((NUM_TILE_Y - 1) << 5)) { 
                    // bottom edge of name table
                    scroll &= ~SCROLL_MASK_COARSE_Y; // coarse_y = 0
                    scroll ^= 0x0800u; // switch name table vertically
                }
                // else if coarse_y == 31
                else if ((scroll & SCROLL_MASK_COARSE_Y) == SCROLL_MASK_COARSE_Y) {
                    scroll &= ~SCROLL_MASK_COARSE_Y; // coarse_y = 0
                }
                else {
                    scroll += NUM_TILE_X; // coarse_y++
                }
            }

            // horizontal recovery
            constexpr uint16_t copy_mask = 0x041fu;
            scroll &= ~copy_mask;
            scroll |= reg.scroll & copy_mask;
            shift_x = reg.fine_x;
        }
    }
    else if (focus_y == SCAN_LINES - 1) {
        if (280 <= focus_x && focus_x <= 304) {
            // vertical recovery
            constexpr uint16_t copy_mask = 0x7be0u;
            scroll &= ~copy_mask;
            scroll |= reg.scroll & copy_mask;
        }
    }
}

}
