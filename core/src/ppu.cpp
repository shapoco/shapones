#include "shapones/ppu.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"

#pragma GCC optimize("Ofast")

namespace nes::ppu {

static Registers reg;

volatile cycle_t cycle_count;

static int focus_x;
static int focus_y;

static volatile uint16_t scroll;
static int fine_x;

static uint8_t bus_read_data_latest = 0;
static uint8_t bus_read_data_delayed = 0;

static bool scroll_ppuaddr_high_stored = false;

static bool nmi_level = false;

static uint8_t palette_file[PALETTE_NUM_BANK * PALETTE_SIZE];

static OamEntry oam[MAX_SPRITE_COUNT];
static SpriteLine sprite_lines[MAX_VISIBLE_SPRITES];
static int num_visible_sprites;

static volatile bool mmc3_irq_enable = false;
static volatile bool mmc3_irq_reloading = false;
static volatile uint8_t mmc3_irq_latch = 255;
static volatile uint8_t mmc3_irq_counter = 0;

static uint8_t bus_read(addr_t addr);
static void bus_write(addr_t addr, uint8_t data);
static uint8_t oam_read(addr_t addr);
static void oam_write(addr_t addr, uint8_t data);
static uint8_t palette_read(addr_t addr);
static void palette_write(addr_t addr, uint8_t data);

static void render_bg(uint8_t *line_buff, int x0, int x1, bool skip_render);
static void enum_visible_sprites(bool skip_render);
static void render_sprite(uint8_t *line_buff, int x0, int x1, bool skip_render);

void reset() {
  // https://www.nesdev.org/wiki/PPU_power_up_state
  reg.control.raw = 0;
  reg.mask.raw = 0;
  reg.oam_addr = 0;
  reg.scroll = 0;
  reg.status.raw = 0;
  reg.fine_x = 0;

  bus_read_data_latest = 0;
  bus_read_data_delayed = 0;

  focus_x = 0;
  focus_y = 0;

  num_visible_sprites = 0;

  scroll_ppuaddr_high_stored = false;

  nmi_level = false;
}

bool is_in_hblank() { return focus_x >= SCREEN_WIDTH; }

int current_focus_y() { return focus_y; }

uint8_t reg_read(addr_t addr) {
  uint8_t retval;
  switch (addr) {
    case REG_PPUSTATUS: {
      Exclusive lock(LOCK_PPU);
      retval = reg.status.raw;
      reg.status.vblank_flag = 0;
      scroll_ppuaddr_high_stored = false;
    } break;

    case REG_OAMDATA: retval = oam_read(reg.oam_addr); break;

    case REG_PPUDATA: {
      Exclusive lock(LOCK_PPU);
      addr_t addr = scroll & SCROLL_MASK_PPU_ADDR;
      bus_read(addr);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scroll &= ~SCROLL_MASK_PPU_ADDR;
      scroll |= addr;
      retval = bus_read_data_delayed;
    } break;

    default:
      SHAPONES_PRINTF("*Warning: Invalid PPU register read addr: 0x%x\n",
                      (int)addr);
      retval = 0;
      break;
  }
  return retval;
}

void reg_write(addr_t addr, uint8_t data) {
  switch (addr) {
    case REG_PPUCTRL: {
      Exclusive lock(LOCK_PPU);
      // name sel bits
      reg.scroll &= 0xf3ff;
      reg.scroll |= (uint16_t)(data & 0x3) << 10;
      // other bits
      reg.control.raw = data;
    } break;

    case REG_PPUMASK: reg.mask.raw = data; break;

    case REG_OAMADDR: reg.oam_addr = data; break;

    case REG_OAMDATA: oam_write(reg.oam_addr, data); break;

    case REG_PPUSCROLL: {
      Exclusive lock(LOCK_PPU);
      if (!scroll_ppuaddr_high_stored) {
        reg.scroll &= ~SCROLL_MASK_COARSE_X;
        reg.scroll |= (data >> 3) & SCROLL_MASK_COARSE_X;
        reg.fine_x = data & 0x7;
        scroll_ppuaddr_high_stored = true;
      } else {
        reg.scroll &= ~(SCROLL_MASK_COARSE_Y | SCROLL_MASK_FINE_Y);
        reg.scroll |= ((uint16_t)data << 2) & SCROLL_MASK_COARSE_Y;
        reg.scroll |= ((uint16_t)data << 12) & SCROLL_MASK_FINE_Y;
        scroll_ppuaddr_high_stored = false;
      }
    } break;

    case REG_PPUADDR: {
      Exclusive lock(LOCK_PPU);
      if (!scroll_ppuaddr_high_stored) {
        reg.scroll &= 0x00ffu;
        reg.scroll |= ((uint16_t)data << 8) & 0x3f00u;
        scroll_ppuaddr_high_stored = true;
      } else {
        reg.scroll &= 0xff00u;
        reg.scroll |= (uint16_t)data;
        scroll = reg.scroll;
        scroll_ppuaddr_high_stored = false;
      }
    } break;

    case REG_PPUDATA: {
      Exclusive lock(LOCK_PPU);
      uint_fast16_t scr = scroll;
      addr_t addr = scr & SCROLL_MASK_PPU_ADDR;
      bus_write(addr, data);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scr &= ~SCROLL_MASK_PPU_ADDR;
      scr |= addr;
      scroll = scr;
    } break;

    default:
      SHAPONES_PRINTF("*Warning: Invalid PPU register write addr: 0x%x\n",
                      (int)addr);
  }
}

void oam_dma_write(addr_t offset, uint8_t data) {
  oam_write((reg.oam_addr + offset) % OAM_SIZE, data);
}

static SHAPONES_INLINE uint8_t bus_read(addr_t addr) {
  if (CHRROM_BASE <= addr && addr < CHRROM_BASE + CHRROM_RANGE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = mapper::chrrom_read(addr - CHRROM_BASE);
  } else if (VRAM_BASE <= addr && addr < VRAM_BASE + VRAM_SIZE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = memory::vram_read(addr - VRAM_BASE);
  } else if (PALETTE_FILE_BASE <= addr &&
             addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
    bus_read_data_delayed = palette_read(addr - PALETTE_FILE_BASE);
    bus_read_data_latest = bus_read_data_delayed;
  } else if (VRAM_MIRROR_BASE <= addr &&
             addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = memory::vram_read(addr - VRAM_MIRROR_BASE);
  } else {
    bus_read_data_delayed = 0;
    bus_read_data_latest = 0;
  }
  return bus_read_data_latest;
}

static SHAPONES_INLINE void bus_write(addr_t addr, uint8_t data) {
  if (VRAM_BASE <= addr && addr < VRAM_BASE + VRAM_SIZE) {
    memory::vram_write(addr - VRAM_BASE, data);
  } else if (PALETTE_FILE_BASE <= addr &&
             addr < PALETTE_FILE_BASE + PALETTE_FILE_SIZE_WITH_MIRROR) {
    palette_write(addr - PALETTE_FILE_BASE, data);
  } else if (VRAM_MIRROR_BASE <= addr &&
             addr < VRAM_MIRROR_BASE + VRAM_MIRROR_SIZE) {
    memory::vram_write(addr - VRAM_MIRROR_BASE, data);
  } else {
    // SHAPONES_PRINTF("*Warning: Invalid PPU bus write addr: 0x%x\n",
    // addr);
  }
}

static SHAPONES_INLINE uint8_t oam_read(addr_t addr) {
  switch (addr & 0x3) {
    case 0: return oam[addr / 4].y;
    case 1: return oam[addr / 4].tile;
    case 2: return oam[addr / 4].attr;
    default: return oam[addr / 4].x;
  }
}

static SHAPONES_INLINE void oam_write(addr_t addr, uint8_t data) {
  switch (addr & 0x3) {
    case 0: oam[addr / 4].y = data; break;
    case 1: oam[addr / 4].tile = data; break;
    case 2: oam[addr / 4].attr = data; break;
    default: oam[addr / 4].x = data; break;
  }
}

static SHAPONES_INLINE uint8_t palette_read(addr_t addr) {
  addr %= PALETTE_FILE_SIZE;
  switch (addr) {
    case 0x10: return palette_file[0 * PALETTE_SIZE];
    case 0x14: return palette_file[1 * PALETTE_SIZE];
    case 0x18: return palette_file[2 * PALETTE_SIZE];
    case 0x1c: return palette_file[3 * PALETTE_SIZE];
    default: return palette_file[addr];
  }
}

static SHAPONES_INLINE void palette_write(addr_t addr, uint8_t data) {
  addr %= PALETTE_FILE_SIZE;
  data &= 0x3f;
  switch (addr) {
    case 0x10: palette_file[0 * PALETTE_SIZE] = data; break;
    case 0x14: palette_file[1 * PALETTE_SIZE] = data; break;
    case 0x18: palette_file[2 * PALETTE_SIZE] = data; break;
    case 0x1c: palette_file[3 * PALETTE_SIZE] = data; break;
    default: palette_file[addr] = data; return;
  }
}

uint32_t service(uint8_t *line_buff, bool skip_render, int *y) {
  uint32_t timing = 0;
  bool irq = false;

  while (true) {
    cycle_t cycle_diff = cpu::ppu_cycle_leading() - cycle_count;
    if (cycle_diff <= 0) {
      break;
    }

    // events
    if (focus_x == 0) {
      if (focus_y == SCREEN_HEIGHT + 1) {
        // vblank flag/interrupt
        Exclusive lock(LOCK_PPU);
        reg.status.vblank_flag = 1;
      } else if (focus_y == SCAN_LINES - 1) {
        // clear flags
        Exclusive lock(LOCK_PPU);
        reg.status.vblank_flag = 0;
        reg.status.sprite0_hit = 0;
      }
    }

    bool nmi_level_new =
        reg.status.vblank_flag && reg.control.vblank_nmi_enable;
    if (nmi_level_new && !nmi_level) {
      interrupt::assert_nmi();
      irq = true;
    }
    nmi_level = nmi_level_new;

    // determine step count
    int step_count;
    int dist_to_end;
    if (focus_y < SCREEN_HEIGHT && focus_x < SCREEN_WIDTH) {
      // visible area
      dist_to_end = SCREEN_WIDTH - focus_x;
    } else {
      // blank_area
      dist_to_end = LINE_CYCLES - focus_x;
    }
    step_count = dist_to_end < cycle_diff ? dist_to_end : cycle_diff;
    step_count =
        step_count < MAX_DELAY_CYCLES / 2 ? step_count : MAX_DELAY_CYCLES / 2;

    int next_focus_x = focus_x + step_count;

    if (focus_x == 0 && focus_y < SCREEN_HEIGHT && reg.mask.sprite_enable) {
      // enumerate visible sprites in current line
      enum_visible_sprites(skip_render);
    }

    // render background
    render_bg(line_buff, focus_x, next_focus_x, skip_render);

    if (focus_x < SCREEN_WIDTH && focus_y < SCREEN_HEIGHT &&
        reg.mask.sprite_enable) {
      // render sprite
      render_sprite(line_buff, focus_x, next_focus_x, skip_render);
    }

    if (mapper::get_id() == 4 && focus_y < SCREEN_HEIGHT) {
      // MMC3 IRQ counter clock
      // see: https://www.nesdev.org/wiki/MMC3
      if (focus_x <= 260 && next_focus_x > 260 && reg.mask.bg_enable &&
          reg.mask.sprite_enable) {
        uint8_t irq_counter_before = mmc3_irq_counter;
        if (mmc3_irq_counter == 0 || mmc3_irq_reloading) {
          mmc3_irq_reloading = false;
          mmc3_irq_counter = mmc3_irq_latch;
        } else {
          mmc3_irq_counter--;
        }
        if (mmc3_irq_enable && mmc3_irq_counter == 0) {
          interrupt::assert_irq(interrupt::Source::MMC3);
          irq = true;
        }
      }
    }

    // step focus
    focus_x = next_focus_x;
    if (focus_x >= LINE_CYCLES) {
      focus_x -= LINE_CYCLES;
      focus_y++;
      if (focus_y >= SCAN_LINES) {
        focus_y = 0;
        timing |= END_OF_FRAME;
      }
    }

    // step cycle counter
    cycle_count += step_count;

    if (focus_y < SCREEN_HEIGHT) {
      if (focus_x == SCREEN_WIDTH) {
        timing |= END_OF_VISIBLE_LINE;
        if (focus_y == SCREEN_HEIGHT - 1) {
          timing |= END_OF_VISIBLE_AREA;
        }
      }
    } else {
      if (focus_x == 0) {
        timing |= START_OF_VBLANK_LINE;
      }
    }

    if (timing || irq) {
      break;
    }
  }

  if (y != nullptr) {
    *y = focus_y;
  }

  return timing;
}

static void render_bg(uint8_t *line_buff, int x0_block, int x1_block,
                      bool skip_render) {
#if !SHAPONES_MUTEX_FAST
  Exclusive lock(LOCK_PPU);
#endif

  bool visible_area = (x0_block < SCREEN_WIDTH && focus_y < SCREEN_HEIGHT);
  bool bg_enabled = reg.mask.bg_enable;

  if (skip_render && num_visible_sprites == 0) {
    // If skip_render is set, only render lines where sprite #0 exists.
    visible_area = false;
  }

  uint32_t bg_offset = (uint32_t)reg.control.bg_name_sel << 12;

  while (x0_block < x1_block) {
    int x0, x1;
    {
      uint_fast16_t scr;
      {
#if SHAPONES_MUTEX_FAST
        Exclusive lock(LOCK_PPU);
#endif
        scr = scroll;
      }

      // determine step count
      x0 = x0_block;
      if (visible_area && bg_enabled) {
        x1 = x0 + (BLOCK_SIZE - ((scr & 0x1) * TILE_SIZE + fine_x));
      } else {
        x1 = x0 + 64;
      }
      x1 = x1 < x1_block ? x1 : x1_block;

      if (visible_area) {
        if (bg_enabled) {
          // read name table for two tiles
          addr_t name_addr0 = scr & 0xffeu;
          addr_t name_addr1 = name_addr0 + 1;
          uint32_t name0 = memory::vram_read(name_addr0);
          uint32_t name1 = memory::vram_read(name_addr1);

          uint32_t fine_y = (scr & SCROLL_MASK_FINE_Y) >> 12;

          uint32_t chr = 0xFFFFFFFF;
          const uint8_t *palette = nullptr;
          if (!skip_render) {
#if SHAPONES_ENABLE_CHROM_CACHE
            // read CHRROM
            uint32_t chrrom_index0 = (name0 << 3) + fine_y;
            uint32_t chrrom_index1 = (name1 << 3) + fine_y;
            chrrom_index0 += bg_offset / 2;
            chrrom_index1 += bg_offset / 2;
            uint16_t chr0 = mapper::chrrom_read_cache(chrrom_index0, false);
            uint16_t chr1 = mapper::chrrom_read_cache(chrrom_index1, false);
#else
            // read CHRROM
            uint32_t chrrom_index0 = (name0 << 4) + fine_y;
            uint32_t chrrom_index1 = (name1 << 4) + fine_y;
            chrrom_index0 += bg_offset;
            chrrom_index1 += bg_offset;
            uint_fast16_t chr0 =
                mapper::chrrom_read_double(chrrom_index0, false);
            uint_fast16_t chr1 =
                mapper::chrrom_read_double(chrrom_index1, false);
#endif
            chr = ((uint32_t)chr1 << 16) | (uint32_t)chr0;

            // adjust CHR bit pos
            int chr_shift_size = ((scr << 4) & 0x10) | (fine_x << 1);
            chr >>= chr_shift_size;

            // calc attr index
            addr_t attr_index = (scr & SCROLL_MASK_NAME_SEL) | 0x3c0 |
                                ((scr >> 2) & 0x7) | ((scr >> 4) & 0x38);
            int attr_shift_size = ((scr >> 4) & 0x4) | (scr & 0x2);

            // read attr table
            uint8_t attr = memory::vram_read(attr_index);
            attr = (attr >> attr_shift_size) & 0x3;
            palette = palette_file + attr * PALETTE_SIZE;
          }

          // render BG block
          uint8_t bg_color = palette_file[0];
          for (int x = x0; x < x1; x++) {
            uint32_t palette_index = chr & 0x3;
            chr >>= 2;
            if (palette_index == 0) {
              line_buff[x] = bg_color;
            } else {
              uint8_t col = OPAQUE_FLAG;
              if (!skip_render) {
                col |= palette[palette_index];
              }
              line_buff[x] = col;
            }
          }
        } else {
          // blank background
          uint8_t bg_color = palette_file[0];
          memset(&line_buff[x0], bg_color, x1 - x0);
        }
      }
    }

    // update scroll counter
    // see: https://www.nesdev.org/wiki/PPU_scrolling
    if (reg.mask.bg_enable || reg.mask.sprite_enable) {
#if SHAPONES_MUTEX_FAST
      Exclusive lock(LOCK_PPU);
#endif
      uint_fast16_t scr = scroll;
      int fy = focus_y;
      int fx = fine_x;
      if (fy < SCREEN_HEIGHT) {
        if (x0 < SCREEN_WIDTH) {
          // step scroll counter for x-axis
          fx += (x1 - x0);
          while (fx >= TILE_SIZE) {
            fx -= TILE_SIZE;
            // if coarse_x < 31
            if ((scr & SCROLL_MASK_COARSE_X) < SCROLL_MASK_COARSE_X) {
              scr++;  // coarse_x++
            } else {
              // right edge of name table
              scr &= ~SCROLL_MASK_COARSE_X;  // coarse_x = 0
              scr ^= 0x0400u;                // switch name table horizontally
            }
          }
        } else if (SCREEN_WIDTH < x1 && x0 <= SCREEN_WIDTH) {
          // step scroll counter for y-axis
          // if fine_y < 7
          if ((scr & SCROLL_MASK_FINE_Y) < SCROLL_MASK_FINE_Y) {
            scr += 0x1000u;  // fine_y++
          } else {
            // bottom edge of tile
            scr &= ~SCROLL_MASK_FINE_Y;  // fine_y = 0
            // if coarse_y == 29
            if ((scr & SCROLL_MASK_COARSE_Y) == ((NUM_TILE_Y - 1) << 5)) {
              // bottom edge of name table
              scr &= ~SCROLL_MASK_COARSE_Y;  // coarse_y = 0
              scr ^= 0x0800u;                // switch name table vertically
            }
            // else if coarse_y == 31
            else if ((scr & SCROLL_MASK_COARSE_Y) == SCROLL_MASK_COARSE_Y) {
              scr &= ~SCROLL_MASK_COARSE_Y;  // coarse_y = 0
            } else {
              scr += NUM_TILE_X;  // coarse_y++
            }
          }

          // horizontal recovery
          constexpr uint16_t copy_mask = 0x041fu;
          scr &= ~copy_mask;
          scr |= reg.scroll & copy_mask;
          fx = reg.fine_x;
        }
        fine_x = fx;
      } else if (fy == SCAN_LINES - 1) {
        if (280 <= x1 && x0 <= 304) {
          // vertical recovery
          constexpr uint16_t copy_mask = 0x7be0u;
          scr &= ~copy_mask;
          scr |= reg.scroll & copy_mask;
        }
      }
      scroll = scr;
    }  // if

    x0_block = x1;
  }  // while
}

static void enum_visible_sprites(bool skip_render) {
  int n = MAX_SPRITE_COUNT;
  if (skip_render) {
    // When skip_render is set, only sprite #0 is processed for IRQ.
    n = 1;
  }

  num_visible_sprites = 0;
  uint8_t h = reg.control.sprite_size ? 16 : 8;
  for (int i = 0; i < n; i++) {
    const auto &s = oam[i];

    // vertical hit test
    int src_y = focus_y - (s.y + SPRITE_Y_OFFSET);
    if (0 <= src_y && src_y < h) {
      int tile_index = 0;
      uint_fast16_t chr = 0xFFFF;

      if (!skip_render) {
        if (s.attr & OAM_ATTR_INVERT_V) {
          // vertical inversion
          if (reg.control.sprite_size) {
            src_y ^= 0xf;
          } else {
            src_y ^= 0x7;
          }
        }

        // tile index calculation
        if (reg.control.sprite_size) {
          // 8x16 sprite
          if (src_y < 8) {
            tile_index = s.tile & 0xfe;
          } else {
            tile_index = s.tile | 0x01;
          }

          if (s.tile & 0x1) {
            tile_index += 0x1000 / 16;
          }
        } else {
          // 8x8 sprite
          tile_index = s.tile;
          if (reg.control.sprite_name_sel) {
            tile_index += 0x1000 / 16;
          }
        }
#if SHAPONES_ENABLE_CHROM_CACHE
        // read CHRROM
        int chrrom_index = (tile_index << 3) + (src_y & 0x7);
        chr =
            mapper::chrrom_read_cache(chrrom_index, s.attr & OAM_ATTR_INVERT_H);
#else
        // read CHRROM
        int chrrom_index = (tile_index << 4) + (src_y & 0x7);
        chr = mapper::chrrom_read_double(chrrom_index,
                                         s.attr & OAM_ATTR_INVERT_H);
#endif
      }

      // store sprite information
      SpriteLine &sl = sprite_lines[num_visible_sprites++];
      sl.chr = chr;
      sl.x = s.x;
      sl.palette_offset = (4 + (s.attr & OAM_ATTR_PALETTE)) * PALETTE_SIZE;
      sl.attr = 0;
      if (s.attr & OAM_ATTR_PRIORITY) sl.attr |= SL_ATTR_BEHIND;
      if (i == 0) sl.attr |= SL_ATTR_ZERO;

      if (num_visible_sprites >= MAX_VISIBLE_SPRITES) {
        break;
      }
    }
  }
}

static void render_sprite(uint8_t *line_buff, int x0_block, int x1_block,
                          bool skip_render) {
  for (int i = 0; i < num_visible_sprites; i++) {
    const auto &sl = sprite_lines[i];
    int x0 = (x0_block > sl.x) ? x0_block : sl.x;
    int x1 = (x1_block < sl.x + TILE_SIZE) ? x1_block : (sl.x + TILE_SIZE);
    uint_fast16_t chr = sl.chr >> (2 * (x0 - sl.x));
    uint_fast8_t attr = sl.attr;
    uint8_t *palette = palette_file + sl.palette_offset;
    for (int x = x0; x < x1; x++) {
      uint_fast16_t palette_index = chr & 0x3;
      chr >>= 2;
      bool sprite_opaque = (palette_index != 0);
      uint_fast8_t col = line_buff[x];
      bool bg_opaque = (col & OPAQUE_FLAG) != 0;
      if (!skip_render && !(col & BEHIND_FLAG)) {
        bool sprite_front = !bg_opaque || !(attr & SL_ATTR_BEHIND);
        if (sprite_opaque) {
          if (sprite_front) {
            col = palette[palette_index];
          }
          col |= BEHIND_FLAG;
          line_buff[x] = col;
        }
      }
      if ((attr & SL_ATTR_ZERO) && sprite_opaque && bg_opaque) {
        reg.status.sprite0_hit = 1;
      }
    }
  }
}

void mmc3_irq_set_enable(bool enable) {
  bool enable_old = mmc3_irq_enable;
  if (enable && !enable_old) {
  } else if (!enable && enable_old) {
    interrupt::deassert_irq(interrupt::Source::MMC3);
  }
  mmc3_irq_enable = enable;
}

void mmc3_irq_set_reload() { mmc3_irq_reloading = true; }

void mmc3_irq_set_latch(uint8_t data) { mmc3_irq_latch = data; }

}  // namespace nes::ppu
