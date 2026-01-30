#include "shapones/ppu.hpp"
#include "shapones/cpu.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/menu.hpp"
#include "shapones/state.hpp"

namespace nes::ppu {

static constexpr uint32_t STATE_HEADER_SIZE = registers_t::STATE_SIZE + 32;

static registers_t reg;

volatile cycle_t cycle_count;

static uint16_t focus_x;
static uint16_t focus_y;

static volatile uint16_t scroll_counter;
static uint8_t fine_x_counter;

static uint8_t bus_read_data_latest = 0;
static uint8_t bus_read_data_delayed = 0;

static bool scroll_ppuaddr_high_stored = false;

static bool nmi_level = false;

static uint8_t palette_file[PALETTE_NUM_BANK * PALETTE_SIZE];

static uint8_t oam[OAM_SIZE];
static sprite_line_t sprite_lines[MAX_VISIBLE_SPRITES];
static int num_visible_sprites;

static uint8_t bus_read(addr_t addr);
static void bus_write(addr_t addr, uint8_t data);
static uint8_t oam_read(addr_t addr);
static void oam_write(addr_t addr, uint8_t data);
static uint8_t palette_read(addr_t addr);
static void palette_write(addr_t addr, uint8_t data);

static void render_bg(uint8_t *line_buff, int x0, int x1, bool skip_render);
static void enum_visible_sprites(bool skip_render);
static void render_sprite(uint8_t *line_buff, int x0, int x1, bool skip_render);

result_t init() {
  deinit();
  return result_t::SUCCESS;
}
void deinit() {}

result_t reset() {
  // https://www.nesdev.org/wiki/PPU_power_up_state
  memset(&reg, 0, sizeof(registers_t));
  memset(oam, 0, sizeof(oam));

  bus_read_data_latest = 0;
  bus_read_data_delayed = 0;

  focus_x = 0;
  focus_y = 0;

  num_visible_sprites = 0;

  scroll_ppuaddr_high_stored = false;

  nmi_level = false;

  cycle_count = 0;

  return result_t::SUCCESS;
}

bool is_in_hblank() { return focus_x >= SCREEN_WIDTH; }

int current_focus_y() { return focus_y; }

uint8_t reg_read(addr_t addr) {
  uint8_t retval;
  switch (addr) {
    case REG_PPUSTATUS: {
      LockBlock lock(LOCK_REGS_PPU);
      retval = reg.status.raw;
      reg.status.vblank_flag = 0;
      scroll_ppuaddr_high_stored = false;
    } break;

    case REG_OAMDATA: retval = oam_read(reg.oam_addr); break;

    case REG_PPUDATA: {
      LockBlock lock(LOCK_REGS_PPU);
      addr_t addr = scroll_counter & SCROLL_MASK_PPU_ADDR;
      bus_read(addr);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scroll_counter &= ~SCROLL_MASK_PPU_ADDR;
      scroll_counter |= addr;
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
      LockBlock lock(LOCK_REGS_PPU);
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
      LockBlock lock(LOCK_REGS_PPU);
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
      LockBlock lock(LOCK_REGS_PPU);
      if (!scroll_ppuaddr_high_stored) {
        reg.scroll &= 0x00ffu;
        reg.scroll |= ((uint16_t)data << 8) & 0x3f00u;
        scroll_ppuaddr_high_stored = true;
      } else {
        reg.scroll &= 0xff00u;
        reg.scroll |= (uint16_t)data;
        scroll_counter = reg.scroll;
        scroll_ppuaddr_high_stored = false;
      }
    } break;

    case REG_PPUDATA: {
      LockBlock lock(LOCK_REGS_PPU);
      uint_fast16_t scr = scroll_counter;
      addr_t addr = scr & SCROLL_MASK_PPU_ADDR;
      bus_write(addr, data);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scr &= ~SCROLL_MASK_PPU_ADDR;
      scr |= addr;
      scroll_counter = scr;
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
  if (memory::CHRROM_BASE <= addr &&
      addr < memory::CHRROM_BASE + CHRROM_RANGE) {
    bus_read_data_delayed = bus_read_data_latest;
    bus_read_data_latest = memory::chrrom_read(addr - memory::CHRROM_BASE);
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
  if (memory::CHRROM_BASE <= addr &&
      addr < memory::CHRROM_BASE + CHRROM_RANGE) {
    memory::chrram_write(addr - memory::CHRROM_BASE, data);
  } else if (VRAM_BASE <= addr && addr < VRAM_BASE + VRAM_SIZE) {
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
  return oam[addr % OAM_SIZE];
}

static SHAPONES_INLINE void oam_write(addr_t addr, uint8_t data) {
  oam[addr % OAM_SIZE] = data;
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

result_t service(uint8_t *line_buff, bool skip_render, status_t *status) {
  if (!lock_try_get(LOCK_STATE_PPU)) {
    status->timing = timing_t::NONE;
    status->focus_y = focus_y;
    return result_t::SUCCESS;
  }

  timing_t timing = timing_t::NONE;
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
        LockBlock lock(LOCK_REGS_PPU);
        reg.status.vblank_flag = 1;
      } else if (focus_y == SCAN_LINES - 1) {
        // clear flags
        LockBlock lock(LOCK_REGS_PPU);
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
    uint_fast16_t step_count;
    uint_fast16_t dist_to_end;
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

    uint_fast16_t next_focus_x = focus_x + step_count;

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

    if (focus_y < SCREEN_HEIGHT) {
      if (focus_x <= SCREEN_WIDTH && SCREEN_WIDTH < next_focus_x) {
        mapper::instance->hblank(reg, focus_y);
      }
    } else {
      if (focus_x == 0) {
        mapper::instance->vblank(reg);
      }
    }

    // step focus
    focus_x = next_focus_x;
    if (focus_x >= LINE_CYCLES) {
      focus_x -= LINE_CYCLES;
      focus_y++;
      if (focus_y >= SCAN_LINES) {
        focus_y = 0;
        timing |= timing_t::END_OF_FRAME;
      }
    }

    // step cycle counter
    cycle_count += step_count;

    if (focus_y < SCREEN_HEIGHT) {
      if (focus_x == SCREEN_WIDTH) {
        timing |= timing_t::END_OF_VISIBLE_LINE;
        if (focus_y == SCREEN_HEIGHT - 1) {
          timing |= timing_t::END_OF_VISIBLE_AREA;
        }
      }
    } else {
      if (focus_x == 0) {
        timing |= timing_t::START_OF_VBLANK_LINE;
      }
    }

    if (timing != timing_t::NONE || irq) {
      break;
    }
  }

  if (!!(timing & timing_t::END_OF_VISIBLE_LINE)) {
    nes::state::auto_screenshot(focus_y, line_buff, skip_render);
    if (!skip_render) {
      nes::menu::overlay(focus_y, line_buff);
    }
  }

  if (status) {
    status->focus_y = focus_y;
    status->timing = timing;
  }

  lock_release(LOCK_STATE_PPU);

  return result_t::SUCCESS;
}

static void render_bg(uint8_t *line_buff, int x0_block, int x1_block,
                      bool skip_render) {
#if !SHAPONES_LOCK_FAST
  LockBlock lock(LOCK_REGS_PPU);
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
#if SHAPONES_LOCK_FAST
        Exclusive lock(LOCK_REGS_PPU);
#endif
        scr = scroll_counter;
      }

      // determine step count
      x0 = x0_block;
      if (visible_area && bg_enabled) {
        x1 = x0 + (BLOCK_SIZE - ((scr & 0x1) * TILE_SIZE + fine_x_counter));
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
            // read CHRROM
            uint32_t chrrom_index0 = (name0 << 4) + fine_y;
            uint32_t chrrom_index1 = (name1 << 4) + fine_y;
            chrrom_index0 += bg_offset;
            chrrom_index1 += bg_offset;
            uint_fast16_t chr0 =
                memory::chrrom_read_double(chrrom_index0, false);
            uint_fast16_t chr1 =
                memory::chrrom_read_double(chrrom_index1, false);
            chr = ((uint32_t)chr1 << 16) | (uint32_t)chr0;

            // adjust CHR bit pos
            int chr_shift_size = ((scr << 4) & 0x10) | (fine_x_counter << 1);
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
#if SHAPONES_LOCK_FAST
      Exclusive lock(LOCK_REGS_PPU);
#endif
      uint_fast16_t scr = scroll_counter;
      uint_fast8_t fx = fine_x_counter;
      if (focus_y < SCREEN_HEIGHT) {
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
          constexpr uint16_t MASK = 0x041fu;
          scr &= ~MASK;
          scr |= reg.scroll & MASK;
          fx = reg.fine_x;
        }
        fine_x_counter = fx;
      } else if (focus_y == SCAN_LINES - 1) {
        if (280 <= x1 && x0 <= 304) {
          // vertical recovery
          constexpr uint16_t copy_mask = 0x7be0u;
          scr &= ~copy_mask;
          scr |= reg.scroll & copy_mask;
        }
      }
      scroll_counter = scr;
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
    uint_fast8_t s_y = oam[i * 4 + OAM_ENTRY_OFFSET_Y];
    uint_fast8_t s_tile = oam[i * 4 + OAM_ENTRY_OFFSET_TILE];
    uint_fast8_t s_attr = oam[i * 4 + OAM_ENTRY_OFFSET_ATTR];
    uint_fast8_t s_x = oam[i * 4 + OAM_ENTRY_OFFSET_X];

    const auto &s = oam[i];

    // vertical hit test
    int src_y = focus_y - (s_y + SPRITE_Y_OFFSET);
    if (0 <= src_y && src_y < h) {
      int tile_index = 0;
      uint_fast16_t chr = 0xFFFF;

      if (!skip_render) {
        if (s_attr & OAM_ATTR_INVERT_V) {
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
            tile_index = s_tile & 0xfe;
          } else {
            tile_index = s_tile | 0x01;
          }

          if (s_tile & 0x1) {
            tile_index += 0x1000 / 16;
          }
        } else {
          // 8x8 sprite
          tile_index = s_tile;
          if (reg.control.sprite_name_sel) {
            tile_index += 0x1000 / 16;
          }
        }
        // read CHRROM
        int chrrom_index = (tile_index << 4) + (src_y & 0x7);
        chr = memory::chrrom_read_double(chrrom_index,
                                         s_attr & OAM_ATTR_INVERT_H);
      }

      // store sprite information
      sprite_line_t &sl = sprite_lines[num_visible_sprites++];
      sl.chr = chr;
      sl.x = s_x;
      sl.palette_offset = (4 + (s_attr & OAM_ATTR_PALETTE)) * PALETTE_SIZE;
      sl.attr = 0;
      if (s_attr & OAM_ATTR_PRIORITY) sl.attr |= SL_ATTR_BEHIND;
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

uint32_t get_state_size() {
  return STATE_HEADER_SIZE + sizeof(palette_file) + sizeof(oam);
}

result_t save_state(void *file_handle) {
  uint8_t buff[STATE_HEADER_SIZE];
  memset(buff, 0, sizeof(buff));
  uint8_t *p = buff;
  reg.store(p);
  p += registers_t::STATE_SIZE;
  BufferWriter writer(p);
  writer.u64(cycle_count);
  writer.u16(focus_x);
  writer.u16(focus_y);
  writer.u16(scroll_counter);
  writer.u8(fine_x_counter);
  writer.u8(bus_read_data_latest);
  writer.u8(bus_read_data_delayed);
  writer.b(scroll_ppuaddr_high_stored);
  writer.b(nmi_level);
  SHAPONES_TRY(fs_write(file_handle, buff, sizeof(buff)));
  SHAPONES_TRY(fs_write(file_handle, palette_file, sizeof(palette_file)));
  SHAPONES_TRY(fs_write(file_handle, oam, sizeof(oam)));
  return result_t::SUCCESS;
}

result_t load_state(void *file_handle) {
  uint8_t buff[STATE_HEADER_SIZE];
  SHAPONES_TRY(fs_read(file_handle, buff, sizeof(buff)));
  const uint8_t *p = buff;
  reg.load(p);
  p += registers_t::STATE_SIZE;
  BufferReader reader(p);
  cycle_count = reader.u64();
  focus_x = reader.u16();
  focus_y = reader.u16();
  scroll_counter = reader.u16();
  fine_x_counter = reader.u8();
  bus_read_data_latest = reader.u8();
  bus_read_data_delayed = reader.u8();
  scroll_ppuaddr_high_stored = reader.b();
  nmi_level = reader.b();
  SHAPONES_TRY(fs_read(file_handle, palette_file, sizeof(palette_file)));
  SHAPONES_TRY(fs_read(file_handle, oam, sizeof(oam)));
  return result_t::SUCCESS;
}

}  // namespace nes::ppu
