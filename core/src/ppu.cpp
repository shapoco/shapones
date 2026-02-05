#include "shapones/ppu.hpp"
#include "shapones/cpu.hpp"
#include "shapones/fifo.hpp"
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

volatile cycle_t cycle_count = 0;

static uint16_t focus_x = 0;
static uint16_t focus_y = 0;

static uint16_t scroll_counter = 0;
static uint8_t fine_x_counter = 0;

static uint8_t bus_read_data_latest = 0;
static uint8_t bus_read_data_delayed = 0;

static bool scroll_ppuaddr_high_stored = false;

static bool nmi_level = false;

static uint8_t palette_file[PALETTE_NUM_BANK * PALETTE_SIZE];

static uint32_t oam[MAX_SPRITE_COUNT];
static sprite_line_t sprite_lines[MAX_VISIBLE_SPRITES];
static int num_visible_sprites;

static uint64_t perf_last_time_us = 0;
static uint32_t perf_num_rendered_frames = 0;
static uint32_t perf_num_frames = 0;
static float perf_fps = 0.0f;
#if SHAPONES_PERF_DETAIL
static uint32_t perf_accum_t_sprite_enum = 0;
static uint32_t perf_accum_t_bg_render = 0;
static uint32_t perf_accum_t_sprite_render = 0;
static uint32_t perf_t_sprite_enum = 0;
static uint32_t perf_t_bg_render = 0;
static uint32_t perf_t_sprite_render = 0;
#endif

static AsyncFifo<reg_write_t, 6> write_queue;
static volatile bool ppu_status_read = false;

static void flush_write_queue();

static uint8_t bus_read(addr_t addr);
static void bus_write(addr_t addr, uint8_t data);
static uint8_t oam_read(addr_t addr);
static void oam_write(addr_t addr, uint8_t data);
static uint8_t palette_read(addr_t addr);
static void palette_write(addr_t addr, uint8_t data);

static void render_bg(uint8_t *line_buff, bool skip_render);
static void enum_visible_sprites(bool skip_render);
static void render_sprites(uint8_t *line_buff, bool skip_render);

static void measure(bool skip_render);

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

  write_queue.clear();

  return result_t::SUCCESS;
}

int current_focus_y() { return focus_y; }

uint8_t reg_read(addr_t addr) {
  if (!write_queue.is_empty()) {
    SemaphoreBlock block(SEMAPHORE_PPU);
    flush_write_queue();
  }

  uint8_t retval;
  switch (addr) {
    case REG_PPUSTATUS: {
      retval = reg.status.raw;
      scroll_ppuaddr_high_stored = false;
      ppu_status_read = true;
    } break;

    case REG_OAMDATA: retval = oam_read(reg.oam_addr); break;

    case REG_PPUDATA: {
      uint_fast16_t scr = scroll_counter;
      addr_t addr = scr & SCROLL_MASK_PPU_ADDR;
      bus_read(addr);
      addr += reg.control.incr_stride ? 32 : 1;
      addr &= SCROLL_MASK_PPU_ADDR;
      scr &= ~SCROLL_MASK_PPU_ADDR;
      scr |= addr;
      scroll_counter = scr;
      retval = bus_read_data_delayed;
    } break;

    default: retval = 0; break;
  }
  return retval;
}

void reg_write(addr_t addr, uint8_t data) {
  reg_write_t req;
  req.addr = addr;
  req.data = data;
  if (!write_queue.try_push(req)) {
    SemaphoreBlock block(SEMAPHORE_PPU);
    flush_write_queue();
    write_queue.push_blocking(req);
  }
}

static void flush_write_queue() {
  reg_write_t req;

  if (ppu_status_read) {
    ppu_status_read = false;
    reg.status.vblank_flag = 0;
  }

  while (write_queue.try_peek(&req)) {
    switch (req.addr) {
      case REG_PPUCTRL:
        // name sel bits
        reg.scroll &= 0xf3ff;
        reg.scroll |= (uint16_t)(req.data & 0x3) << 10;
        // other bits
        reg.control.raw = req.data;
        break;

      case REG_PPUMASK: reg.mask.raw = req.data; break;

      case REG_OAMADDR: reg.oam_addr = req.data; break;

      case REG_OAMDATA: oam_write(reg.oam_addr, req.data); break;

      case REG_PPUSCROLL:
        if (!scroll_ppuaddr_high_stored) {
          reg.scroll &= ~SCROLL_MASK_COARSE_X;
          reg.scroll |= (req.data >> 3) & SCROLL_MASK_COARSE_X;
          reg.fine_x = req.data & 0x7;
          scroll_ppuaddr_high_stored = true;
        } else {
          reg.scroll &= ~(SCROLL_MASK_COARSE_Y | SCROLL_MASK_FINE_Y);
          reg.scroll |= ((uint16_t)req.data << 2) & SCROLL_MASK_COARSE_Y;
          reg.scroll |= ((uint16_t)req.data << 12) & SCROLL_MASK_FINE_Y;
          scroll_ppuaddr_high_stored = false;
        }
        break;

      case REG_PPUADDR:
        if (!scroll_ppuaddr_high_stored) {
          reg.scroll &= 0x00ffu;
          reg.scroll |= ((uint16_t)req.data << 8) & 0x3f00u;
          scroll_ppuaddr_high_stored = true;
        } else {
          reg.scroll &= 0xff00u;
          reg.scroll |= (uint16_t)req.data;
          scroll_counter = reg.scroll;
          scroll_ppuaddr_high_stored = false;
        }
        break;

      case REG_PPUDATA: {
        uint_fast16_t scr = scroll_counter;
        addr_t addr = scr & SCROLL_MASK_PPU_ADDR;
        bus_write(addr, req.data);
        addr += reg.control.incr_stride ? 32 : 1;
        addr &= SCROLL_MASK_PPU_ADDR;
        scr &= ~SCROLL_MASK_PPU_ADDR;
        scr |= addr;
        scroll_counter = scr;
      } break;
    }

    write_queue.try_pop(&req);
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
  }
}

static SHAPONES_INLINE uint8_t oam_read(addr_t addr) {
  uint32_t index = addr / 4;
  uint32_t byte = addr % 4;
  return (oam[index] >> (byte * 8)) & 0xff;
}

static SHAPONES_INLINE void oam_write(addr_t addr, uint8_t data) {
  uint32_t index = addr / 4;
  uint32_t byte = addr % 4;
  uint32_t word = oam[index];
  word &= ~(0xff << (byte * 8));
  word |= ((uint32_t)data << (byte * 8));
  oam[index] = word;
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
  status->timing = timing_t::NONE;
  status->focus_y = focus_y;

  cycle_t cycle = cycle_count;
  cycle_t cycle_diff = cpu::ppu_cycle_leading() - cycle;
  if (cycle_diff & 0x80000000) {
    return result_t::SUCCESS;
  }

  if (!semaphore_try_take(SEMAPHORE_PPU)) {
    return result_t::SUCCESS;
  }

  flush_write_queue();

  bool visible_area = false;
  bool hblank = false;
  bool vblank = false;
  if (focus_y >= SCREEN_HEIGHT) {
    vblank = true;
  } else if (focus_x >= SCREEN_WIDTH) {
    hblank = true;
  } else {
    visible_area = true;
  }

  if (visible_area) {
    cycle_count = cycle + SCREEN_WIDTH;

    if (reg.mask.sprite_enable) {
      // enumerate visible sprites in current line
      enum_visible_sprites(skip_render);
    }

    // render background
    render_bg(line_buff, skip_render);

    if (reg.mask.sprite_enable) {
      // render sprites
      render_sprites(line_buff, skip_render);
    }

    if (!skip_render) {
      uint32_t *ptr = (uint32_t *)line_buff;
      for (uint32_t x = 0; x < SCREEN_WIDTH / 4; x++) {
        *(ptr++) &= 0x3F3F3F3F;
      }
    }

    nes::state::hsync(focus_y, line_buff, skip_render);
    if (!skip_render) {
      nes::menu::overlay(focus_y, line_buff);
    }

    status->timing |= timing_t::END_OF_VISIBLE_LINE;
    if (focus_y == SCREEN_HEIGHT - 1) {
      status->timing |= timing_t::END_OF_VISIBLE_AREA;
    }

    focus_x = SCREEN_WIDTH;
  } else if (hblank) {
    cycle_count = cycle + (LINE_CYCLES - SCREEN_WIDTH);
    mapper::instance->hblank(reg, focus_y);
    focus_x = 0;
    focus_y++;
  } else /*if (vblank)*/ {
    cycle_count = cycle + LINE_CYCLES;

    if (focus_y == SCREEN_HEIGHT) {
      reg.status.vblank_flag = 1;
      mapper::instance->vblank(reg);
      status->timing |= timing_t::START_OF_VBLANK_LINE;
    } else if (focus_y == SCAN_LINES - 2) {
      reg.status.vblank_flag = 0;
      reg.status.sprite0_hit = 0;
    } else if (focus_y == SCAN_LINES - 1) {
      if (reg.mask.bg_enable) {
        constexpr uint16_t COPY_MASK = 0x7be0u;
        scroll_counter &= ~COPY_MASK;
        scroll_counter |= reg.scroll & COPY_MASK;
      }
    }

    focus_x = 0;
    focus_y++;
    if (focus_y >= SCAN_LINES) {
      status->timing |= timing_t::END_OF_FRAME;
      measure(skip_render);
      focus_y = 0;
    }
  }

  bool nmi_level_new = reg.status.vblank_flag && reg.control.vblank_nmi_enable;
  if (nmi_level_new && !nmi_level) {
    interrupt::assert_nmi();
  }
  nmi_level = nmi_level_new;

  semaphore_give(SEMAPHORE_PPU);

  return result_t::SUCCESS;
}

static void render_bg(uint8_t *line_buff, bool skip_render) {
  if (!reg.mask.bg_enable) {
    uint8_t bg_color = palette_file[0];
    memset(line_buff, bg_color, SCREEN_WIDTH);
  }
  if (!reg.mask.bg_enable && !reg.mask.sprite_enable) {
    return;
  }

  uint_fast16_t scr = scroll_counter;
  int fine_x = reg.fine_x;

  uint32_t bg_offset = (uint32_t)reg.control.bg_name_sel << 12;

  constexpr int NUM_BLOCKS = SCREEN_WIDTH / BLOCK_SIZE + 1;
  for (int i_block = 0; i_block < NUM_BLOCKS; i_block++) {
    uint32_t chr = 0xFFFFFFFF;
    const uint8_t *palette = nullptr;

    if (!skip_render) {
      // read name table for two tiles
      addr_t name_addr0 = scr & 0xffeu;
      addr_t name_addr1 = name_addr0 + 1;
      uint32_t name0 = memory::vram_read(name_addr0);
      uint32_t name1 = memory::vram_read(name_addr1);

      // read CHRROM
      uint32_t fine_y = (scr & SCROLL_MASK_FINE_Y) >> 12;
      uint32_t chrrom_index0 = (name0 << 4) + fine_y;
      uint32_t chrrom_index1 = (name1 << 4) + fine_y;
      chrrom_index0 += bg_offset;
      chrrom_index1 += bg_offset;
      uint_fast16_t chr0 = memory::chrrom_read_double(chrrom_index0, false);
      uint_fast16_t chr1 = memory::chrrom_read_double(chrrom_index1, false);
      chr = ((uint32_t)chr1 << 16) | (uint32_t)chr0;

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
    int x_offset = i_block * BLOCK_SIZE - (((scr & 1) << 3) | fine_x);
    for (int x_local = 0; x_local < BLOCK_SIZE; x_local++) {
      uint32_t palette_index = chr & 0x3;
      chr >>= 2;
      // todo: remove branch
      uint8_t col = 0;
      if (palette_index == 0) {
        col = bg_color;
      } else {
        col = OPAQUE_FLAG;
        if (!skip_render) {
          col |= palette[palette_index];
        }
      }
      int x = x_offset + x_local;
      if (0 <= x && x < SCREEN_WIDTH) {
        line_buff[x] = col;
      }
    }

    for (int i = 0; i < 2; i++) {
      // step scroll counter for two tiles
      // step scroll counter for x-axis
      // if coarse_x < 31
      if ((scr & SCROLL_MASK_COARSE_X) < SCROLL_MASK_COARSE_X) {
        scr++;  // coarse_x++
      } else {
        // right edge of name table
        scr &= ~SCROLL_MASK_COARSE_X;  // coarse_x = 0
        scr ^= 0x0400u;                // switch name table horizontally
      }
    }
  }

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
  {
    constexpr uint16_t COPY_MASK = 0x041fu;
    scr &= ~COPY_MASK;
    scr |= reg.scroll & COPY_MASK;
  }

  scroll_counter = scr;
}

static void enum_visible_sprites(bool skip_render) {
  int n = MAX_SPRITE_COUNT;
  if (skip_render) {
    // When skip_render is set, only sprite #0 is processed for IRQ.
    n = 1;
  }

  num_visible_sprites = 0;
  bool sprite_size_16 = reg.control.sprite_size;
  uint8_t h = reg.control.sprite_size ? 16 : 8;
  for (int i = 0; i < n; i++) {
    uint32_t word = oam[i];
    uint_fast8_t s_y = (word >> (OAM_ENTRY_OFFSET_Y * 8)) & 0xff;
    uint_fast8_t s_tile = (word >> (OAM_ENTRY_OFFSET_TILE * 8)) & 0xff;
    uint_fast8_t s_attr = (word >> (OAM_ENTRY_OFFSET_ATTR * 8)) & 0xff;
    uint_fast8_t s_x = (word >> (OAM_ENTRY_OFFSET_X * 8)) & 0xff;

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

static void render_sprites(uint8_t *line_buff, bool skip_render) {
  for (int i = 0; i < num_visible_sprites; i++) {
    const auto &sl = sprite_lines[i];
    int x0 = sl.x;
    int x1 =
        (SCREEN_WIDTH < sl.x + TILE_SIZE) ? SCREEN_WIDTH : (sl.x + TILE_SIZE);
    uint_fast16_t chr = sl.chr;
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

static void measure(bool skip_render) {
  perf_num_frames++;
  if (!skip_render) {
    perf_num_rendered_frames++;
  }

  if (perf_num_frames >= 60 * 3) {
    uint64_t now_us = get_time_us();
    uint32_t t_elapsed = now_us - perf_last_time_us;
    perf_fps = (float)(perf_num_rendered_frames * 1000000) / t_elapsed;

#if SHAPONES_PERF_DETAIL
    uint32_t n = perf_num_rendered_frames;
    if (n == 0) n = 1;
    perf_t_sprite_enum = perf_accum_t_sprite_enum / n;
    perf_t_bg_render = perf_accum_t_bg_render / n;
    perf_t_sprite_render = perf_accum_t_sprite_render / n;
    SHAPONES_PRINTF("%5.2ffps, SprEnum:%uus, BgRndr:%uus, SprRndr:%uus\n",
                    perf_fps, perf_t_sprite_enum, perf_t_bg_render,
                    perf_t_sprite_render);
    perf_accum_t_sprite_enum = 0;
    perf_accum_t_bg_render = 0;
    perf_accum_t_sprite_render = 0;
#else
    SHAPONES_PRINTF("%5.2ffps\n", perf_fps);
#endif

    perf_last_time_us = now_us;
    perf_num_frames = 0;
    perf_num_rendered_frames = 0;
  }
}

result_t save_state(void *file_handle) {
  flush_write_queue();

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
  SHAPONES_TRY(fsys::write(file_handle, buff, sizeof(buff)));

  SHAPONES_TRY(fsys::write(file_handle, palette_file, sizeof(palette_file)));

  uint8_t oam_buff[sizeof(oam)];
  for (size_t i = 0; i < sizeof(oam); i += 4) {
    uint32_t word = oam[i / 4];
    oam_buff[i + 0] = (word >> 0) & 0xff;
    oam_buff[i + 1] = (word >> 8) & 0xff;
    oam_buff[i + 2] = (word >> 16) & 0xff;
    oam_buff[i + 3] = (word >> 24) & 0xff;
  }
  SHAPONES_TRY(fsys::write(file_handle, oam_buff, sizeof(oam)));

  return result_t::SUCCESS;
}

result_t load_state(void *file_handle) {
  write_queue.clear();

  uint8_t buff[STATE_HEADER_SIZE];
  SHAPONES_TRY(fsys::read(file_handle, buff, sizeof(buff)));
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
  SHAPONES_TRY(fsys::read(file_handle, palette_file, sizeof(palette_file)));

  uint8_t oam_buff[sizeof(oam)];
  SHAPONES_TRY(fsys::read(file_handle, oam_buff, sizeof(oam)));
  for (size_t i = 0; i < sizeof(oam); i += 4) {
    uint32_t word = 0;
    word |= ((uint32_t)oam_buff[i + 0]) << 0;
    word |= ((uint32_t)oam_buff[i + 1]) << 8;
    word |= ((uint32_t)oam_buff[i + 2]) << 16;
    word |= ((uint32_t)oam_buff[i + 3]) << 24;
    oam[i / 4] = word;
  }

  return result_t::SUCCESS;
}

}  // namespace nes::ppu
