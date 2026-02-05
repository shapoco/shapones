#include "shapones/cpu.hpp"
#include "shapones/apu.hpp"
#include "shapones/common.hpp"
#include "shapones/host_intf.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/lock.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/menu.hpp"
#include "shapones/ppu.hpp"
#include "shapones/state.hpp"

namespace nes::cpu {

static constexpr int MAX_BATCH_SIZE = 32;

static registers_t reg;
static bool stopped = false;
#if SHAPONES_IRQ_PENDING_SUPPORT
static uint8_t irq_pending = 0;
#else
static constexpr uint8_t irq_pending = 0;
#endif
volatile cycle_t ppu_cycle_count;

static addr_t dma_addr = 0;
static uint16_t dma_cycle = DMA_TRANSFER_SIZE;

static SHAPONES_INLINE bool dma_is_running() {
  return dma_cycle < DMA_TRANSFER_SIZE;
}

static SHAPONES_INLINE void dma_start(uint8_t src_page) {
  dma_cycle = 0;
  dma_addr = (addr_t)src_page * 0x100;
}

static SHAPONES_INLINE uint_fast8_t dma_service() {
  if (dma_cycle >= DMA_TRANSFER_SIZE) return 0;
  ppu::oam_dma_write(dma_cycle++, cpu::bus_read(dma_addr++));
  return 2;
}

uint8_t bus_read(addr_t addr) {
  uint8_t retval;
  if (memory::PRGROM_BASE <= addr &&
      addr < memory::PRGROM_BASE + PRGROM_RANGE) {
    retval = memory::prgrom_read(addr - memory::PRGROM_BASE);
  } else if (WRAM_BASE <= addr && addr < WRAM_BASE + WRAM_SIZE) {
    retval = memory::wram[addr - WRAM_BASE];
  } else if (memory::PRGRAM_BASE <= addr &&
             addr < memory::PRGRAM_BASE + PRGRAM_RANGE) {
    retval = memory::prgram_read(addr - memory::PRGRAM_BASE);
  } else if (PPUREG_BASE <= addr && addr < PPUREG_BASE + ppu::REG_SIZE) {
    retval = ppu::reg_read(addr);
  } else if (apu::REG_PULSE1_REG0 <= addr && addr <= apu::REG_DMC_REG3 ||
             addr == apu::REG_STATUS) {
    retval = apu::reg_read(addr);
  } else if (INPUT_REG_0 <= addr && addr <= INPUT_REG_1) {
    retval = input::read_latched(addr - INPUT_REG_0);
  } else if (WRAM_MIRROR_BASE <= addr && addr < WRAM_MIRROR_BASE + WRAM_SIZE) {
    retval = memory::wram[addr - WRAM_MIRROR_BASE];
  } else {
    retval = 0;
  }
  return retval;
}

void bus_write(addr_t addr, uint8_t data) {
  if (WRAM_BASE <= addr && addr < WRAM_BASE + WRAM_SIZE) {
    memory::wram[addr - WRAM_BASE] = data;
  } else if (memory::PRGRAM_BASE <= addr &&
             addr < memory::PRGRAM_BASE + PRGRAM_RANGE) {
    memory::prgram_write(addr - memory::PRGRAM_BASE, data);
  } else if (PPUREG_BASE <= addr && addr < PPUREG_BASE + ppu::REG_SIZE) {
    ppu::reg_write(addr, data);
  } else if (addr == OAM_DMA_REG) {
    dma_start(data);
  } else if (apu::REG_PULSE1_REG0 <= addr && addr <= apu::REG_DMC_REG3 ||
             addr == apu::REG_STATUS) {
    apu::reg_write(addr, data);
  } else if (addr == INPUT_REG_0) {
    input::write_control(data);
  } else if (WRAM_MIRROR_BASE <= addr && addr < WRAM_MIRROR_BASE + WRAM_SIZE) {
    memory::wram[addr - WRAM_MIRROR_BASE] = data;
  } else {
    mapper::instance->write(addr, data);
  }
}

static SHAPONES_INLINE uint16_t bus_read_w(addr_t addr) {
  return (uint16_t)bus_read(addr) | ((uint16_t)bus_read(addr + 1) << 8);
}

result_t init() {
  deinit();
  return result_t::SUCCESS;
}

void deinit() {}

result_t reset() {
  reg.A = 0;
  reg.X = 0;
  reg.Y = 0;
  reg.status.negative = 0;
  reg.status.overflow = 0;
  reg.status.reserved = 1;
  reg.status.breakmode = 0;
  reg.status.decimalmode = 0;
  reg.status.interrupt = 1;
  reg.status.zero = 0;
  reg.status.carry = 0;
  reg.SP = 0xfd;
  reg.PC = bus_read_w(VEC_RESET) | 0x8000;
  dma_cycle = DMA_TRANSFER_SIZE;
  dma_addr = 0;
  ppu_cycle_count = 0;
  stopped = false;
  SHAPONES_PRINTF("Entry point: 0x%x\n", (int)reg.PC);
  return result_t::SUCCESS;
}

void stop() {
  stopped = true;
  SHAPONES_PRINTF("CPU stopped.\n");
  SHAPONES_PRINTF("  PC: 0x%02x\n", (int)reg.PC);
  SHAPONES_PRINTF("  A : 0x%02x\n", (int)reg.A);
  SHAPONES_PRINTF("  X : 0x%02x\n", (int)reg.X);
  SHAPONES_PRINTF("  Y : 0x%02x\n", (int)reg.Y);
  SHAPONES_PRINTF("  SP: 0x%02x\n", (int)reg.SP);
  SHAPONES_PRINTF("  STATUS: 0x%02x\n", (int)reg.status.raw);
  SHAPONES_PRINTF("    carry:       %d\n", (int)reg.status.carry);
  SHAPONES_PRINTF("    zero:        %d\n", (int)reg.status.zero);
  SHAPONES_PRINTF("    interrupt:   %d\n", (int)reg.status.interrupt);
  SHAPONES_PRINTF("    decimalmode: %d\n", (int)reg.status.decimalmode);
  SHAPONES_PRINTF("    breakmode:   %d\n", (int)reg.status.breakmode);
  SHAPONES_PRINTF("    reserved:    %d\n", (int)reg.status.reserved);
  SHAPONES_PRINTF("    overflow:    %d\n", (int)reg.status.overflow);
  SHAPONES_PRINTF("    negative:    %d\n", (int)reg.status.negative);
}

bool is_stopped() { return stopped; }

static SHAPONES_INLINE uint8_t fetch() {
  uint8_t retval = bus_read(reg.PC);
  reg.PC += 1;
  if (reg.PC == 0) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0x0000\n");
  }
  return retval;
}

static SHAPONES_INLINE uint16_t fetch_w() {
  uint16_t retval = bus_read_w(reg.PC);
  reg.PC += 2;
  if (reg.PC == 0 || reg.PC == 1) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0x0000\n");
  }
  return retval;
}

static SHAPONES_INLINE uint8_t set_nz(uint8_t value) {
  reg.status.negative = (value >> 7) & 1;
  reg.status.zero = (value == 0) ? 1 : 0;
  return value;
}

static SHAPONES_INLINE void push(uint8_t value) {
  if (reg.SP == 0) {
    SHAPONES_ERRORF("Stack Overflow at push()\n");
  }
  bus_write(0x100 | reg.SP--, value);
}

static SHAPONES_INLINE uint8_t pop() {
  if (reg.SP >= 255) {
    SHAPONES_ERRORF("Stack Underflow at pop()\n");
  }
  return bus_read(0x100 | ++reg.SP);
}

static SHAPONES_INLINE addr_t fetch_zpg() { return fetch(); }
static SHAPONES_INLINE addr_t fetch_zpg_x() { return (fetch() + reg.X) & 0xff; }
static SHAPONES_INLINE addr_t fetch_zpg_y() { return (fetch() + reg.Y) & 0xff; }

static SHAPONES_INLINE addr_t fetch_imm() { return fetch(); }

static SHAPONES_INLINE addr_t fetch_pre_idx_ind(cycle_t *cycle) {
  addr_t base = (fetch() + reg.X) & 0xff;
  addr_t addr = bus_read(base) | ((uint16_t)bus_read((base + 1) & 0xffu) << 8);
  if ((addr & 0xff00u) != (base & 0xff00u)) *cycle += 1;
  return addr;
}

static SHAPONES_INLINE addr_t fetch_post_idx_ind(cycle_t *cycle) {
  addr_t addrOrData = fetch();
  addr_t baseAddr = bus_read(addrOrData) |
                    ((uint16_t)bus_read((addrOrData + 1) & 0xffu) << 8);
  addr_t addr = (baseAddr + reg.Y) & 0xffff;
  if ((addr & 0xff00u) != (baseAddr & 0xff00u)) *cycle += 1;
  return addr;
}

static SHAPONES_INLINE addr_t fetch_abs() { return fetch_w(); }

static SHAPONES_INLINE addr_t fetch_abs_x(cycle_t *cycle) {
  uint16_t base = fetch_w();
  uint16_t retval = base + reg.X;
  if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
  return retval;
}

static SHAPONES_INLINE addr_t fetch_abs_y(cycle_t *cycle) {
  uint16_t base = fetch_w();
  uint16_t retval = base + reg.Y;
  if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
  return retval;
}

static SHAPONES_INLINE addr_t fetch_ind_abs() {
  addr_t addr_or_data = fetch_w();
  addr_t next_addr =
      (addr_or_data & 0xFF00) | (((addr_or_data & 0xFF) + 1) & 0xFF);
  addr_t addr = bus_read(addr_or_data) | ((uint16_t)bus_read(next_addr) << 8);
  return addr;
}

static SHAPONES_INLINE addr_t fetch_rel(cycle_t *cycle) {
  int distance = fetch();
  if (distance >= 0x80) distance -= 256;
  addr_t retval = reg.PC + distance;
  if ((retval & 0xff00u) != (reg.PC & 0xff00u)) *cycle += 1;
  return retval;
}

static SHAPONES_INLINE void opBRK() {
  fetch();  // padding
  push(reg.PC >> 8);
  push(reg.PC & 0xff);
  auto s = reg.status;
  s.breakmode = true;
  push(s.raw);
  reg.status.interrupt = true;
  reg.PC = bus_read_w(VEC_IRQ);
}

static SHAPONES_INLINE void opJMP(addr_t addr) { reg.PC = addr; }

static SHAPONES_INLINE void opJSR(addr_t addr) {
  addr_t pc = reg.PC - 1;
  if (pc == 0xFFFF) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0xFFFF after JSR\n");
  }
  push(pc >> 8);
  push(pc & 0xff);
  reg.PC = addr;
}

static SHAPONES_INLINE void opRTI() {
  reg.status.raw = pop();
  reg.status.reserved = 1;
  reg.status.breakmode = 0;
  reg.PC = (addr_t)pop();
  reg.PC |= ((addr_t)pop() << 8);
}

static SHAPONES_INLINE void opRTS() {
  reg.PC = (addr_t)pop();
  reg.PC |= ((addr_t)pop() << 8);
  reg.PC += 1;
  if (reg.PC == 0) {
    SHAPONES_PRINTF("*Warning: PC wrapped around to 0x0000 after RTS\n");
  }
}

static SHAPONES_INLINE void opBIT(addr_t addr) {
  uint8_t data = bus_read(addr);
  reg.status.negative = (data >> 7) & 1;
  reg.status.overflow = (data >> 6) & 1;
  reg.status.zero = (reg.A & data) ? 0 : 1;
}

static SHAPONES_INLINE void opPHP() {
  auto s = reg.status;
  s.breakmode = 1;
  push(s.raw);
}

static SHAPONES_INLINE void opPLP() {
  reg.status.raw = pop();
  reg.status.reserved = 1;
  reg.status.breakmode = 0;
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = 2;
#endif
}

static SHAPONES_INLINE void opPHA() { push(reg.A); }

static SHAPONES_INLINE void opPLA() { reg.A = set_nz(pop()); }

static SHAPONES_INLINE void cond_jump(bool cond, addr_t addr, cycle_t *cycle) {
  if (cond) {
    reg.PC = addr;
    (*cycle)++;
  }
}

static SHAPONES_INLINE void opBPL(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.negative, addr, cycle);
}
static SHAPONES_INLINE void opBMI(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.negative, addr, cycle);
}
static SHAPONES_INLINE void opBVC(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.overflow, addr, cycle);
}
static SHAPONES_INLINE void opBVS(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.overflow, addr, cycle);
}
static SHAPONES_INLINE void opBCC(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.carry, addr, cycle);
}
static SHAPONES_INLINE void opBCS(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.carry, addr, cycle);
}
static SHAPONES_INLINE void opBNE(addr_t addr, cycle_t *cycle) {
  cond_jump(!reg.status.zero, addr, cycle);
}
static SHAPONES_INLINE void opBEQ(addr_t addr, cycle_t *cycle) {
  cond_jump(reg.status.zero, addr, cycle);
}

static SHAPONES_INLINE void opCLC() { reg.status.carry = 0; }
static SHAPONES_INLINE void opSEC() { reg.status.carry = 1; }
static SHAPONES_INLINE void opCLI() {
  reg.status.interrupt = 0;
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = 2;
#endif
}
static SHAPONES_INLINE void opSEI() {
  reg.status.interrupt = 1;
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = 2;
#endif
}
static SHAPONES_INLINE void opCLV() { reg.status.overflow = 0; }
static SHAPONES_INLINE void opCLD() { reg.status.decimalmode = 0; }
static SHAPONES_INLINE void opSED() { reg.status.decimalmode = 1; }

static SHAPONES_INLINE void opTXA() { reg.A = set_nz(reg.X); }
static SHAPONES_INLINE void opTYA() { reg.A = set_nz(reg.Y); }
static SHAPONES_INLINE void opTXS() { reg.SP = reg.X; }
static SHAPONES_INLINE void opTAY() { reg.Y = set_nz(reg.A); }
static SHAPONES_INLINE void opTAX() { reg.X = set_nz(reg.A); }
static SHAPONES_INLINE void opTSX() { reg.X = set_nz(reg.SP); }

static SHAPONES_INLINE void opLDA(uint8_t data) { reg.A = set_nz(data); }
static SHAPONES_INLINE void opLDX(uint8_t data) { reg.X = set_nz(data); }
static SHAPONES_INLINE void opLDY(uint8_t data) { reg.Y = set_nz(data); }

static SHAPONES_INLINE void opSTA(addr_t addr) { bus_write(addr, reg.A); }
static SHAPONES_INLINE void opSTX(addr_t addr) { bus_write(addr, reg.X); }
static SHAPONES_INLINE void opSTY(addr_t addr) { bus_write(addr, reg.Y); }

static SHAPONES_INLINE void compare(uint8_t a, uint8_t b) {
  int16_t compared = (int16_t)a - (int16_t)b;
  reg.status.carry = compared >= 0;
  set_nz(compared);
}
static SHAPONES_INLINE void opCMP(uint8_t data) { compare(reg.A, data); }
static SHAPONES_INLINE void opCPX(uint8_t data) { compare(reg.X, data); }
static SHAPONES_INLINE void opCPY(uint8_t data) { compare(reg.Y, data); }

static SHAPONES_INLINE void opINX() { set_nz(++reg.X); }
static SHAPONES_INLINE void opINY() { set_nz(++reg.Y); }
static SHAPONES_INLINE void opDEX() { set_nz(--reg.X); }
static SHAPONES_INLINE void opDEY() { set_nz(--reg.Y); }

static SHAPONES_INLINE void opINC(addr_t addr) {
  bus_write(addr, set_nz(bus_read(addr) + 1));
}
static SHAPONES_INLINE void opDEC(addr_t addr) {
  bus_write(addr, set_nz(bus_read(addr) - 1));
}

static SHAPONES_INLINE void opAND(uint8_t data) {
  reg.A = set_nz(data & reg.A);
}
static SHAPONES_INLINE void opORA(uint8_t data) {
  reg.A = set_nz(data | reg.A);
}
static SHAPONES_INLINE void opEOR(uint8_t data) {
  reg.A = set_nz(data ^ reg.A);
}

static SHAPONES_INLINE void opADC(uint8_t data) {
  uint_fast16_t operated = (uint_fast16_t)reg.A + data + reg.status.carry;
  reg.status.overflow =
      (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
  reg.status.carry = (operated >= 0x100) ? 1 : 0;
  reg.A = set_nz(operated);
}

static SHAPONES_INLINE void opSBC(uint8_t data) {
  int_fast16_t operated =
      (int_fast16_t)reg.A - data - (reg.status.carry ? 0 : 1);
  reg.status.overflow =
      (((reg.A ^ operated) & 0x80) != 0 && ((reg.A ^ data) & 0x80) != 0);
  reg.status.carry = (operated >= 0) ? 1 : 0;
  reg.A = set_nz(operated);
}

static SHAPONES_INLINE uint8_t opASL(uint8_t data) {
  reg.status.carry = (data >> 7) & 1;
  return set_nz(data << 1);
}
static SHAPONES_INLINE void opASL_a() { reg.A = opASL(reg.A); }
static SHAPONES_INLINE void opASL_m(addr_t addr) {
  bus_write(addr, opASL(bus_read(addr)));
}

static SHAPONES_INLINE uint8_t opLSR(uint8_t data) {
  reg.status.carry = data & 1;
  return set_nz((data >> 1) & 0x7f);
}
static SHAPONES_INLINE void opLSR_a() { reg.A = opLSR(reg.A); }
static SHAPONES_INLINE void opLSR_m(addr_t addr) {
  bus_write(addr, opLSR(bus_read(addr)));
}

static SHAPONES_INLINE uint8_t opROL(uint8_t data) {
  uint_fast8_t carry = (data >> 7) & 1;
  data = (data << 1) | reg.status.carry;
  reg.status.carry = carry;
  return set_nz(data);
}
static SHAPONES_INLINE void opROL_a() { reg.A = opROL(reg.A); }
static SHAPONES_INLINE void opROL_m(addr_t addr) {
  bus_write(addr, opROL(bus_read(addr)));
}

static SHAPONES_INLINE uint8_t opROR(uint8_t data) {
  uint_fast8_t carry = data & 1;
  data = ((data >> 1) & 0x7f) | (reg.status.carry << 7);
  reg.status.carry = carry;
  return set_nz(data);
}
static SHAPONES_INLINE void opROR_a() { reg.A = opROR(reg.A); }
static SHAPONES_INLINE void opROR_m(addr_t addr) {
  bus_write(addr, opROR(bus_read(addr)));
}

static SHAPONES_INLINE void opNOP() {}

static SHAPONES_INLINE void opSLO(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  reg.status.carry = (data & 0x80) >> 7;
  data <<= 1;
  reg.A = set_nz(reg.A | data);
  bus_write(addr, data);
}
static SHAPONES_INLINE void opRLA(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  uint_fast8_t carry = (data & 0x80) >> 7;
  data = (data << 1) | reg.status.carry;
  reg.status.carry = carry;
  reg.A = set_nz(reg.A & data);
  bus_write(addr, data);
}
static SHAPONES_INLINE void opSRE(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  reg.status.carry = data & 0x1;
  data >>= 1;
  reg.A = set_nz(reg.A ^ data);
  bus_write(addr, data);
}
static SHAPONES_INLINE void opRRA(addr_t addr) {
  uint_fast8_t data = bus_read(addr);
  uint_fast8_t carry = data & 0x1;
  data = ((data >> 1) & 0x7f);
  data |= reg.status.carry << 7;
  uint_fast16_t operated = (uint_fast16_t)data + reg.A + carry;
  reg.status.overflow =
      (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
  reg.A = set_nz(operated);
  reg.status.carry = (operated >> 8) & 1;
  bus_write(addr, data);
}

static SHAPONES_INLINE void opSAX(addr_t addr) {
  bus_write(addr, reg.A & reg.X);
}
static SHAPONES_INLINE void opLAX(uint8_t data) {
  reg.A = reg.X = set_nz(data);
}

static SHAPONES_INLINE void opDCP(addr_t addr) {
  uint_fast8_t operated = bus_read(addr) - 1;
  set_nz(reg.A - operated);
  bus_write(addr, operated);
}

static SHAPONES_INLINE void opISB(addr_t addr) {
  uint_fast8_t data = bus_read(addr) + 1;
  uint_fast16_t operated =
      (uint_fast16_t)(~data & 0xff) + reg.A + reg.status.carry;
  reg.status.overflow =
      (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
  reg.status.carry = (operated >> 8) & 1;
  reg.A = set_nz(operated);
  bus_write(addr, data);
}

result_t service() {
  menu::service();
  input::update();

  int n = MAX_BATCH_SIZE;
  while (n-- > 0) {
    cycle_t cycle = 0;

    cycle_t ppu_cycle_diff = ppu_cycle_count - ppu::cycle_following();
    if (ppu_cycle_diff >= ppu::MAX_DELAY_CYCLES) {
      break;
    }

    if (stopped) {
      cycle += 1;  // nop
    } else if (dma_is_running()) {
      // DMA is running
      cycle += dma_service();
    } else if (irq_pending == 0 && interrupt::is_nmi_asserted()) {
      // NMI
      interrupt::deassert_nmi();
      auto s = reg.status;
      s.breakmode = 0;
      push(reg.PC >> 8);
      push(reg.PC & 0xff);
      push(s.raw);
      reg.status.interrupt = true;
      reg.PC = bus_read_w(VEC_NMI);
      cycle += 7;  // ?
    } else if (irq_pending == 0 && !!interrupt::get_irq() &&
               !reg.status.interrupt) {
      // IRQ
      auto s = reg.status;
      s.breakmode = 0;
      push(reg.PC >> 8);
      push(reg.PC & 0xff);
      push(s.raw);
      reg.status.interrupt = true;
      reg.PC = bus_read_w(VEC_IRQ);
      cycle += 7;  // ?
    } else {
      uint8_t op_code = fetch();

      static const void *JUMPTABLE[] = {
      // clang-format off
          &&op00, &&op01, &&op02, &&op03, &&op04, &&op05, &&op06, &&op07, &&op08, &&op09, &&op0A, &&op0B, &&op0C, &&op0D, &&op0E, &&op0F,
          &&op10, &&op11, &&op12, &&op13, &&op14, &&op15, &&op16, &&op17, &&op18, &&op19, &&op1A, &&op1B, &&op1C, &&op1D, &&op1E, &&op1F,
          &&op20, &&op21, &&op22, &&op23, &&op24, &&op25, &&op26, &&op27, &&op28, &&op29, &&op2A, &&op2B, &&op2C, &&op2D, &&op2E, &&op2F,
          &&op30, &&op31, &&op32, &&op33, &&op34, &&op35, &&op36, &&op37, &&op38, &&op39, &&op3A, &&op3B, &&op3C, &&op3D, &&op3E, &&op3F,
          &&op40, &&op41, &&op42, &&op43, &&op44, &&op45, &&op46, &&op47, &&op48, &&op49, &&op4A, &&op4B, &&op4C, &&op4D, &&op4E, &&op4F,
          &&op50, &&op51, &&op52, &&op53, &&op54, &&op55, &&op56, &&op57, &&op58, &&op59, &&op5A, &&op5B, &&op5C, &&op5D, &&op5E, &&op5F,
          &&op60, &&op61, &&op62, &&op63, &&op64, &&op65, &&op66, &&op67, &&op68, &&op69, &&op6A, &&op6B, &&op6C, &&op6D, &&op6E, &&op6F,
          &&op70, &&op71, &&op72, &&op73, &&op74, &&op75, &&op76, &&op77, &&op78, &&op79, &&op7A, &&op7B, &&op7C, &&op7D, &&op7E, &&op7F,
          &&op80, &&op81, &&op82, &&op83, &&op84, &&op85, &&op86, &&op87, &&op88, &&op89, &&op8A, &&op8B, &&op8C, &&op8D, &&op8E, &&op8F,
          &&op90, &&op91, &&op92, &&op93, &&op94, &&op95, &&op96, &&op97, &&op98, &&op99, &&op9A, &&op9B, &&op9C, &&op9D, &&op9E, &&op9F,
          &&opA0, &&opA1, &&opA2, &&opA3, &&opA4, &&opA5, &&opA6, &&opA7, &&opA8, &&opA9, &&opAA, &&opAB, &&opAC, &&opAD, &&opAE, &&opAF,
          &&opB0, &&opB1, &&opB2, &&opB3, &&opB4, &&opB5, &&opB6, &&opB7, &&opB8, &&opB9, &&opBA, &&opBB, &&opBC, &&opBD, &&opBE, &&opBF,
          &&opC0, &&opC1, &&opC2, &&opC3, &&opC4, &&opC5, &&opC6, &&opC7, &&opC8, &&opC9, &&opCA, &&opCB, &&opCC, &&opCD, &&opCE, &&opCF,
          &&opD0, &&opD1, &&opD2, &&opD3, &&opD4, &&opD5, &&opD6, &&opD7, &&opD8, &&opD9, &&opDA, &&opDB, &&opDC, &&opDD, &&opDE, &&opDF,
          &&opE0, &&opE1, &&opE2, &&opE3, &&opE4, &&opE5, &&opE6, &&opE7, &&opE8, &&opE9, &&opEA, &&opEB, &&opEC, &&opED, &&opEE, &&opEF,
          &&opF0, &&opF1, &&opF2, &&opF3, &&opF4, &&opF5, &&opF6, &&opF7, &&opF8, &&opF9, &&opFA, &&opFB, &&opFC, &&opFD, &&opFE, &&opFF,
      // clang-format on
      };

      goto *JUMPTABLE[op_code];

      // clang-format off
      do {
      op00: opBRK();                                     cycle += 7; break;
      op20: opJSR(fetch_abs());                          cycle += 6; break;
      op40: opRTI();                                     cycle += 6; break;
      op60: opRTS();                                     cycle += 6; break;
      op4C: opJMP(fetch_abs());                          cycle += 3; break;
      op6C: opJMP(fetch_ind_abs());                      cycle += 5; break;

      op24: opBIT(fetch_zpg());                          cycle += 3; break;
      op2C: opBIT(fetch_abs());                          cycle += 4; break;

      op08: opPHP();                                     cycle += 3; break;
      op28: opPLP();                                     cycle += 4; break;
      op48: opPHA();                                     cycle += 3; break;
      op68: opPLA();                                     cycle += 4; break;

      op10: opBPL(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      op30: opBMI(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      op50: opBVC(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      op70: opBVS(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      op90: opBCC(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      opB0: opBCS(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      opD0: opBNE(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      opF0: opBEQ(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      op18: opCLC();                                     cycle += 2; break;
      op38: opSEC();                                     cycle += 2; break;
      op58: opCLI();                                     cycle += 2; break;
      op78: opSEI();                                     cycle += 2; break;
      opB8: opCLV();                                     cycle += 2; break;
      opD8: opCLD();                                     cycle += 2; break;
      opF8: opSED();                                     cycle += 2; break;
      op8A: opTXA();                                     cycle += 2; break;
      op98: opTYA();                                     cycle += 2; break;
      op9A: opTXS();                                     cycle += 2; break;
      opA8: opTAY();                                     cycle += 2; break;
      opAA: opTAX();                                     cycle += 2; break;
      opBA: opTSX();                                     cycle += 2; break;
      op81: opSTA(fetch_pre_idx_ind(&cycle));            cycle += 6; break;
      op85: opSTA(fetch_zpg());                          cycle += 3; break;
      op8D: opSTA(fetch_abs());                          cycle += 4; break;
      op91: opSTA(fetch_post_idx_ind(&cycle));           cycle += 6; break;
      op95: opSTA(fetch_zpg_x());                        cycle += 4; break;
      op99: opSTA(fetch_abs_y(&cycle));                  cycle += 4; break;
      op9D: opSTA(fetch_abs_x(&cycle));                  cycle += 4; break;
      op86: opSTX(fetch_zpg());                          cycle += 3; break;
      op8E: opSTX(fetch_abs());                          cycle += 4; break;
      op96: opSTX(fetch_zpg_y());                        cycle += 4; break;
      op84: opSTY(fetch_zpg());                          cycle += 3; break;
      op8C: opSTY(fetch_abs());                          cycle += 4; break;
      op94: opSTY(fetch_zpg_x());                        cycle += 4; break;
      opA1: opLDA(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      opA5: opLDA(bus_read(fetch_zpg()));                cycle += 3; break;
      opA9: opLDA(fetch_imm());                          cycle += 2; break;
      opAD: opLDA(bus_read(fetch_abs()));                cycle += 4; break;
      opB1: opLDA(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      opB5: opLDA(bus_read(fetch_zpg_x()));              cycle += 4; break;
      opB9: opLDA(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      opBD: opLDA(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      opA2: opLDX(fetch_imm());                          cycle += 2; break;
      opA6: opLDX(bus_read(fetch_zpg()));                cycle += 3; break;
      opAE: opLDX(bus_read(fetch_abs()));                cycle += 4; break;
      opB6: opLDX(bus_read(fetch_zpg_y()));              cycle += 4; break;
      opBE: opLDX(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      opA0: opLDY(fetch_imm());                          cycle += 2; break;
      opA4: opLDY(bus_read(fetch_zpg()));                cycle += 3; break;
      opAC: opLDY(bus_read(fetch_abs()));                cycle += 4; break;
      opB4: opLDY(bus_read(fetch_zpg_x()));              cycle += 4; break;
      opBC: opLDY(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      opC1: opCMP(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      opC5: opCMP(bus_read(fetch_zpg()));                cycle += 3; break;
      opC9: opCMP(fetch_imm());                          cycle += 2; break;
      opCD: opCMP(bus_read(fetch_abs()));                cycle += 4; break;
      opD1: opCMP(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      opD5: opCMP(bus_read(fetch_zpg_x()));              cycle += 4; break;
      opD9: opCMP(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      opDD: opCMP(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      opE0: opCPX(fetch_imm());                          cycle += 2; break;
      opE4: opCPX(bus_read(fetch_zpg()));                cycle += 3; break;
      opEC: opCPX(bus_read(fetch_abs()));                cycle += 4; break;
      opC0: opCPY(fetch_imm());                          cycle += 2; break;
      opC4: opCPY(bus_read(fetch_zpg()));                cycle += 3; break;
      opCC: opCPY(bus_read(fetch_abs()));                cycle += 4; break;
      opCA: opDEX();                                     cycle += 2; break;
      op88: opDEY();                                     cycle += 2; break;
      opE8: opINX();                                     cycle += 2; break;
      opC8: opINY();                                     cycle += 2; break;
      opC6: opDEC(fetch_zpg());                          cycle += 5; break;
      opCE: opDEC(fetch_abs());                          cycle += 6; break;
      opD6: opDEC(fetch_zpg_x());                        cycle += 6; break;
      opDE: opDEC(fetch_abs_x(&cycle));                  cycle += 7; break;
      opE6: opINC(fetch_zpg());                          cycle += 5; break;
      opEE: opINC(fetch_abs());                          cycle += 6; break;
      opF6: opINC(fetch_zpg_x());                        cycle += 6; break;
      opFE: opINC(fetch_abs_x(&cycle));                  cycle += 7; break;
      op01: opORA(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      op05: opORA(bus_read(fetch_zpg()));                cycle += 3; break;
      op09: opORA(fetch_imm());                          cycle += 2; break;
      op0D: opORA(bus_read(fetch_abs()));                cycle += 4; break;
      op11: opORA(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      op15: opORA(bus_read(fetch_zpg_x()));              cycle += 4; break;
      op19: opORA(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      op1D: opORA(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      op21: opAND(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      op25: opAND(bus_read(fetch_zpg()));                cycle += 3; break;
      op29: opAND(fetch_imm());                          cycle += 2; break;
      op2D: opAND(bus_read(fetch_abs()));                cycle += 4; break;
      op31: opAND(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      op35: opAND(bus_read(fetch_zpg_x()));              cycle += 4; break;
      op39: opAND(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      op3D: opAND(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      op41: opEOR(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      op45: opEOR(bus_read(fetch_zpg()));                cycle += 3; break;
      op49: opEOR(fetch_imm());                          cycle += 2; break;
      op4D: opEOR(bus_read(fetch_abs()));                cycle += 4; break;
      op51: opEOR(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      op55: opEOR(bus_read(fetch_zpg_x()));              cycle += 4; break;
      op59: opEOR(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      op5D: opEOR(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      op61: opADC(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      op65: opADC(bus_read(fetch_zpg()));                cycle += 3; break;
      op69: opADC(fetch_imm());                          cycle += 2; break;
      op6D: opADC(bus_read(fetch_abs()));                cycle += 4; break;
      op71: opADC(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      op75: opADC(bus_read(fetch_zpg_x()));              cycle += 4; break;
      op79: opADC(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      op7D: opADC(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      opE1: opSBC(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      opE5: opSBC(bus_read(fetch_zpg()));                cycle += 3; break;
      opE9: opSBC(fetch_imm());                          cycle += 2; break;
      opED: opSBC(bus_read(fetch_abs()));                cycle += 4; break;
      opF1: opSBC(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      opF5: opSBC(bus_read(fetch_zpg_x()));              cycle += 4; break;
      opF9: opSBC(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      opFD: opSBC(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;
      op06: opASL_m(fetch_zpg());                        cycle += 5; break;
      op0A: opASL_a();                                   cycle += 2; break;
      op0E: opASL_m(fetch_abs());                        cycle += 6; break;
      op16: opASL_m(fetch_zpg_x());                      cycle += 6; break;
      op1E: opASL_m(fetch_abs_x(&cycle));                cycle += 6; break;
      op26: opROL_m(fetch_zpg());                        cycle += 5; break;
      op2A: opROL_a();                                   cycle += 2; break;
      op2E: opROL_m(fetch_abs());                        cycle += 6; break;
      op36: opROL_m(fetch_zpg_x());                      cycle += 6; break;
      op3E: opROL_m(fetch_abs_x(&cycle));                cycle += 6; break;
      op46: opLSR_m(fetch_zpg());                        cycle += 5; break;
      op4A: opLSR_a();                                   cycle += 2; break;
      op4E: opLSR_m(fetch_abs());                        cycle += 6; break;
      op56: opLSR_m(fetch_zpg_x());                      cycle += 6; break;
      op5E: opLSR_m(fetch_abs_x(&cycle));                cycle += 6; break;
      op66: opROR_m(fetch_zpg());                        cycle += 5; break;
      op6A: opROR_a();                                   cycle += 2; break;
      op6E: opROR_m(fetch_abs());                        cycle += 6; break;
      op76: opROR_m(fetch_zpg_x());                      cycle += 6; break;
      op7E: opROR_m(fetch_abs_x(&cycle));                cycle += 6; break;
      opEA: opNOP();                                     cycle += 2; break;

      // unofficial opcodes
      op03: /* opSLO(fetch_pre_idx_ind(&cycle));            cycle += 8; */ break;
      op07: /* opSLO(fetch_zpg());                          cycle += 5; */ break;
      op0F: /* opSLO(fetch_abs());                          cycle += 6; */ break;
      op13: /* opSLO(fetch_post_idx_ind(&cycle));           cycle += 8; */ break;
      op17: /* opSLO(fetch_zpg_x());                        cycle += 6; */ break;
      op1B: /* opSLO(fetch_abs_y(&cycle));                  cycle += 7; */ break;
      op1F: /* opSLO(fetch_abs_x(&cycle));                  cycle += 7; */ break;
      op23: /* opRLA(fetch_pre_idx_ind(&cycle));            cycle += 8; */ break;
      op27: /* opRLA(fetch_zpg());                          cycle += 5; */ break;
      op2F: /* opRLA(fetch_abs());                          cycle += 6; */ break;
      op33: /* opRLA(fetch_post_idx_ind(&cycle));           cycle += 8; */ break;
      op37: /* opRLA(fetch_zpg_x());                        cycle += 6; */ break;
      op3B: /* opRLA(fetch_abs_y(&cycle));                  cycle += 7; */ break;
      op3F: /* opRLA(fetch_abs_x(&cycle));                  cycle += 7; */ break;
      op43: /* opSRE(fetch_pre_idx_ind(&cycle));            cycle += 8; */ break;
      op47: /* opSRE(fetch_zpg());                          cycle += 5; */ break;
      op4F: /* opSRE(fetch_abs());                          cycle += 6; */ break;
      op53: /* opSRE(fetch_post_idx_ind(&cycle));           cycle += 8; */ break;
      op57: /* opSRE(fetch_zpg_x());                        cycle += 6; */ break;
      op5B: /* opSRE(fetch_abs_y(&cycle));                  cycle += 7; */ break;
      op5F: /* opSRE(fetch_abs_x(&cycle));                  cycle += 7; */ break;
      op63: /* opRRA(fetch_pre_idx_ind(&cycle));            cycle += 8; */ break;
      op67: /* opRRA(fetch_zpg());                          cycle += 5; */ break;
      op6F: /* opRRA(fetch_abs());                          cycle += 6; */ break;
      op73: /* opRRA(fetch_post_idx_ind(&cycle));           cycle += 8; */ break;
      op77: /* opRRA(fetch_zpg_x());                        cycle += 6; */ break;
      op7B: /* opRRA(fetch_abs_y(&cycle));                  cycle += 7; */ break;
      op7F: /* opRRA(fetch_abs_x(&cycle));                  cycle += 7; */ break;
      op83: /* opSAX(fetch_pre_idx_ind(&cycle));            cycle += 6; */ break;
      op87: /* opSAX(fetch_zpg());                          cycle += 3; */ break;
      op8F: /* opSAX(fetch_abs());                          cycle += 4; */ break;
      op97: /* opSAX(fetch_zpg_y());                        cycle += 4; */ break;
      opA3: /* opLAX(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; */ break;
      opA7: /* opLAX(bus_read(fetch_zpg()));                cycle += 3; */ break;
      opAB: /* opLAX(fetch_imm());                          cycle += 2; */ break;
      opAF: /* opLAX(bus_read(fetch_abs()));                cycle += 4; */ break;
      opB3: /* opLAX(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; */ break;
      opB7: /* opLAX(bus_read(fetch_zpg_y()));              cycle += 4; */ break;
      opBF: /* opLAX(bus_read(fetch_abs_y(&cycle)));        cycle += 4; */ break;
      opC3: /* opDCP(fetch_pre_idx_ind(&cycle));            cycle += 8; */ break;
      opC7: /* opDCP(fetch_zpg());                          cycle += 5; */ break;
      opCF: /* opDCP(fetch_abs());                          cycle += 6; */ break;
      opD3: /* opDCP(fetch_post_idx_ind(&cycle));           cycle += 8; */ break;
      opD7: /* opDCP(fetch_zpg_x());                        cycle += 6; */ break;
      opDB: /* opDCP(fetch_abs_y(&cycle));                  cycle += 7; */ break;
      opDF: /* opDCP(fetch_abs_x(&cycle));                  cycle += 7; */ break;
      opE3: /* opISB(fetch_pre_idx_ind(&cycle));            cycle += 8; */ break;
      opE7: /* opISB(fetch_zpg());                          cycle += 5; */ break;
      opEF: /* opISB(fetch_abs());                          cycle += 6; */ break;
      opF3: /* opISB(fetch_post_idx_ind(&cycle));           cycle += 8; */ break;
      opF7: /* opISB(fetch_zpg_x());                        cycle += 6; */ break;
      opFB: /* opISB(fetch_abs_y(&cycle));                  cycle += 7; */ break;
      opFF: /* opISB(fetch_abs_x(&cycle));                  cycle += 7; */ break;
      opEB: /* opSBC(fetch_imm());                          cycle += 2; */ break;
      op1A: /* opNOP();                                     cycle += 2; */ break;
      op3A: /* opNOP();                                     cycle += 2; */ break;
      op5A: /* opNOP();                                     cycle += 2; */ break;
      op7A: /* opNOP();                                     cycle += 2; */ break;
      opDA: /* opNOP();                                     cycle += 2; */ break;
      opFA: /* opNOP();                                     cycle += 2; */ break;
      op02: /* opNOP();                                     cycle += 2; */ break; // STP
      op12: /* opNOP();                                     cycle += 2; */ break; // STP
      op22: /* opNOP();                                     cycle += 2; */ break; // STP
      op32: /* opNOP();                                     cycle += 2; */ break; // STP
      op42: /* opNOP();                                     cycle += 2; */ break; // STP
      op52: /* opNOP();                                     cycle += 2; */ break; // STP
      op62: /* opNOP();                                     cycle += 2; */ break; // STP
      op72: /* opNOP();                                     cycle += 2; */ break; // STP
      op92: /* opNOP();                                     cycle += 2; */ break; // STP
      opB2: /* opNOP();                                     cycle += 2; */ break; // STP
      opD2: /* opNOP();                                     cycle += 2; */ break; // STP
      opF2: /* opNOP();                                     cycle += 2; */ break; // STP
      op9C: /* fetch_abs_x(&cycle); opNOP();                cycle += 5; */ break; // SHY
      op9E: /* fetch_abs_y(&cycle); opNOP();                cycle += 5; */ break; // SHX
      op0B: /* fetch_imm(); opNOP();                        cycle += 2; */ break; // ANC
      op2B: /* fetch_imm(); opNOP();                        cycle += 2; */ break; // ANC
      op4B: /* fetch_imm(); opNOP();                        cycle += 2; */ break; // ALR
      op6B: /* fetch_imm(); opNOP();                        cycle += 2; */ break; // ARR
      op8B: /* fetch_imm(); opNOP();                        cycle += 2; */ break; // XAA
      opCB: /* fetch_imm(); opNOP();                        cycle += 2; */ break; // AXS
      op93: /* fetch_post_idx_ind(&cycle); opNOP();         cycle += 6; */ break; // AHX
      op9F: /* fetch_abs_y(&cycle); opNOP();                cycle += 5; */ break; // AHX
      op9B: /* fetch_abs_y(&cycle); opNOP();                cycle += 5; */ break; // TAS
      opBB: /* fetch_abs_y(&cycle); opNOP();                cycle += 4; */ break; // LAS
      op80: /* fetch_imm(); opNOP();                        cycle += 2; */ break; 
      op82: /* fetch_imm(); opNOP();                        cycle += 2; */ break; 
      op89: /* fetch_imm(); opNOP();                        cycle += 2; */ break; 
      opC2: /* fetch_imm(); opNOP();                        cycle += 2; */ break;
      opE2: /* fetch_imm(); opNOP();                        cycle += 3; */ break;
      op04: /* fetch_zpg(); opNOP();                        cycle += 3; */ break; 
      op44: /* fetch_zpg(); opNOP();                        cycle += 3; */ break; 
      op64: /* fetch_zpg(); opNOP();                        cycle += 3; */ break;
      op0C: /* fetch_abs(); opNOP();                        cycle += 4; */ break;
      op14: /* fetch_zpg_x(); opNOP();                      cycle += 4; */ break;
      op34: /* fetch_zpg_x(); opNOP();                      cycle += 4; */ break;
      op54: /* fetch_zpg_x(); opNOP();                      cycle += 4; */ break;
      op74: /* fetch_zpg_x(); opNOP();                      cycle += 4; */ break;
      opD4: /* fetch_zpg_x(); opNOP();                      cycle += 4; */ break;
      opF4: /* fetch_zpg_x(); opNOP();                      cycle += 4; */ break;
      op1C: /* fetch_abs_x(&cycle); opNOP();                cycle += 4; */ break;
      op3C: /* fetch_abs_x(&cycle); opNOP();                cycle += 4; */ break;
      op5C: /* fetch_abs_x(&cycle); opNOP();                cycle += 4; */ break;
      op7C: /* fetch_abs_x(&cycle); opNOP();                cycle += 4; */ break;
      opDC: /* fetch_abs_x(&cycle); opNOP();                cycle += 4; */ break;
      opFC: /* fetch_abs_x(&cycle); opNOP();                cycle += 4; */ break;
        // clang-format on

      opXX:
        SHAPONES_ERRORF("UNKNOWN INSTRUCTION: 0x%02x (PC=0x%04x)\n",
                        (int)op_code, (int)reg.PC);
        break;

      } while (0);
    }  // if

#if SHAPONES_IRQ_PENDING_SUPPORT
    if (irq_pending > 0) {
      irq_pending--;
    }
#endif
    ppu_cycle_count += cycle * 3;

  }  // while

  return result_t::SUCCESS;
}

uint32_t get_state_size() { return STATE_SIZE; }

result_t save_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  memset(buffer, 0, sizeof(buffer));
  uint8_t *p = buffer;
  reg.store(p);
  p += registers_t::STATE_SIZE;
  BufferWriter writer(p);
  writer.u64(ppu_cycle_count);
  writer.u16(dma_addr);
  writer.u16(dma_cycle);
  writer.b(stopped);
  writer.u8(irq_pending);
  return fsys::write(file_handle, buffer, sizeof(buffer));
}

result_t load_state(void *file_handle) {
  uint8_t buffer[STATE_SIZE];
  SHAPONES_TRY(fsys::read(file_handle, buffer, sizeof(buffer)));
  const uint8_t *p = buffer;
  reg.load(p);
  p += registers_t::STATE_SIZE;
  BufferReader reader(p);
  ppu_cycle_count = reader.u64();
  dma_addr = reader.u16();
  dma_cycle = reader.u16();
  stopped = reader.b();
#if SHAPONES_IRQ_PENDING_SUPPORT
  irq_pending = reader.u8();
#else
  reader.u8();  // discard
#endif
  return result_t::SUCCESS;
}

}  // namespace nes::cpu
