#include "shapones/cpu.hpp"
#include "shapones/apu.hpp"
#include "shapones/common.hpp"
#include "shapones/dma.hpp"
#include "shapones/input.hpp"
#include "shapones/interrupt.hpp"
#include "shapones/mapper.hpp"
#include "shapones/memory.hpp"
#include "shapones/ppu.hpp"

#pragma GCC optimize("Ofast")

namespace nes::cpu {

static Registers reg;
static bool stopped = false;
#if SHAPONES_IRQ_PENDING_SUPPORT
static int irq_pending = 0;
#else
static constexpr int irq_pending = 0;
#endif
volatile cycle_t ppu_cycle_count;

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
  } else if (apu::REG_PULSE1_REG0 <= addr && addr <= apu::REG_DMC_REG3 ||
             addr == apu::REG_STATUS) {
    apu::reg_write(addr, data);
  } else if (addr == OAM_DMA_REG) {
    dma::start(data);
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

void reset() {
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
  stopped = false;
  ppu_cycle_count = 0;
  SHAPONES_PRINTF("Entry point: 0x%x\n", (int)reg.PC);
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
  addr_t addr = baseAddr + reg.Y;
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
  // SHAPONES_PRINTF("RTI to PC=0x%04x, SP=0x%02x\n", (unsigned int)reg.PC,
  // (unsigned int)reg.SP);
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

void service() {
  constexpr int MAX_BATCH_SIZE = 32;

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
    } else if (dma::is_running()) {
      // DMA is running
      cycle += dma::exec_next_cycle();
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
      // SHAPONES_PRINTF("NMI PC=0x%04x, SP=0x%02x, interrupt=%d\n", (unsigned
      // int)reg.PC, (unsigned int)reg.SP, (int)reg.status.interrupt);
      cycle += 7;  // ?
    } else if (irq_pending == 0 && !!interrupt::get_irq() &&
               !reg.status.interrupt) {
      // IRQ
      auto irq_source = interrupt::get_irq();
      auto s = reg.status;
      s.breakmode = 0;
      push(reg.PC >> 8);
      push(reg.PC & 0xff);
      push(s.raw);
      reg.status.interrupt = true;
      reg.PC = bus_read_w(VEC_IRQ);
      // SHAPONES_PRINTF("IRQ 0x%02x, PC=0x%04x, SP=0x%02x\n", (unsigned
      // int)interrupt::get_irq(), (unsigned int)reg.PC, (unsigned int)reg.SP);
      cycle += 7;  // ?
    } else {
      uint8_t op_code = fetch();

      // clang-format off
      switch(op_code) {

      case 0x00: opBRK();                                     cycle += 7; break;
      case 0x20: opJSR(fetch_abs());                          cycle += 6; break;
      case 0x40: opRTI();                                     cycle += 6; break;
      case 0x60: opRTS();                                     cycle += 6; break;
      case 0x4c: opJMP(fetch_abs());                          cycle += 3; break;
      case 0x6c: opJMP(fetch_ind_abs());                      cycle += 5; break;

      case 0x24: opBIT(fetch_zpg());                          cycle += 3; break;
      case 0x2c: opBIT(fetch_abs());                          cycle += 4; break;

      case 0x08: opPHP();                                     cycle += 3; break;
      case 0x28: opPLP();                                     cycle += 4; break;
      case 0x48: opPHA();                                     cycle += 3; break;
      case 0x68: opPLA();                                     cycle += 4; break;

      case 0x10: opBPL(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x30: opBMI(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x50: opBVC(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x70: opBVS(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0x90: opBCC(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0xb0: opBCS(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0xd0: opBNE(fetch_rel(&cycle), &cycle);            cycle += 2; break;
      case 0xf0: opBEQ(fetch_rel(&cycle), &cycle);            cycle += 2; break;

      case 0x18: opCLC();                                     cycle += 2; break;
      case 0x38: opSEC();                                     cycle += 2; break;
      case 0x58: opCLI();                                     cycle += 2; break;
      case 0x78: opSEI();                                     cycle += 2; break;
      case 0xb8: opCLV();                                     cycle += 2; break;
      case 0xd8: opCLD();                                     cycle += 2; break;
      case 0xf8: opSED();                                     cycle += 2; break;

      case 0x8a: opTXA();                                     cycle += 2; break;
      case 0x98: opTYA();                                     cycle += 2; break;
      case 0x9a: opTXS();                                     cycle += 2; break;
      case 0xa8: opTAY();                                     cycle += 2; break;
      case 0xaa: opTAX();                                     cycle += 2; break;
      case 0xba: opTSX();                                     cycle += 2; break;

      case 0x81: opSTA(fetch_pre_idx_ind(&cycle));            cycle += 6; break;
      case 0x85: opSTA(fetch_zpg());                          cycle += 3; break;
      case 0x8d: opSTA(fetch_abs());                          cycle += 4; break;
      case 0x91: opSTA(fetch_post_idx_ind(&cycle));           cycle += 6; break;
      case 0x95: opSTA(fetch_zpg_x());                        cycle += 4; break;
      case 0x99: opSTA(fetch_abs_y(&cycle));                  cycle += 4; break;
      case 0x9d: opSTA(fetch_abs_x(&cycle));                  cycle += 4; break;

      case 0x86: opSTX(fetch_zpg());                          cycle += 3; break;
      case 0x8e: opSTX(fetch_abs());                          cycle += 4; break;
      case 0x96: opSTX(fetch_zpg_y());                        cycle += 4; break;

      case 0x84: opSTY(fetch_zpg());                          cycle += 3; break;
      case 0x8c: opSTY(fetch_abs());                          cycle += 4; break;
      case 0x94: opSTY(fetch_zpg_x());                        cycle += 4; break;
  
      case 0xa1: opLDA(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xa5: opLDA(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xa9: opLDA(fetch_imm());                          cycle += 2; break;
      case 0xad: opLDA(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb1: opLDA(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xb5: opLDA(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xb9: opLDA(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0xbd: opLDA(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xa2: opLDX(fetch_imm());                          cycle += 2; break;
      case 0xa6: opLDX(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xae: opLDX(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb6: opLDX(bus_read(fetch_zpg_y()));              cycle += 4; break;
      case 0xbe: opLDX(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;

      case 0xa0: opLDY(fetch_imm());                          cycle += 2; break;
      case 0xa4: opLDY(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xac: opLDY(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb4: opLDY(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xbc: opLDY(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xc1: opCMP(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xc5: opCMP(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xc9: opCMP(fetch_imm());                          cycle += 2; break;
      case 0xcd: opCMP(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xd1: opCMP(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xd5: opCMP(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xd9: opCMP(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0xdd: opCMP(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xe0: opCPX(fetch_imm());                          cycle += 2; break;
      case 0xe4: opCPX(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xec: opCPX(bus_read(fetch_abs()));                cycle += 4; break;

      case 0xc0: opCPY(fetch_imm());                          cycle += 2; break;
      case 0xc4: opCPY(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xcc: opCPY(bus_read(fetch_abs()));                cycle += 4; break;

      case 0xca: opDEX();                                     cycle += 2; break;
      case 0x88: opDEY();                                     cycle += 2; break;
      
      case 0xe8: opINX();                                     cycle += 2; break;
      case 0xc8: opINY();                                     cycle += 2; break;

      case 0xc6: opDEC(fetch_zpg());                          cycle += 5; break;
      case 0xce: opDEC(fetch_abs());                          cycle += 6; break;
      case 0xd6: opDEC(fetch_zpg_x());                        cycle += 6; break;
      case 0xde: opDEC(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0xe6: opINC(fetch_zpg());                          cycle += 5; break;
      case 0xee: opINC(fetch_abs());                          cycle += 6; break;
      case 0xf6: opINC(fetch_zpg_x());                        cycle += 6; break;
      case 0xfe: opINC(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x01: opORA(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x05: opORA(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x09: opORA(fetch_imm());                          cycle += 2; break;
      case 0x0d: opORA(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x11: opORA(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x15: opORA(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x19: opORA(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x1d: opORA(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x21: opAND(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x25: opAND(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x29: opAND(fetch_imm());                          cycle += 2; break;
      case 0x2d: opAND(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x31: opAND(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x35: opAND(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x39: opAND(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x3d: opAND(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x41: opEOR(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x45: opEOR(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x49: opEOR(fetch_imm());                          cycle += 2; break;
      case 0x4d: opEOR(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x51: opEOR(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x55: opEOR(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x59: opEOR(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x5d: opEOR(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x61: opADC(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0x65: opADC(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0x69: opADC(fetch_imm());                          cycle += 2; break;
      case 0x6d: opADC(bus_read(fetch_abs()));                cycle += 4; break;
      case 0x71: opADC(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0x75: opADC(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0x79: opADC(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0x7d: opADC(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0xe1: opSBC(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xe5: opSBC(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xe9: opSBC(fetch_imm());                          cycle += 2; break;
      case 0xed: opSBC(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xf1: opSBC(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xf5: opSBC(bus_read(fetch_zpg_x()));              cycle += 4; break;
      case 0xf9: opSBC(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;
      case 0xfd: opSBC(bus_read(fetch_abs_x(&cycle)));        cycle += 4; break;

      case 0x06: opASL_m(fetch_zpg());                        cycle += 5; break;
      case 0x0a: opASL_a();                                   cycle += 2; break;
      case 0x0e: opASL_m(fetch_abs());                        cycle += 6; break;
      case 0x16: opASL_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x1e: opASL_m(fetch_abs_x(&cycle));                cycle += 6; break;

      case 0x26: opROL_m(fetch_zpg());                        cycle += 5; break;
      case 0x2a: opROL_a();                                   cycle += 2; break;
      case 0x2e: opROL_m(fetch_abs());                        cycle += 6; break;
      case 0x36: opROL_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x3e: opROL_m(fetch_abs_x(&cycle));                cycle += 6; break;

      case 0x46: opLSR_m(fetch_zpg());                        cycle += 5; break;
      case 0x4a: opLSR_a();                                   cycle += 2; break;
      case 0x4e: opLSR_m(fetch_abs());                        cycle += 6; break;
      case 0x56: opLSR_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x5e: opLSR_m(fetch_abs_x(&cycle));                cycle += 6; break;

      case 0x66: opROR_m(fetch_zpg());                        cycle += 5; break;
      case 0x6a: opROR_a();                                   cycle += 2; break;
      case 0x6e: opROR_m(fetch_abs());                        cycle += 6; break;
      case 0x76: opROR_m(fetch_zpg_x());                      cycle += 6; break;
      case 0x7e: opROR_m(fetch_abs_x(&cycle));                cycle += 6; break;
      
      case 0xea: opNOP();                                     cycle += 2; break;
#if 0
      // unofficial opcodes
      case 0x03: opSLO(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x07: opSLO(fetch_zpg());                          cycle += 5; break;
      case 0x0f: opSLO(fetch_abs());                          cycle += 6; break;
      case 0x13: opSLO(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x17: opSLO(fetch_zpg_x());                        cycle += 6; break;
      case 0x1b: opSLO(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x1f: opSLO(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x23: opRLA(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x27: opRLA(fetch_zpg());                          cycle += 5; break;
      case 0x2f: opRLA(fetch_abs());                          cycle += 6; break;
      case 0x33: opRLA(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x37: opRLA(fetch_zpg_x());                        cycle += 6; break;
      case 0x3b: opRLA(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x3f: opRLA(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x43: opSRE(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x47: opSRE(fetch_zpg());                          cycle += 5; break;
      case 0x4f: opSRE(fetch_abs());                          cycle += 6; break;
      case 0x53: opSRE(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x57: opSRE(fetch_zpg_x());                        cycle += 6; break;
      case 0x5b: opSRE(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x5f: opSRE(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x63: opRRA(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0x67: opRRA(fetch_zpg());                          cycle += 5; break;
      case 0x6f: opRRA(fetch_abs());                          cycle += 6; break;
      case 0x73: opRRA(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0x77: opRRA(fetch_zpg_x());                        cycle += 6; break;
      case 0x7b: opRRA(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0x7f: opRRA(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0x83: opSAX(fetch_pre_idx_ind(&cycle));            cycle += 6; break;
      case 0x87: opSAX(fetch_zpg());                          cycle += 3; break;
      case 0x8f: opSAX(fetch_abs());                          cycle += 4; break;
      case 0x97: opSAX(fetch_zpg_y());                        cycle += 4; break;

      case 0xa3: opLAX(bus_read(fetch_pre_idx_ind(&cycle)));  cycle += 6; break;
      case 0xa7: opLAX(bus_read(fetch_zpg()));                cycle += 3; break;
      case 0xab: opLAX(fetch_imm());                          cycle += 2; break;
      case 0xaf: opLAX(bus_read(fetch_abs()));                cycle += 4; break;
      case 0xb3: opLAX(bus_read(fetch_post_idx_ind(&cycle))); cycle += 5; break;
      case 0xb7: opLAX(bus_read(fetch_zpg_y()));              cycle += 4; break;
      case 0xbf: opLAX(bus_read(fetch_abs_y(&cycle)));        cycle += 4; break;

      case 0xc3: opDCP(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0xc7: opDCP(fetch_zpg());                          cycle += 5; break;
      case 0xcf: opDCP(fetch_abs());                          cycle += 6; break;
      case 0xd3: opDCP(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0xd7: opDCP(fetch_zpg_x());                        cycle += 6; break;
      case 0xdb: opDCP(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0xdf: opDCP(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0xe3: opISB(fetch_pre_idx_ind(&cycle));            cycle += 8; break;
      case 0xe7: opISB(fetch_zpg());                          cycle += 5; break;
      case 0xef: opISB(fetch_abs());                          cycle += 6; break;
      case 0xf3: opISB(fetch_post_idx_ind(&cycle));           cycle += 8; break;
      case 0xf7: opISB(fetch_zpg_x());                        cycle += 6; break;
      case 0xfb: opISB(fetch_abs_y(&cycle));                  cycle += 7; break;
      case 0xff: opISB(fetch_abs_x(&cycle));                  cycle += 7; break;

      case 0xeb: opSBC(fetch_imm());                          cycle += 2; break;

      case 0x1a: opNOP();                                     cycle += 2; break;
      case 0x3a: opNOP();                                     cycle += 2; break;
      case 0x5a: opNOP();                                     cycle += 2; break;
      case 0x7a: opNOP();                                     cycle += 2; break;
      case 0xda: opNOP();                                     cycle += 2; break;
      case 0xfa: opNOP();                                     cycle += 2; break;

      case 0x02: opNOP();                                     cycle += 2; break; // STP
      case 0x12: opNOP();                                     cycle += 2; break; // STP
      case 0x22: opNOP();                                     cycle += 2; break; // STP
      case 0x32: opNOP();                                     cycle += 2; break; // STP
      case 0x42: opNOP();                                     cycle += 2; break; // STP
      case 0x52: opNOP();                                     cycle += 2; break; // STP
      case 0x62: opNOP();                                     cycle += 2; break; // STP
      case 0x72: opNOP();                                     cycle += 2; break; // STP
      case 0x92: opNOP();                                     cycle += 2; break; // STP
      case 0xb2: opNOP();                                     cycle += 2; break; // STP
      case 0xd2: opNOP();                                     cycle += 2; break; // STP
      case 0xf2: opNOP();                                     cycle += 2; break; // STP

      case 0x9c: fetch_abs_x(&cycle); opNOP();                cycle += 5; break; // SHY
      case 0x9e: fetch_abs_y(&cycle); opNOP();                cycle += 5; break; // SHX
      case 0x0b: fetch_imm(); opNOP();                        cycle += 2; break; // ANC
      case 0x2b: fetch_imm(); opNOP();                        cycle += 2; break; // ANC
      case 0x4b: fetch_imm(); opNOP();                        cycle += 2; break; // ALR
      case 0x6b: fetch_imm(); opNOP();                        cycle += 2; break; // ARR
      case 0x8b: fetch_imm(); opNOP();                        cycle += 2; break; // XAA
      case 0xcb: fetch_imm(); opNOP();                        cycle += 2; break; // AXS

      case 0x93: fetch_post_idx_ind(&cycle); opNOP();         cycle += 6; break; // AHX
      case 0x9f: fetch_abs_y(&cycle); opNOP();                cycle += 5; break; // AHX

      case 0x9b: fetch_abs_y(&cycle); opNOP();                cycle += 5; break; // TAS
      case 0xbb: fetch_abs_y(&cycle); opNOP();                cycle += 4; break; // LAS

      case 0x80: fetch_imm(); opNOP();                        cycle += 2; break; 
      case 0x82: fetch_imm(); opNOP();                        cycle += 2; break; 
      case 0x89: fetch_imm(); opNOP();                        cycle += 2; break; 
      case 0xc2: fetch_imm(); opNOP();                        cycle += 2; break;
      case 0xe2: fetch_imm(); opNOP();                        cycle += 3; break;

      case 0x04: fetch_zpg(); opNOP();                        cycle += 3; break; 
      case 0x44: fetch_zpg(); opNOP();                        cycle += 3; break; 
      case 0x64: fetch_zpg(); opNOP();                        cycle += 3; break;

      case 0x0c: fetch_abs(); opNOP();                        cycle += 4; break;

      case 0x14: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0x34: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0x54: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0x74: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0xd4: fetch_zpg_x(); opNOP();                      cycle += 4; break;
      case 0xf4: fetch_zpg_x(); opNOP();                      cycle += 4; break;

      case 0x1c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0x3c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0x5c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0x7c: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0xdc: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
      case 0xfc: fetch_abs_x(&cycle); opNOP();                cycle += 4; break;
#else
      default:
          SHAPONES_ERRORF("UNKNOWN INSTRUCTION: 0x%02x (PC=0x%04x)\n", (int)op_code, (int)reg.PC);
          break;
#endif
      }
      // clang-format on
    }  // if

#if SHAPONES_IRQ_PENDING_SUPPORT
    if (irq_pending > 0) {
      irq_pending--;
    }
#endif
    ppu_cycle_count += cycle * 3;

  }  // while
}

}  // namespace nes::cpu
