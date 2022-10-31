#include "stdint.h"
#include "shapones/shapones.hpp"

namespace nes::cpu {

static Registers reg;
static bool stopped = false;

uint8_t bus_read(addr_t addr) {
    uint8_t retval;
    if (PRGROM_BASE <= addr && addr < PRGROM_BASE + memory::PRGROM_SIZE ) {
        retval = memory::prgrom_read(addr - PRGROM_BASE);
    }
    else if (WRAM_BASE <= addr && addr < WRAM_BASE + memory::WRAM_SIZE) {
        retval = memory::wram[addr - WRAM_BASE];
    }
    else if (PPUREG_BASE <= addr && addr < PPUREG_BASE + ppu::REG_SIZE) {
        retval = ppu::reg_read(addr);
    }
    else if (INPUT_REG_0 <= addr && addr <= INPUT_REG_1) {
        retval = input::read_latched(addr - INPUT_REG_0);
    }
    else if (WRAM_MIRROR_BASE <= addr && addr < WRAM_MIRROR_BASE + memory::WRAM_SIZE) {
        retval = memory::wram[addr - WRAM_MIRROR_BASE];
    }
    else {
        retval = 0;
    }
    return retval;
}

void bus_write(addr_t addr, uint8_t data) {
    if (WRAM_BASE <= addr && addr < WRAM_BASE + memory::WRAM_SIZE) {
        memory::wram[addr - WRAM_BASE] = data;
    }
    else if (PPUREG_BASE <= addr && addr < PPUREG_BASE + ppu::REG_SIZE) {
        ppu::reg_write(addr, data);
    }
    else if (WRAM_MIRROR_BASE <= addr && addr < WRAM_MIRROR_BASE + memory::WRAM_SIZE) {
        memory::wram[addr - WRAM_MIRROR_BASE] = data;
    }
    else if (addr == OAM_DMA_REG) {
        dma::start(data);
    }
    else if (addr == INPUT_REG_0) {
        input::write_control(data);
    }
    else {
        //NES_PRINTF("*Warning: Invalid CPU bus write addr: 0x%04x\n", (int)addr);
    }
}

static inline uint16_t bus_read_w(addr_t addr) {
    return
        (uint16_t)bus_read(addr) | 
        ((uint16_t)bus_read(addr + 1) << 8);
}

void reset() {
    reg.A = 0;
    reg.X = 0;
    reg.Y = 0;
    reg.status.negative = 0;
    reg.status.overflow = 0;
    reg.status.reserved = 1;
    reg.status.breakmode = 1;
    reg.status.decimalmode = 0;
    reg.status.interrupt = 1;
    reg.status.zero = 0;
    reg.status.carry = 0;
    reg.SP = 0xfd;
    reg.PC = bus_read_w(VEC_RESET) | 0x8000;
    NES_PRINTF("entry point = 0x%x\n", (int)reg.PC);
}

void stop() {
    stopped = true;
}

static inline uint8_t fetch() { 
    uint8_t retval = bus_read(reg.PC); 
    reg.PC += 1;
    return retval;
}

static inline uint16_t fetch_w() { 
    uint16_t retval = bus_read_w(reg.PC) ;
    reg.PC += 2;
    return retval;
}

static inline uint8_t set_nz(uint8_t value) {
    reg.status.negative = (value >> 7) & 1;
    reg.status.zero = (value == 0) ? 1 : 0;
    return value;
}

static inline void push(uint8_t value) {
    if (reg.SP == 0) {
        NES_ERRORF("Stack Overflow at push()\n");
    }
    bus_write(0x100 | reg.SP--, value);
}

static inline uint8_t pop() {
    if (reg.SP >= 255) {
        NES_ERRORF("Stack Overflow at pop()\n");
    }
    return bus_read(0x100 | ++reg.SP);
}

static inline addr_t fetch_zpg() { return fetch(); }
static inline addr_t fetch_zpg_x() { return (fetch() + reg.X) & 0xff; }
static inline addr_t fetch_zpg_y() { return (fetch() + reg.Y) & 0xff; }

static inline addr_t fetch_imm() { 
    return fetch();
}

static inline addr_t fetch_pre_idx_ind(int *cycle) {
    addr_t base = (fetch() + reg.X) & 0xff;
    addr_t addr = bus_read(base) | ((uint16_t)bus_read((base + 1) & 0xffu) << 8);
    if ((addr & 0xff00u) != (base & 0xff00u)) *cycle += 1;
    return addr;
}

static inline addr_t fetch_post_idx_ind(int *cycle) {
    addr_t addrOrData = fetch();
    addr_t baseAddr = bus_read(addrOrData) | ((uint16_t)bus_read((addrOrData + 1) & 0xffu) << 8);
    addr_t addr = baseAddr + reg.Y;
    if ((addr & 0xff00u) != (baseAddr & 0xff00u)) *cycle += 1;
    return addr;
}

static inline addr_t fetch_abs() { 
    return fetch_w();
}

static inline addr_t fetch_abs_x(int *cycle) { 
    uint16_t base = fetch_w();
    uint16_t retval = base + reg.X;
    if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
    return retval;
}
            
static inline addr_t fetch_abs_y(int *cycle) { 
    uint16_t base = fetch_w();
    uint16_t retval = base + reg.Y;
    if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
    return retval;
}

static inline addr_t fetch_ind_abs() {
    addr_t addr_or_data = fetch_w();
    addr_t next_addr = (addr_or_data & 0xFF00) | (((addr_or_data & 0xFF) + 1) & 0xFF);
    addr_t addr = bus_read(addr_or_data) | ((uint16_t)bus_read(next_addr) << 8);
    return addr;
}

static inline addr_t fetch_rel(int *cycle) {    
    int distance = fetch();
    if (distance >= 0x80) distance -= 256;
    addr_t retval = reg.PC + distance;
    if ((retval & 0xff00u) != (reg.PC & 0xff00u)) *cycle += 1;
    return retval;
}

static inline void opBRK() {
    bool interrupt = reg.status.interrupt != 0;
    push(reg.PC >> 8);
    push(reg.PC & 0xff);
    reg.status.breakmode = true;
    push(reg.status.raw);
    reg.status.interrupt = true;
    if ( ! interrupt) {
        reg.PC = bus_read_w(VEC_IRQ);
    }
}

static inline void opJMP(addr_t addr) {
    reg.PC = addr;
}

static inline void opJSR(addr_t addr) {
    addr_t pc = reg.PC - 1;
    push(pc >> 8);
    push(pc & 0xff);
    reg.PC = addr;
}

static inline void opRTI() {
    reg.status.raw = pop();
    reg.status.reserved = true;
    reg.PC = (addr_t)pop();
    reg.PC |= ((addr_t)pop() << 8);
}

static inline void opRTS() {
    reg.PC = (addr_t)pop();
    reg.PC |= ((addr_t)pop() << 8);
    reg.PC += 1;
}

static inline void opBIT(addr_t addr) {
    uint8_t data = bus_read(addr);
    reg.status.negative = (data >> 7) & 1;
    reg.status.overflow = (data >> 6) & 1;
    reg.status.zero = (reg.A & data) ? 0 : 1;
}

static inline void opPHP() {
    reg.status.breakmode = 1;
    push(reg.status.raw);
}

static inline void opPLP() {
    reg.status.raw = pop();
    reg.status.reserved = 1;
}
        
static inline void opPHA() {
    push(reg.A);
}
        
static inline void opPLA() {
    reg.A = pop();
    set_nz(reg.A);
}

static inline void cond_jump(bool cond, addr_t addr, int *cycle) {
    if (cond) {
        reg.PC = addr;
        (*cycle)++;
    }
}

static inline void opBPL(addr_t addr, int *cycle) { cond_jump(!reg.status.negative, addr, cycle); }
static inline void opBMI(addr_t addr, int *cycle) { cond_jump( reg.status.negative, addr, cycle); }
static inline void opBVC(addr_t addr, int *cycle) { cond_jump(!reg.status.overflow, addr, cycle); }
static inline void opBVS(addr_t addr, int *cycle) { cond_jump( reg.status.overflow, addr, cycle); }
static inline void opBCC(addr_t addr, int *cycle) { cond_jump(!reg.status.carry, addr, cycle); }
static inline void opBCS(addr_t addr, int *cycle) { cond_jump( reg.status.carry, addr, cycle); }
static inline void opBNE(addr_t addr, int *cycle) { cond_jump(!reg.status.zero, addr, cycle); }
static inline void opBEQ(addr_t addr, int *cycle) { cond_jump( reg.status.zero, addr, cycle); }

static inline void opCLC() { reg.status.carry = 0; }
static inline void opSEC() { reg.status.carry = 1; }
static inline void opCLI() { reg.status.interrupt = 0; }
static inline void opSEI() { reg.status.interrupt = 1; }
static inline void opCLV() { reg.status.overflow = 0; }
static inline void opCLD() { reg.status.decimalmode = 0; }
static inline void opSED() { reg.status.decimalmode = 1; }
        
static inline void opTXA() { reg.A = set_nz(reg.X); }
static inline void opTYA() { reg.A = set_nz(reg.Y); }
static inline void opTXS() { reg.SP = reg.X; }
static inline void opTAY() { reg.Y = set_nz(reg.A); }
static inline void opTAX() { reg.X = set_nz(reg.A); }
static inline void opTSX() { reg.X = set_nz(reg.SP); }

static inline void opLDA(uint8_t data) { reg.A = set_nz(data); }
static inline void opLDX(uint8_t data) { reg.X = set_nz(data); }
static inline void opLDY(uint8_t data) { reg.Y = set_nz(data); }

static inline void opSTA(addr_t addr) { bus_write(addr, reg.A); }
static inline void opSTX(addr_t addr) { bus_write(addr, reg.X); }
static inline void opSTY(addr_t addr) { bus_write(addr, reg.Y); }

static inline void compare(uint8_t a, uint8_t b) {
    int16_t compared = (int16_t)a - (int16_t)b;
    reg.status.carry = compared >= 0;
    set_nz(compared);
}
static inline void opCMP(uint8_t data) { compare(reg.A, data); }
static inline void opCPX(uint8_t data) { compare(reg.X, data); }
static inline void opCPY(uint8_t data) { compare(reg.Y, data); }

static inline void opINX() { set_nz(++reg.X); }
static inline void opINY() { set_nz(++reg.Y); }
static inline void opDEX() { set_nz(--reg.X); }
static inline void opDEY() { set_nz(--reg.Y); }

static inline void opINC(addr_t addr) { bus_write(addr, set_nz(bus_read(addr) + 1)); }
static inline void opDEC(addr_t addr) { bus_write(addr, set_nz(bus_read(addr) - 1)); }

static inline void opAND(uint8_t data) { reg.A = set_nz(data & reg.A); }
static inline void opORA(uint8_t data) { reg.A = set_nz(data | reg.A); }
static inline void opEOR(uint8_t data) { reg.A = set_nz(data ^ reg.A); }

static inline void opADC(uint8_t data) {
    uint16_t operated = (uint16_t)reg.A + data + reg.status.carry;
    reg.status.overflow = (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
    reg.status.carry = (operated >= 0x100) ? 1 : 0;
    reg.A = set_nz(operated);
}

static inline void opSBC(uint8_t data) {
    int operated = (int)reg.A - data - (reg.status.carry ? 0 : 1);
    reg.status.overflow = (((reg.A ^ operated) & 0x80) != 0 && ((reg.A ^ data) & 0x80) != 0);
    reg.status.carry = (operated >= 0) ? 1 : 0;
    reg.A = set_nz(operated);
}

static inline uint8_t opASL(uint8_t data) {
    reg.status.carry = (data >> 7) & 1;
    return set_nz(data << 1);
}
static inline void opASL_a() { reg.A = opASL(reg.A); }
static inline void opASL_m(addr_t addr) { bus_write(addr, opASL(bus_read(addr))); }

static inline uint8_t opLSR(uint8_t data) {
    reg.status.carry = data & 1;
    return set_nz((data >> 1) & 0x7f);
}
static inline void opLSR_a() { reg.A = opLSR(reg.A); }
static inline void opLSR_m(addr_t addr) { bus_write(addr, opLSR(bus_read(addr))); }

static inline uint8_t opROL(uint8_t data) {
    uint8_t carry = (data >> 7) & 1;
    data = (data << 1) | reg.status.carry;
    reg.status.carry = carry;
    return set_nz(data);
}
static inline void opROL_a() { reg.A = opROL(reg.A); }
static inline void opROL_m(addr_t addr) { bus_write(addr, opROL(bus_read(addr))); }

static inline uint8_t opROR(uint8_t data) {
    uint8_t carry = data & 1;
    data = ((data >> 1) & 0x7f) | (reg.status.carry << 7);
    reg.status.carry = carry;
    return set_nz(data);
}
static inline void opROR_a() { reg.A = opROR(reg.A); }
static inline void opROR_m(addr_t addr) { bus_write(addr, opROR(bus_read(addr))); }

void render_next_line(uint8_t *line_buff) {
    bool line_started = false;
    bool line_ended = false;
    constexpr int BATCH_SIZE = 3;

    do {
        input::update();

        if (!line_started) {
            line_started |= !ppu::is_in_hblank();
        }
        else if (!line_ended) {
            line_ended |= ppu::is_in_hblank();
        }

        int cycle = 0;

        if (stopped) {
            cycle += 1; // nop
        }
        else if (dma::is_running()) {
            cycle += dma::exec_next_cycle();
        }
        else if (interrupt::is_nmi_asserted()) {
            interrupt::deassert_nmi();
            reg.status.breakmode = false;
            push(reg.PC >> 8);
            push(reg.PC & 0xff);
            push(reg.status.raw);
            reg.status.interrupt = true;
            reg.PC = bus_read_w(VEC_NMI);
            //cycle += 5; // ?
        }
        else if (interrupt::is_irq_asserted() && !reg.status.interrupt) {
            interrupt::deassert_irq();
            reg.status.breakmode = false;
            push(reg.PC >> 8);
            push(reg.PC & 0xff);
            push(reg.status.raw);
            reg.status.interrupt = true;
            reg.PC = bus_read_w(VEC_IRQ);
            //cycle += 5; // ?
        }
        else {
            int bs = BATCH_SIZE;
            while (bs-- > 0) {
                uint8_t op_code = fetch();

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

                case 0xea: /* NOP */                                    cycle += 2; break;

                default:
                    NES_ERRORF("UNKNOWN INSTRUCTION: 0x%02x (PC=0x%04x)\n", (int)op_code, (int)reg.PC);
                    break;
                }
            } // while (bs-- > 0)
        } // if

        ppu::render_next_block(line_buff, cycle * 3);

    } while (!line_ended);
}

}
