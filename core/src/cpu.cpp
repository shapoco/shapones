#include "stdint.h"
#include "shapones/shapones.hpp"

namespace nes::cpu {

// instruction decode table
static const OpDecRecord OpDecTable[] = {
    { BRK   , Implied   , 7 },  { ORA   , PreIdxInd , 6 },  { NOP   , Implied   , 2 },  { SLO   , PreIdxInd , 8 }, 
    { NOPD  , Implied   , 3 },  { ORA   , ZeroPage  , 3 },  { ASL   , ZeroPage  , 5 },  { SLO   , ZeroPage  , 5 }, 
    { PHP   , Implied   , 3 },  { ORA   , Immediate , 2 },  { ASL   , Accum     , 2 },  { ANC   , Immediate , 2 }, 
    { NOPI  , Implied   , 4 },  { ORA   , Absolute  , 4 },  { ASL   , Absolute  , 6 },  { SLO   , Absolute  , 6 }, 
    { BPL   , Relative  , 2 },  { ORA   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { SLO   , PostIdxInd, 8 }, 
    { NOPD  , Implied   , 4 },  { ORA   , ZeroPageX , 4 },  { ASL   , ZeroPageX , 6 },  { SLO   , ZeroPageX , 6 }, 
    { CLC   , Implied   , 2 },  { ORA   , AbsoluteY , 4 },  { NOP   , Implied   , 2 },  { SLO   , AbsoluteY , 7 }, 
    { NOPI  , Implied   , 4 },  { ORA   , AbsoluteX , 4 },  { ASL   , AbsoluteX , 6 },  { SLO   , AbsoluteX , 7 }, 
    { JSR   , Absolute  , 6 },  { AND   , PreIdxInd , 6 },  { NOP   , Implied   , 2 },  { RLA   , PreIdxInd , 8 }, 
    { BIT   , ZeroPage  , 3 },  { AND   , ZeroPage  , 3 },  { ROL   , ZeroPage  , 5 },  { RLA   , ZeroPage  , 5 }, 
    { PLP   , Implied   , 4 },  { AND   , Immediate , 2 },  { ROL   , Accum     , 2 },  { ANC   , Immediate , 2 }, 
    { BIT   , Absolute  , 4 },  { AND   , Absolute  , 4 },  { ROL   , Absolute  , 6 },  { RLA   , Absolute  , 6 }, 
    { BMI   , Relative  , 2 },  { AND   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { RLA   , PostIdxInd, 8 }, 
    { NOPD  , Implied   , 4 },  { AND   , ZeroPageX , 4 },  { ROL   , ZeroPageX , 6 },  { RLA   , ZeroPageX , 6 }, 
    { SEC   , Implied   , 2 },  { AND   , AbsoluteY , 4 },  { NOP   , Implied   , 2 },  { RLA   , AbsoluteY , 7 }, 
    { NOPI  , Implied   , 4 },  { AND   , AbsoluteX , 4 },  { ROL   , AbsoluteX , 6 },  { RLA   , AbsoluteX , 7 }, 
    { RTI   , Implied   , 6 },  { EOR   , PreIdxInd , 6 },  { NOP   , Implied   , 2 },  { SRE   , PreIdxInd , 8 }, 
    { NOPD  , Implied   , 3 },  { EOR   , ZeroPage  , 3 },  { LSR   , ZeroPage  , 5 },  { SRE   , ZeroPage  , 5 }, 
    { PHA   , Implied   , 3 },  { EOR   , Immediate , 2 },  { LSR   , Accum     , 2 },  { ALR   , Immediate , 2 }, 
    { JMP   , Absolute  , 3 },  { EOR   , Absolute  , 4 },  { LSR   , Absolute  , 6 },  { SRE   , Absolute  , 6 }, 
    { BVC   , Relative  , 2 },  { EOR   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { SRE   , PostIdxInd, 8 }, 
    { NOPD  , Implied   , 4 },  { EOR   , ZeroPageX , 4 },  { LSR   , ZeroPageX , 6 },  { SRE   , ZeroPageX , 6 }, 
    { CLI   , Implied   , 2 },  { EOR   , AbsoluteY , 4 },  { NOP   , Implied   , 2 },  { SRE   , AbsoluteY , 7 }, 
    { NOPI  , Implied   , 4 },  { EOR   , AbsoluteX , 4 },  { LSR   , AbsoluteX , 6 },  { SRE   , AbsoluteX , 7 }, 
    { RTS   , Implied   , 6 },  { ADC   , PreIdxInd , 6 },  { NOP   , Implied   , 2 },  { RRA   , PreIdxInd , 8 }, 
    { NOPD  , Implied   , 3 },  { ADC   , ZeroPage  , 3 },  { ROR   , ZeroPage  , 5 },  { RRA   , ZeroPage  , 5 }, 
    { PLA   , Implied   , 4 },  { ADC   , Immediate , 2 },  { ROR   , Accum     , 2 },  { ARR   , Immediate , 2 }, 
    { JMP   , IndAbs    , 5 },  { ADC   , Absolute  , 4 },  { ROR   , Absolute  , 6 },  { RRA   , Absolute  , 6 }, 
    { BVS   , Relative  , 2 },  { ADC   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { RRA   , PostIdxInd, 8 }, 
    { NOPD  , Implied   , 4 },  { ADC   , ZeroPageX , 4 },  { ROR   , ZeroPageX , 6 },  { RRA   , ZeroPageX , 6 }, 
    { SEI   , Implied   , 2 },  { ADC   , AbsoluteY , 4 },  { NOP   , Implied   , 2 },  { RRA   , AbsoluteY , 7 }, 
    { NOPI  , Implied   , 4 },  { ADC   , AbsoluteX , 4 },  { ROR   , AbsoluteX , 6 },  { RRA   , AbsoluteX , 7 }, 
    { NOPD  , Implied   , 2 },  { STA   , PreIdxInd , 6 },  { NOPD  , Implied   , 2 },  { SAX   , PreIdxInd , 6 }, 
    { STY   , ZeroPage  , 3 },  { STA   , ZeroPage  , 3 },  { STX   , ZeroPage  , 3 },  { SAX   , ZeroPage  , 3 }, 
    { DEY   , Implied   , 2 },  { NOPD  , Implied   , 2 },  { TXA   , Implied   , 2 },  { XAA   , Immediate , 2 }, 
    { STY   , Absolute  , 4 },  { STA   , Absolute  , 4 },  { STX   , Absolute  , 4 },  { SAX   , Absolute  , 4 }, 
    { BCC   , Relative  , 2 },  { STA   , PostIdxInd, 6 },  { NOP   , Implied   , 2 },  { AHX   , PostIdxInd, 6 }, 
    { STY   , ZeroPageX , 4 },  { STA   , ZeroPageX , 4 },  { STX   , ZeroPageY , 4 },  { SAX   , ZeroPageY , 4 }, 
    { TYA   , Implied   , 2 },  { STA   , AbsoluteY , 4 },  { TXS   , Implied   , 2 },  { TAS   , AbsoluteY , 5 }, 
    { SHY   , AbsoluteX , 5 },  { STA   , AbsoluteX , 4 },  { SHX   , AbsoluteY , 5 },  { AHX   , AbsoluteY , 5 }, 
    { LDY   , Immediate , 2 },  { LDA   , PreIdxInd , 6 },  { LDX   , Immediate , 2 },  { LAX   , PreIdxInd , 6 }, 
    { LDY   , ZeroPage  , 3 },  { LDA   , ZeroPage  , 3 },  { LDX   , ZeroPage  , 3 },  { LAX   , ZeroPage  , 3 }, 
    { TAY   , Implied   , 2 },  { LDA   , Immediate , 2 },  { TAX   , Implied   , 2 },  { LAX   , Immediate , 2 }, 
    { LDY   , Absolute  , 4 },  { LDA   , Absolute  , 4 },  { LDX   , Absolute  , 4 },  { LAX   , Absolute  , 4 }, 
    { BCS   , Relative  , 2 },  { LDA   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { LAX   , PostIdxInd, 5 }, 
    { LDY   , ZeroPageX , 4 },  { LDA   , ZeroPageX , 4 },  { LDX   , ZeroPageY , 4 },  { LAX   , ZeroPageY , 4 }, 
    { CLV   , Implied   , 2 },  { LDA   , AbsoluteY , 4 },  { TSX   , Implied   , 2 },  { LAS   , AbsoluteY , 4 }, 
    { LDY   , AbsoluteX , 4 },  { LDA   , AbsoluteX , 4 },  { LDX   , AbsoluteY , 4 },  { LAX   , AbsoluteY , 4 }, 
    { CPY   , Immediate , 2 },  { CMP   , PreIdxInd , 6 },  { NOPD  , Implied   , 2 },  { DCP   , PreIdxInd , 8 }, 
    { CPY   , ZeroPage  , 3 },  { CMP   , ZeroPage  , 3 },  { DEC   , ZeroPage  , 5 },  { DCP   , ZeroPage  , 5 }, 
    { INY   , Implied   , 2 },  { CMP   , Immediate , 2 },  { DEX   , Implied   , 2 },  { AXS   , Immediate , 2 }, 
    { CPY   , Absolute  , 4 },  { CMP   , Absolute  , 4 },  { DEC   , Absolute  , 6 },  { DCP   , Absolute  , 6 }, 
    { BNE   , Relative  , 2 },  { CMP   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { DCP   , PostIdxInd, 8 }, 
    { NOPD  , Implied   , 4 },  { CMP   , ZeroPageX , 4 },  { DEC   , ZeroPageX , 6 },  { DCP   , ZeroPageX , 6 }, 
    { CLD   , Implied   , 2 },  { CMP   , AbsoluteY , 4 },  { NOP   , Implied   , 2 },  { DCP   , AbsoluteY , 7 }, 
    { NOPI  , Implied   , 4 },  { CMP   , AbsoluteX , 4 },  { DEC   , AbsoluteX , 7 },  { DCP   , AbsoluteX , 7 }, 
    { CPX   , Immediate , 2 },  { SBC   , PreIdxInd , 6 },  { NOPD  , Implied   , 3 },  { ISB   , PreIdxInd , 8 }, 
    { CPX   , ZeroPage  , 3 },  { SBC   , ZeroPage  , 3 },  { INC   , ZeroPage  , 5 },  { ISB   , ZeroPage  , 5 }, 
    { INX   , Implied   , 2 },  { SBC   , Immediate , 2 },  { NOP   , Implied   , 2 },  { SBC   , Immediate , 2 }, 
    { CPX   , Absolute  , 4 },  { SBC   , Absolute  , 4 },  { INC   , Absolute  , 6 },  { ISB   , Absolute  , 6 }, 
    { BEQ   , Relative  , 2 },  { SBC   , PostIdxInd, 5 },  { NOP   , Implied   , 2 },  { ISB   , PostIdxInd, 8 }, 
    { NOPD  , Implied   , 4 },  { SBC   , ZeroPageX , 4 },  { INC   , ZeroPageX , 6 },  { ISB   , ZeroPageX , 6 }, 
    { SED   , Implied   , 2 },  { SBC   , AbsoluteY , 4 },  { NOP   , Implied   , 2 },  { ISB   , AbsoluteY , 7 }, 
    { NOPI  , Implied   , 4 },  { SBC   , AbsoluteX , 4 },  { INC   , AbsoluteX , 7 },  { ISB   , AbsoluteX , 7 },
};

static bool stopped = false;

static Registers reg;

static void process_interrupts();
static int process_next_inst(int num_inst);

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

static inline void set_nz(uint8_t value) {
    reg.status.negative = (value >> 7) & 1;
    reg.status.zero = (value == 0) ? 1 : 0;
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

static inline uint16_t fetch_addr_or_data(AddrMode mode, int *cycle) {
    switch (mode) {
    case Accum: return 0;
    case Implied: return 0;    
    case Immediate: return fetch();
    case Relative:
        {
            int distance = fetch();
            if (distance >= 0x80) distance -= 256;
            uint16_t retval = reg.PC + distance;
            if ((retval & 0xff00u) != (reg.PC & 0xff00u)) *cycle += 1;
            return retval;
        }
    case ZeroPage: return fetch();
    case ZeroPageX: return (fetch() + reg.X) & 0xff;
    case ZeroPageY: return (fetch() + reg.Y) & 0xff;
    case Absolute: return fetch_w();
    case AbsoluteX:
        {
            uint16_t base = fetch_w();
            uint16_t retval = base + reg.X;
            if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
            return retval;
        }
    case AbsoluteY:
        {
            uint16_t base = fetch_w();
            uint16_t retval = base + reg.Y;
            if ((base & 0xff00u) != (retval & 0xff00u)) *cycle += 1;
            return retval;
        }
    case PreIdxInd:
        {
            addr_t base = (fetch() + reg.X) & 0xffu;
            addr_t addr = bus_read(base) | ((uint16_t)bus_read((base + 1) & 0xffu) << 8);
            if ((addr & 0xff00u) != (base & 0xff00u)) *cycle += 1;
            return addr;
        }
    case PostIdxInd:
        {
            addr_t addrOrData = fetch();
            addr_t baseAddr = bus_read(addrOrData) | ((uint16_t)bus_read((addrOrData + 1) & 0xffu) << 8);
            addr_t addr = baseAddr + reg.Y;
            if ((addr & 0xff00u) != (baseAddr & 0xff00u)) *cycle += 1;
            return addr;
        }
    default: //case IndAbs:
        {
            addr_t addrOrData = fetch_w();
            addr_t next_addr = (addrOrData & 0xFF00) | (((addrOrData & 0xFF) + 1) & 0xFF);
            addr_t addr = bus_read(addrOrData) | ((uint16_t)bus_read(next_addr) << 8);
            return addr;
        }
    }
}

static inline void process_interrupts() {
    if (interrupt::is_nmi_asserted()) {
        interrupt::deassert_nmi();
        reg.status.breakmode = false;
        push(reg.PC >> 8);
        push(reg.PC & 0xff);
        push(reg.status.raw);
        reg.status.interrupt = true;
        reg.PC = bus_read_w(VEC_NMI);
    }
    else if (interrupt::is_irq_asserted() && !reg.status.interrupt) {
        interrupt::deassert_irq();
        reg.status.breakmode = false;
        push(reg.PC >> 8);
        push(reg.PC & 0xff);
        push(reg.status.raw);
        reg.status.interrupt = true;
        reg.PC = bus_read_w(VEC_IRQ);
    }
}

int exec_next_inst(int batch_size) {
    input::update();
    int cycle;
    if (stopped) {
        cycle = 1; // nop
    }
    else if (dma::is_running()) {
        cycle = dma::exec_next_cycle();
    }
    else {
        cycle = process_next_inst(batch_size);
    }
    
    return cycle;
}

static int process_next_inst(int batch_size) {
    int total_cycle = 0;
    
    process_interrupts();
    
    while (batch_size-- > 0) {
        
        addr_t op_addr = reg.PC;

        uint8_t op_code = fetch();

        auto op = OpDecTable[op_code];

        int cycle = op.cycle;

        const char* op_mode_str = "???";
        switch (op.addr_mode) {
        case Absolute   : op_mode_str = "Absolute";     break;
        case AbsoluteX  : op_mode_str = "AbsoluteX";    break;
        case AbsoluteY  : op_mode_str = "AbsoluteY";    break;
        case Accum      : op_mode_str = "Accum";        break;
        case Immediate  : op_mode_str = "Immediate";    break;
        case Implied    : op_mode_str = "Implied";      break;
        case IndAbs     : op_mode_str = "IndAbs";       break;
        case PostIdxInd : op_mode_str = "PostIdxInd";   break;
        case PreIdxInd  : op_mode_str = "PreIdxInd";    break;
        case Relative   : op_mode_str = "Relative";     break;
        case ZeroPage   : op_mode_str = "ZeroPage";     break;
        case ZeroPageX  : op_mode_str = "ZeroPageX";    break;
        case ZeroPageY  : op_mode_str = "ZeroPageY";    break;
        }


        uint16_t addr_or_data = fetch_addr_or_data((AddrMode)op.addr_mode, &cycle);

        const char* op_name = "???";

    #define NES_CPU_OP_CASE(op) \
        case op: op_name = #op;

        switch (op.mnemonic) {
        NES_CPU_OP_CASE(LDA)
            reg.A = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
            set_nz(reg.A);
            break;

        NES_CPU_OP_CASE(LDX)
            reg.X = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
            set_nz(reg.X);
            break;

        NES_CPU_OP_CASE(LDY)
            reg.Y = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
            set_nz(reg.Y);
            break;

        NES_CPU_OP_CASE(STA) 
            bus_write(addr_or_data, reg.A);
            break;

        NES_CPU_OP_CASE(STX)
            bus_write(addr_or_data, reg.X);
            break;

        NES_CPU_OP_CASE(STY)
            bus_write(addr_or_data, reg.Y);
            break;

        NES_CPU_OP_CASE(TAX)
            reg.X = reg.A;
            set_nz(reg.X);
            break;

        NES_CPU_OP_CASE(TAY)
            reg.Y = reg.A;
            set_nz(reg.Y);
            break;

        NES_CPU_OP_CASE(TSX)
            reg.X = reg.SP & 0xffu;
            set_nz(reg.X);
            break;

        NES_CPU_OP_CASE(TXA)
            reg.A = reg.X;
            set_nz(reg.A);
            break;

        NES_CPU_OP_CASE(TXS)
            reg.SP = reg.X + 0x0100;
            break;

        NES_CPU_OP_CASE(TYA)
            reg.A = reg.Y;
            set_nz(reg.A);
            break;

        NES_CPU_OP_CASE(ADC)
            {
                uint16_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                uint16_t operated = (uint16_t)reg.A + data + reg.status.carry;
                reg.status.overflow = (!(((reg.A ^ data) & 0x80) != 0) && (((reg.A ^ operated) & 0x80)) != 0);
                reg.status.carry = (operated >= 0x100) ? 1 : 0;
                reg.A = operated;
                set_nz(reg.A);
            }
            break;

        NES_CPU_OP_CASE(AND)
            {
                uint8_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                reg.A = data & reg.A;
                set_nz(reg.A);
            }
            break;

        NES_CPU_OP_CASE(ASL)
            {
                uint8_t data = (op.addr_mode == Accum) ? reg.A : bus_read(addr_or_data);
                reg.status.carry = (data >> 7) & 1;
                data <<= 1;
                set_nz(data);
                if (op.addr_mode == Accum) {
                    reg.A = data;
                }
                else {
                    bus_write(addr_or_data, data);
                }
            }
            break;
        
        NES_CPU_OP_CASE(BIT)
            {
                uint8_t data = bus_read(addr_or_data);
                reg.status.negative = (data >> 7) & 1;
                reg.status.overflow = (data >> 6) & 1;
                reg.status.zero = (reg.A & data) ? 0 : 1;
            }
            break;

        NES_CPU_OP_CASE(CMP)
            {
                uint8_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                int16_t compared = (int16_t)reg.A - (int16_t)data;
                reg.status.carry = compared >= 0;
                set_nz(compared);
            }
            break;

        NES_CPU_OP_CASE(CPX)
            {
                uint8_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                int16_t compared = (int16_t)reg.X - (int16_t)data;
                reg.status.carry = compared >= 0;
                set_nz(compared);
            }
            break;

        NES_CPU_OP_CASE(CPY)
            {
                uint8_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                int16_t compared = (int16_t)reg.Y - (int16_t)data;
                reg.status.carry = compared >= 0;
                set_nz(compared);
            }
            break;

        NES_CPU_OP_CASE(DEC)
            {
                uint8_t data = bus_read(addr_or_data) - 1;
                set_nz(data);
                bus_write(addr_or_data, data);
            }
            break;

        NES_CPU_OP_CASE(DEX)
            reg.X -= 1;
            set_nz(reg.X);
            break;

        NES_CPU_OP_CASE(DEY)
            reg.Y -= 1;
            set_nz(reg.Y);
            break;

        NES_CPU_OP_CASE(EOR)
            {
                uint8_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                reg.A = data ^ reg.A;
                set_nz(reg.A);
            }
            break;

        NES_CPU_OP_CASE(INC) 
            {
                uint8_t data = bus_read(addr_or_data) + 1;
                set_nz(data);
                bus_write(addr_or_data, data);
            }
            break;

        NES_CPU_OP_CASE(INX)
            reg.X += 1;
            set_nz(reg.X);
            break;

        NES_CPU_OP_CASE(INY)
            reg.Y += 1;
            set_nz(reg.Y);
            break;

        NES_CPU_OP_CASE(LSR) 
            {
                uint8_t data = (op.addr_mode == Accum) ? reg.A : bus_read(addr_or_data);
                reg.status.carry = data & 1;
                data = (data >> 1) & 0x7f;
                set_nz(data);
                if (op.addr_mode == Accum) {
                    reg.A = data;
                }
                else {
                    bus_write(addr_or_data, data);
                }
            }
            break;

        NES_CPU_OP_CASE(ORA) 
            {
                uint8_t data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                reg.A = data | reg.A;
                set_nz(reg.A);
            }
            break;

        NES_CPU_OP_CASE(ROL) 
            {
                uint8_t data = (op.addr_mode == Accum) ? reg.A : bus_read(addr_or_data);
                uint8_t carry = (data >> 7) & 1;
                data = (data << 1) | reg.status.carry;
                reg.status.carry = carry;
                set_nz(data);
                if (op.addr_mode == Accum) {
                    reg.A = data;
                }
                else {
                    bus_write(addr_or_data, data);
                }
            }
            break;

        NES_CPU_OP_CASE(ROR) 
            {
                uint8_t data = (op.addr_mode == Accum) ? reg.A : bus_read(addr_or_data);
                uint8_t carry = data & 1;
                data = ((data >> 1) & 0x7f) | (reg.status.carry << 7);
                reg.status.carry = carry;
                set_nz(data);
                if (op.addr_mode == Accum) {
                    reg.A = data;
                }
                else {
                    bus_write(addr_or_data, data);
                }
            }
            break;

        NES_CPU_OP_CASE(SBC) 
            {
                int data = (op.addr_mode == Immediate) ? addr_or_data : bus_read(addr_or_data);
                int operated = (int)reg.A - data - (reg.status.carry ? 0 : 1);
                reg.status.overflow = (((reg.A ^ operated) & 0x80) != 0 && ((reg.A ^ data) & 0x80) != 0);
                reg.status.carry = (operated >= 0) ? 1 : 0;
                reg.A = operated;
                set_nz(reg.A);
            }
            break;

        NES_CPU_OP_CASE(PHA)
            push(reg.A);
            break;
        
        NES_CPU_OP_CASE(PHP) 
            reg.status.breakmode = 1;
            push(reg.status.raw);
            break;
        
        NES_CPU_OP_CASE(PLA) 
            reg.A = pop();
            set_nz(reg.A);
            break;
        
        NES_CPU_OP_CASE(PLP)
            reg.status.raw = pop();
            reg.status.reserved = 1;
            break;

        NES_CPU_OP_CASE(JMP)
            reg.PC = addr_or_data;
            break;
        
        NES_CPU_OP_CASE(JSR)
            {
                addr_t pc = reg.PC - 1;
                push(pc >> 8);
                push(pc & 0xff);
                reg.PC = addr_or_data;
            }
            break;
        
        NES_CPU_OP_CASE(RTS)
            reg.PC = (addr_t)pop();
            reg.PC |= ((addr_t)pop() << 8);
            reg.PC += 1;
            break;

        NES_CPU_OP_CASE(RTI)
            reg.status.raw = pop();
            reg.status.reserved = true;
            reg.PC = (addr_t)pop();
            reg.PC |= ((addr_t)pop() << 8);
            break;

        NES_CPU_OP_CASE(BCC)
            if (!reg.status.carry) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;
        
        NES_CPU_OP_CASE(BCS)
            if (reg.status.carry) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;
        
        NES_CPU_OP_CASE(BEQ)
            if (reg.status.zero) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;
        
        NES_CPU_OP_CASE(BMI)
            if (reg.status.negative) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;
        
        NES_CPU_OP_CASE(BNE)
            if (!reg.status.zero) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;
            
        NES_CPU_OP_CASE(BPL)
            if (!reg.status.negative) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;

        NES_CPU_OP_CASE(BVS)
            if (reg.status.overflow) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;

        NES_CPU_OP_CASE(BVC)
            if (!reg.status.overflow) { 
                reg.PC = addr_or_data;
                cycle += 1;
            }
            break;
        
        NES_CPU_OP_CASE(CLD)
            reg.status.decimalmode = 0;
            break;

        NES_CPU_OP_CASE(CLC)
            reg.status.carry = 0;
            break;
        
        NES_CPU_OP_CASE(CLI)
            reg.status.interrupt = 0;
            break;
        
        NES_CPU_OP_CASE(CLV)
            reg.status.overflow = 0;
            break;
        
        NES_CPU_OP_CASE(SEC)
            reg.status.carry = 1;
            break;
        
        NES_CPU_OP_CASE(SEI)
            reg.status.interrupt = 1;
            break;
        
        NES_CPU_OP_CASE(SED)
            reg.status.decimalmode = 1;
            break;
        
        NES_CPU_OP_CASE(BRK)
            {
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
            break;

        NES_CPU_OP_CASE(NOP)
            break;

        default:
            NES_ERRORF("UNKNOWN INSTRUCTION: 0x%02x (PC=0x%04x)\n", (int)op_code, (int)reg.PC);
            break;
        }

        //NES_PRINTF("[%04x] %-4s %-12s (0x%02x) 0x%04x A:%02x X:%02x Y:%02x P:%02x SP:%02x\n",
        //    (int)op_addr, op_name, op_mode_str, (int)op_code, (int)addr_or_data,
        //    (int)reg.A, (int)reg.X, (int)reg.Y, (int)reg.status.raw, (int)reg.SP);
        total_cycle += cycle;
    }
    return total_cycle;
}

}
