#ifndef SHAPONES_CPU_HPP
#define SHAPONES_CPU_HPP

#include "shapones/shapones.hpp"

namespace nes::cpu {

static constexpr addr_t WRAM_BASE           = 0x0;
static constexpr addr_t WRAM_MIRROR_BASE    = 0x800;
static constexpr addr_t PPUREG_BASE         = 0x2000;
static constexpr addr_t OAM_DMA_REG         = 0x4014;
static constexpr addr_t INPUT_REG_0         = 0x4016;
static constexpr addr_t INPUT_REG_1         = 0x4017;
static constexpr addr_t PRGROM_BASE         = 0x8000;

static constexpr addr_t VEC_NMI     = 0xfffa;
static constexpr addr_t VEC_RESET   = 0xfffc;
static constexpr addr_t VEC_IRQ     = 0xfffe;

static constexpr uint8_t STATUS_CARRY       = 0x1;
static constexpr uint8_t STATUS_ZERO        = 0x2;
static constexpr uint8_t STATUS_INTERRUPT   = 0x4;
static constexpr uint8_t STATUS_DECIMALMODE = 0x8;
static constexpr uint8_t STATUS_BREAKMODE   = 0x10;
static constexpr uint8_t STATUS_OVERFLOW    = 0x40;
static constexpr uint8_t STATUS_NEGATIVE    = 0x80;

struct Registers {
    uint8_t A; // accumulator
    uint8_t X; // index
    uint8_t Y; // index
    uint8_t SP; // stack pointer (low byte)
    union { // status
        uint8_t raw;
        struct {
            uint8_t carry : 1;
            uint8_t zero : 1;
            uint8_t interrupt : 1;
            uint8_t decimalmode : 1;
            uint8_t breakmode : 1;
            uint8_t reserved : 1;
            uint8_t overflow : 1;
            uint8_t negative : 1;
        };
    } status;
    addr_t PC; // program counter
};

enum Mnemonic : uint8_t {
    ADC , AHX , ALR , ANC , AND , ARR , ASL , AXS , BCC , BCS , BEQ , BIT , BMI , BNE , BPL , BRK ,
    BVC , BVS , CLC , CLD , CLI , CLV , CMP , CPX , CPY , DCP , DEC , DEX , DEY , EOR , INC , INX ,
    INY , ISB , JMP , JSR , LAS , LAX , LDA , LDX , LDY , LSR , NOP , NOPD, NOPI, ORA , PHA , PHP ,
    PLA , PLP , RLA , ROL , ROR , RRA , RTI , RTS , SAX , SBC , SEC , SED , SEI , SHX , SHY , SLO ,
    SRE , STA , STX , STY , TAS , TAX , TAY , TSX , TXA , TXS , TYA , XAA 
};

enum AddrMode : uint8_t {
    Absolute    , AbsoluteX , AbsoluteY , Accum     ,
    Immediate   , Implied   , IndAbs    , PostIdxInd,
    PreIdxInd   , Relative  , ZeroPage  , ZeroPageX ,
    ZeroPageY
};

struct OpDecRecord {
    Mnemonic mnemonic;
    AddrMode addr_mode;
    uint8_t cycle;
};

enum RegSel {
    A, X, Y
};

void render_next_line(uint8_t *line_buff);

void reset();
void stop();

uint8_t bus_read(addr_t addr);
void bus_write(addr_t addr, uint8_t data);

}

#endif
