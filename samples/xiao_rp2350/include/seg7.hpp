#pragma once
#include <stdint.h>

#ifdef SEG7_USE_PROGMEM
#include <avr/pgmspace.h>
#define SEG7_PROGMEM PROGMEM
#define seg7_pgmReadByte pgm_read_byte
#else
#define SEG7_PROGMEM
#define seg7_pgmReadByte(addr) (*(const uint8_t*)(addr))
#endif

#ifdef SEG7_INCLUDE_AS_STATIC
#define SEG7_FUNC_MOD static
#else
#define SEG7_FUNC_MOD
#endif

#ifndef SEG7_POS_TYPE
#define SEG7_POS_TYPE int16_t
#endif

#if defined(SEG7_INCLUDE_AS_STATIC) || !defined(SEG7_INCLUDE_IMPL)
#define SEG7_DOT_POS_DFLT = -1
#define SEG7_NUM_DIG_DFLT = 2
#else
#define SEG7_DOT_POS_DFLT
#define SEG7_NUM_DIG_DFLT
#endif

namespace seg7 {

using pos_t = SEG7_POS_TYPE;

static constexpr uint8_t SEG7_NEG_SIGN = 16;
static constexpr uint8_t SEG7_DOT_MASK = 0x80;

SEG7_FUNC_MOD void fillRect(pos_t x, pos_t y, uint8_t w, uint8_t h);

#ifdef SEG7_INCLUDE_IMPL
static const uint8_t SEG7_LUT[] SEG7_PROGMEM = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c,
    0x58, 0x5e, 0x79, 0x71, 0x40, 0x01, 0x1d, 0x5d, 0x81, 0x58, 0x18, 0x41};
#endif

#ifdef SEG7_FIXED_SIZE
static constexpr uint8_t digSize = SEG7_FIXED_SIZE;
#else
static uint8_t digSize = 1;
SEG7_FUNC_MOD void setDigitSize(uint8_t size)
#ifdef SEG7_INCLUDE_IMPL
{
  digSize = size;
}
#else
    ;
#endif
#endif

SEG7_FUNC_MOD pos_t drawDigit(pos_t x, pos_t y, uint8_t dig)
#ifdef SEG7_INCLUDE_IMPL
{
  uint8_t size = digSize;
  uint8_t seg = seg7_pgmReadByte(&SEG7_LUT[dig & 0x01f]);
  for (uint8_t iseg = 0; iseg < 7; iseg++) {
    if (seg & 1) {
      uint8_t pos = seg7_pgmReadByte(&SEG7_LUT[17 + iseg]);
      pos_t segX = x + size * (pos & 0x07);
      pos_t segY = y + size * ((pos >> 4) & 0x0f);
      bool vertical = 0 != (pos & 0x08);
      uint8_t segW = vertical ? size : (size * 4);
      uint8_t segH = vertical ? (size * 3) : size;
      fillRect(segX, segY, segW, segH);
    }
    seg >>= 1;
  }
  if (dig & SEG7_DOT_MASK) {
    uint8_t dotSize = size * 2;
    fillRect(x + size * 7, y + 8 * size, dotSize, dotSize);
    x += dotSize;
  }
  return x + size * 8;
}
#else
    ;
#endif

SEG7_FUNC_MOD pos_t drawString(pos_t x, pos_t y, const uint8_t* digs,
                               uint8_t len)
#ifdef SEG7_INCLUDE_IMPL
{
  for (uint8_t idig = 0; idig < len; idig++) {
    x = drawDigit(x, y, digs[idig]);
  }
  return x;
}
#else
    ;
#endif

SEG7_FUNC_MOD pos_t drawHex32(pos_t x, pos_t y, uint32_t val,
                              int8_t numDigs SEG7_NUM_DIG_DFLT)
#ifdef SEG7_INCLUDE_IMPL
{
  constexpr uint8_t BUFF_LEN = sizeof(val) * 2;
  uint8_t digs[BUFF_LEN];
  int8_t i = BUFF_LEN;
  do {
    digs[--i] = val & 0x0F;
    val >>= 4;
  } while (--numDigs > 0 || val != 0);
  return drawString(x, y, digs + i, BUFF_LEN - i);
}
#else
    ;
#endif

SEG7_FUNC_MOD pos_t drawDec32(pos_t x, pos_t y, int32_t val,
                              int8_t dotPos SEG7_DOT_POS_DFLT)
#ifdef SEG7_INCLUDE_IMPL
{
  // sizeof(T) * log(256) / log(10) + 1
  constexpr uint8_t BUFF_LEN = (sizeof(val) * 616 + 255) / 256 + 1;
  bool neg = val < 0;
  if (neg) val = -val;
  uint8_t digs[BUFF_LEN];
  uint8_t i = BUFF_LEN;
  do {
    digs[--i] = val % 10;
    val /= 10;
    if (dotPos-- == 0) {
      digs[i] |= SEG7_DOT_MASK;
    }
  } while (val > 0 || dotPos >= 0);
  if (neg) {
    digs[--i] = SEG7_NEG_SIGN;
  }
  return drawString(x, y, digs + i, BUFF_LEN - i);
}
#else
    ;
#endif

#ifdef SEG7_PROGMEM
#undef SEG7_PROGMEM
#endif

#ifdef seg7_pgmReadByte
#undef seg7_pgmReadByte
#endif

}  // namespace seg7
