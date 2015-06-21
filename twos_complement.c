#include "twos_complement.h"

int16_t from_bytes16_limited(uint8_t low, uint8_t hi, uint8_t sig_bits) {
  int16_t mask = ~(0xffff << sig_bits);
  int16_t out = (low | (hi << 8)) & mask; /*  */
  out = le16toh(out);
  if (out & (1 << (sig_bits-1))) {/* number is negative */
    out = ((out ^ mask) + 1);   /* undo two's complement */
    out = -out;                 /* make native negative */
  }
  return out;
}

int16_t from_bytes16(uint8_t low, uint8_t hi) {
  int16_t out = low | (hi << 8);
  out = le16toh(out);
  if (out & 0x8000) {           /* number is negative */
    out = ((out ^ 0xffff) + 1); /* undo two's complement */
    out = -out;                 /* make native negative */
  }
  return out;
}

int32_t from_bytes24(uint8_t low, uint8_t mid, uint8_t hi) {
  int32_t out = low | (mid << 8) | (hi << 16);
  out = le32toh(out);
  if (out & 0x800000) {                      /* number is negative */
    out = ((out ^ 0xffffff) + 1) & 0xffffff; /* undo two's complement */
    out = -out;                              /* make native negative */
  }
  return out;
}
