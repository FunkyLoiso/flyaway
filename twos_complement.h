#ifndef TWOS_COMPLEMENT_H_INCLUDED
#define TWOS_COMPLEMENT_H_INCLUDED

#include <stdint.h>
#include <endian.h>

int16_t from_bytes(uint8_t low, uint8_t hi) {
  int16_t out = low | (hi << 8);
  out = le16toh(out);
  if (out & 0x8000) {           /* number is negative */
    out = ((out ^ 0xffff) + 1); /* undo two's complement */
    out = -out;                 /* make native negative */
  }
  return out;
}

int32_t from_bytes(uint8_t low, uint8_t mid, uint8_t hi) {
  int32_t out = low | (mid << 8) | (high << 16);
  out = le32toh(out);
  if (out & 0x800000) {                      /* number is negative */
    out = ((out ^ 0xffffff) + 1) & 0xffffff; /* undo two's complement */
    out = -out;                              /* make native negative */
  }
  return out;
}

#endif
