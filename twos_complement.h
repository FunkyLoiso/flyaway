#ifndef TWOS_COMPLEMENT_H_INCLUDED
#define TWOS_COMPLEMENT_H_INCLUDED

#include <stdint.h>
#include <endian.h>

int16_t from_bytes16_limited(uint8_t low, uint8_t hi, uint8_t sig_bits);
int16_t from_bytes16(uint8_t low, uint8_t hi);
int32_t from_bytes24(uint8_t low, uint8_t mid, uint8_t hi);

#endif
