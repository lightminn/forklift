/* Consistent Overhead Byte Stuffing without the 0x00 delimiter (mcu-link/1 section 1). */
#ifndef FW_COBS_H
#define FW_COBS_H

#include <stddef.h>
#include <stdint.h>

/* Returns the encoded length, or 0 when out_cap is too small. */
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

/* Returns the decoded length, or 0 for malformed input or too small out_cap. */
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

#endif
