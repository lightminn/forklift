/* Link CRC-16 and KTH7801 CRC-4 (mcu-link/1 contract section 1; design section 5). */
#ifndef FW_CRC_H
#define FW_CRC_H

#include <stddef.h>
#include <stdint.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0. */
uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

/* CRC-4/G-704 (CRC-4/ITU): poly x^4+x+1 reflected, init 0, xorout 0. */
uint8_t crc4_g704_bytes(const uint8_t *data, size_t len);

/* The KTH7801 datasheet example (0x0FF -> 0x2) fits both bit orders; pick on first hardware. */
typedef enum {
    KTH_CRC_G704_BYTES = 0,          /* angle as bytes {a >> 8, a & 0xFF}, CRC-4/G-704 */
    KTH_CRC_REFLECTED_MSB_FIRST = 1, /* 12 angle bits, MSB first, into the reflected register */
} kth_crc_variant;

uint8_t crc4_kth7801(uint16_t angle12, kth_crc_variant variant);

#endif
