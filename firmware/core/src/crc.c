#include "fw/crc.h"

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)(crc ^ ((uint16_t)data[i] << 8));
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* One input bit into the reflected CRC-4 register (poly 0x3 reflected = 0xC). */
static uint8_t crc4_reflected_bit(uint8_t reg, unsigned bit)
{
    unsigned feedback = (reg ^ bit) & 1u;
    reg = (uint8_t)(reg >> 1);
    return feedback ? (uint8_t)(reg ^ 0xCu) : reg;
}

uint8_t crc4_g704_bytes(const uint8_t *data, size_t len)
{
    uint8_t reg = 0;
    for (size_t i = 0; i < len; i++) {
        for (unsigned bit = 0; bit < 8; bit++) { /* reflected input: LSB first */
            reg = crc4_reflected_bit(reg, (data[i] >> bit) & 1u);
        }
    }
    return reg;
}

uint8_t crc4_kth7801(uint16_t angle12, kth_crc_variant variant)
{
    angle12 &= 0x0FFFu;
    if (variant == KTH_CRC_G704_BYTES) {
        const uint8_t bytes[2] = {(uint8_t)(angle12 >> 8), (uint8_t)(angle12 & 0xFFu)};
        return crc4_g704_bytes(bytes, sizeof bytes);
    }
    uint8_t reg = 0;
    for (int bit = 11; bit >= 0; bit--) {
        reg = crc4_reflected_bit(reg, (angle12 >> bit) & 1u);
    }
    return reg;
}
