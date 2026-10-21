#include "fw/crc.h"
#include "fw_test.h"

static const uint8_t CHECK_STRING[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

int main(void)
{
    /* Catalogue check values: CRC-16/CCITT-FALSE 0x29B1, CRC-4/G-704 0x7. */
    CHECK_EQ_U(crc16_ccitt_false(CHECK_STRING, sizeof CHECK_STRING), 0x29B1);
    CHECK_EQ_U(crc16_ccitt_false(CHECK_STRING, 0), 0xFFFF);
    CHECK_EQ_U(crc4_g704_bytes(CHECK_STRING, sizeof CHECK_STRING), 0x7);

    /* KTH7801 datasheet 5.6: position 0x0FF carries CRC 0x2 (received word 0x0FF2). */
    CHECK_EQ_U(crc4_kth7801(0x0FF, KTH_CRC_G704_BYTES), 0x2);
    CHECK_EQ_U(crc4_kth7801(0x0FF, KTH_CRC_REFLECTED_MSB_FIRST), 0x2);
    /* The one example does not pin the bit order: the variants differ elsewhere. */
    CHECK(crc4_kth7801(0x123, KTH_CRC_G704_BYTES) !=
          crc4_kth7801(0x123, KTH_CRC_REFLECTED_MSB_FIRST));
    return FW_TEST_RESULT();
}
