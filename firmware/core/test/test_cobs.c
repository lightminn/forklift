#include "fw/cobs.h"
#include "fw_test.h"

#include <string.h>

static int encodes_to(const uint8_t *in, size_t len, const uint8_t *want, size_t want_len)
{
    uint8_t out[300];
    size_t n = cobs_encode(in, len, out, sizeof out);
    return n == want_len && memcmp(out, want, want_len) == 0;
}

static int round_trips(const uint8_t *in, size_t len)
{
    uint8_t enc[300], dec[300];
    size_t n = cobs_encode(in, len, enc, sizeof enc);
    size_t m = cobs_decode(enc, n, dec, sizeof dec);
    return n > 0 && m == len && memcmp(dec, in, len) == 0;
}

int main(void)
{
    /* Standard COBS examples (Cheshire & Baker). */
    const uint8_t a[] = {0x00}, a_out[] = {0x01, 0x01};
    const uint8_t b[] = {0x00, 0x00}, b_out[] = {0x01, 0x01, 0x01};
    const uint8_t c[] = {0x11, 0x22, 0x00, 0x33}, c_out[] = {0x03, 0x11, 0x22, 0x02, 0x33};
    const uint8_t d[] = {0x11, 0x22, 0x33, 0x44}, d_out[] = {0x05, 0x11, 0x22, 0x33, 0x44};
    CHECK(encodes_to(a, sizeof a, a_out, sizeof a_out));
    CHECK(encodes_to(b, sizeof b, b_out, sizeof b_out));
    CHECK(encodes_to(c, sizeof c, c_out, sizeof c_out));
    CHECK(encodes_to(d, sizeof d, d_out, sizeof d_out));
    CHECK(round_trips(a, sizeof a) && round_trips(b, sizeof b));
    CHECK(round_trips(c, sizeof c) && round_trips(d, sizeof d));

    /* 0x01..0xFE: one full block, no trailing code byte (255 bytes, not 256). */
    uint8_t full[254], full_out[255];
    for (size_t i = 0; i < sizeof full; i++) {
        full[i] = (uint8_t)(i + 1);
        full_out[i + 1] = (uint8_t)(i + 1);
    }
    full_out[0] = 0xFF;
    CHECK(encodes_to(full, sizeof full, full_out, sizeof full_out));
    CHECK(round_trips(full, sizeof full));

    /* Malformed input: an embedded zero, or a code pointing past the end. */
    uint8_t out[16];
    const uint8_t zero_inside[] = {0x03, 0x11, 0x00};
    const uint8_t overrun[] = {0x05, 0x11, 0x22};
    CHECK_EQ_U(cobs_decode(zero_inside, sizeof zero_inside, out, sizeof out), 0);
    CHECK_EQ_U(cobs_decode(overrun, sizeof overrun, out, sizeof out), 0);
    /* No room. */
    CHECK_EQ_U(cobs_encode(d, sizeof d, out, 4), 0);
    CHECK_EQ_U(cobs_decode(d_out, sizeof d_out, out, 3), 0);
    return FW_TEST_RESULT();
}
