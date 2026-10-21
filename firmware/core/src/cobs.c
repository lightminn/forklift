#include "fw/cobs.h"

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    if (out_cap == 0) {
        return 0;
    }
    size_t code_at = 0; /* where the current block's code byte goes */
    size_t n = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_at] = code;
            if (n >= out_cap) {
                return 0;
            }
            code_at = n++;
            code = 1;
            continue;
        }
        if (n >= out_cap) {
            return 0;
        }
        out[n++] = in[i];
        code++;
        /* A full block ends here; open the next one only if more input follows. */
        if (code == 0xFF && i + 1 < len) {
            out[code_at] = code;
            if (n >= out_cap) {
                return 0;
            }
            code_at = n++;
            code = 1;
        }
    }
    out[code_at] = code;
    return n;
}

size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    size_t i = 0, n = 0;
    while (i < len) {
        uint8_t code = in[i++];
        if (code == 0 || i + (size_t)(code - 1) > len) {
            return 0;
        }
        for (uint8_t k = 1; k < code; k++) {
            if (in[i] == 0 || n >= out_cap) {
                return 0;
            }
            out[n++] = in[i++];
        }
        /* A zero follows every block except a full one and the last. */
        if (code != 0xFF && i < len) {
            if (n >= out_cap) {
                return 0;
            }
            out[n++] = 0;
        }
    }
    return n;
}
