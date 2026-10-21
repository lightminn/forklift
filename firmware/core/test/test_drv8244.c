#include "fw/drv8244.h"
#include "fw_test.h"

/* A healthy SDO word: status byte 0b11 + no faults, report byte = data. */
static uint16_t ok_sdo(uint8_t report)
{
    return (uint16_t)(0xC000u | report);
}

static void frames(void)
{
    /* SLVSG24C table 8-20: B15 = 0, B14 = W0 (1 = read), A5..A0, D7..D0. */
    CHECK_EQ_U(drv8244_write_frame(0x0C, 0x40), 0x0C40);
    CHECK_EQ_U(drv8244_read_frame(0x01), 0x4100);
    CHECK(drv8244_response_valid(0xC000));
    CHECK(drv8244_response_valid(0xFF12));
    CHECK(!drv8244_response_valid(0x8000));
    CHECK(!drv8244_response_valid(0x4000));
    CHECK_EQ_U(drv8244_status_byte(0xE5AA), 0xE5);
    CHECK(drv8244_config3_valid(0x40));  /* reset value: TOFF 01b, S_MODE 00b (PH/EN) */
    CHECK(!drv8244_config3_valid(0x41)); /* S_MODE 01b, independent mode */
    CHECK(!drv8244_config3_valid(0x42));
}

/* Runs the sequencer with a well-behaved device; returns the frames it sent. */
static int run_sequence(uint32_t t0, uint8_t config3, uint8_t config4, uint16_t *sent, int max,
                        int corrupt_at, drv8244_init *ini)
{
    drv8244_init_start(ini, t0, 1000);
    uint16_t frame = 0, sdo = 0;
    uint8_t regs[0x0E] = {0};
    int n = 0;
    for (uint32_t t = t0; n < max && !ini->done && !ini->failed; t += 100) {
        if (!drv8244_init_next(ini, t, sdo, config3, config4, &frame)) {
            continue;
        }
        sent[n] = frame;
        uint8_t addr = (uint8_t)((frame >> 8) & 0x3Fu);
        bool read = (frame & 0x4000u) != 0;
        if (!read && addr < sizeof regs) {
            regs[addr] = (uint8_t)frame;
        }
        sdo = ok_sdo(read && addr < sizeof regs ? regs[addr] : 0);
        if (n == corrupt_at) {
            sdo = 0x0000; /* no device / no VM: status bits not 11 */
        }
        n++;
    }
    return n;
}

static void sequence(uint32_t t0)
{
    /* SLVSG24C 8.6.1.5 COMMAND: 0x0809 unlocks CONFIG (REG_LOCK 01b, SPI_IN_LOCK kept 01b),
       0x0888 = CLR_FLT, 0x080A locks CONFIG (REG_LOCK 10b). */
    const uint16_t want[] = {0x0809, 0x0C40, 0x0D04, 0x4C00, 0x4D00, 0x0888, 0x080A, 0x4100};
    uint16_t sent[16] = {0};
    drv8244_init ini;
    drv8244_init_start(&ini, t0, 1000);
    uint16_t frame = 0;
    CHECK(!drv8244_init_next(&ini, t0 + 999, 0, 0x40, 0x04, &frame)); /* tREADY wait */
    int n = run_sequence(t0, 0x40, 0x04, sent, 16, -1, &ini);
    CHECK(ini.done);
    CHECK(!ini.failed);
    CHECK_EQ_I(n, 8);
    for (int i = 0; i < 8; i++) {
        CHECK_EQ_U(sent[i], want[i]);
    }
}

static void readback_mismatch_fails(void)
{
    /* A device that ignores CONFIG3 writes (still locked) reads back its reset value 0x40. */
    drv8244_init ini;
    drv8244_init_start(&ini, 0, 1000);
    uint16_t frame = 0, sdo = 0;
    for (uint32_t t = 1000; t < 10000 && !ini.done && !ini.failed; t += 100) {
        if (drv8244_init_next(&ini, t, sdo, 0x44, 0x04, &frame)) {
            sdo = frame == 0x4C00 ? ok_sdo(0x40) : ok_sdo(frame == 0x4D00 ? 0x04 : 0);
        }
    }
    CHECK(ini.failed);
}

static void bad_status_fails(void)
{
    uint16_t sent[16];
    drv8244_init ini;
    run_sequence(0, 0x40, 0x04, sent, 16, 0, &ini);
    CHECK(ini.failed);
    CHECK(!ini.done);
}

int main(void)
{
    frames();
    sequence(0);
    sequence(0xFFFFFF00u); /* MCU clock wraps during the wake wait */
    readback_mismatch_fails();
    bad_status_fails();
    return FW_TEST_RESULT();
}
