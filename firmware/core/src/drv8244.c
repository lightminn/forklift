#include "fw/drv8244.h"

/* COMMAND (SLVSG24C 8.6.1.5): CLR_FLT bit 7, SPI_IN_LOCK bits 4..3, REG_LOCK bits 1..0.
   SPI_IN_LOCK stays 01b (locked, its reset value); REG_LOCK 01b unlocks, 10b locks. */
#define CMD_UNLOCK 0x09u
#define CMD_CLR_FLT 0x88u
#define CMD_LOCK 0x0Au

uint16_t drv8244_write_frame(uint8_t addr, uint8_t data)
{
    return (uint16_t)(((addr & 0x3Fu) << 8) | data);
}

uint16_t drv8244_read_frame(uint8_t addr)
{
    return (uint16_t)(0x4000u | ((addr & 0x3Fu) << 8));
}

bool drv8244_response_valid(uint16_t sdo)
{
    return (sdo & 0xC000u) == 0xC000u;
}

uint8_t drv8244_status_byte(uint16_t sdo)
{
    return (uint8_t)(sdo >> 8);
}

bool drv8244_config3_valid(uint8_t config3)
{
    return (config3 & 0x03u) == 0;
}

void drv8244_init_start(drv8244_init *ini, uint32_t now_us, uint32_t wake_wait_us)
{
    ini->step = 0;
    ini->wait_start_us = now_us;
    ini->wait_us = wake_wait_us;
    ini->done = false;
    ini->failed = false;
    ini->fault_summary = 0;
}

static bool fail(drv8244_init *ini)
{
    ini->failed = true;
    return false;
}

bool drv8244_init_next(drv8244_init *ini, uint32_t now_us, uint16_t last_sdo, uint8_t config3,
                       uint8_t config4, uint16_t *frame_out)
{
    if (ini->done || ini->failed) {
        return false;
    }
    if (ini->step == 0) {
        if (now_us - ini->wait_start_us < ini->wait_us) { /* modular: survives clock wrap */
            return false;
        }
        ini->step = 1;
        *frame_out = drv8244_write_frame(DRV8244_REG_COMMAND, CMD_UNLOCK);
        return true;
    }
    /* Every later step first checks the response to the frame sent before it. */
    if (!drv8244_response_valid(last_sdo)) {
        return fail(ini);
    }
    uint8_t report = (uint8_t)last_sdo;
    switch (ini->step++) {
    case 1:
        *frame_out = drv8244_write_frame(DRV8244_REG_CONFIG3, config3);
        return true;
    case 2:
        *frame_out = drv8244_write_frame(DRV8244_REG_CONFIG4, config4);
        return true;
    case 3:
        *frame_out = drv8244_read_frame(DRV8244_REG_CONFIG3);
        return true;
    case 4:
        if (report != config3) {
            return fail(ini);
        }
        *frame_out = drv8244_read_frame(DRV8244_REG_CONFIG4);
        return true;
    case 5:
        if (report != config4) {
            return fail(ini);
        }
        *frame_out = drv8244_write_frame(DRV8244_REG_COMMAND, CMD_CLR_FLT);
        return true;
    case 6:
        *frame_out = drv8244_write_frame(DRV8244_REG_COMMAND, CMD_LOCK);
        return true;
    case 7:
        *frame_out = drv8244_read_frame(DRV8244_REG_FAULT_SUMMARY);
        return true;
    default:
        ini->fault_summary = report;
        ini->done = true;
        return false;
    }
}
