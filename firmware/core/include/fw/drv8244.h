/* DRV8244S-Q1 (SPI variant) frames and initialisation sequencer (TI SLVSG24C 8.5-8.6). */
#ifndef FW_DRV8244_H
#define FW_DRV8244_H

#include <stdbool.h>
#include <stdint.h>

#define DRV8244_REG_FAULT_SUMMARY 0x01u
#define DRV8244_REG_STATUS1 0x02u
#define DRV8244_REG_COMMAND 0x08u
#define DRV8244_REG_CONFIG3 0x0Cu
#define DRV8244_REG_CONFIG4 0x0Du

/* Status byte of every SDO word (table 8-21): bits 7..6 always 11, then these. */
#define DRV8244_ST_FAULT 0x20u
#define DRV8244_ST_VMOV 0x10u
#define DRV8244_ST_VMUV 0x08u
#define DRV8244_ST_OCP 0x04u
#define DRV8244_ST_TSD 0x02u
#define DRV8244_ST_SPI_ERR 0x01u

uint16_t drv8244_write_frame(uint8_t addr, uint8_t data);
uint16_t drv8244_read_frame(uint8_t addr);
/* The two status MSBs are forced to 11; anything else means no device or a wiring fault. */
bool drv8244_response_valid(uint16_t sdo);
uint8_t drv8244_status_byte(uint16_t sdo);
/* The output mapping assumes PH/EN mode: CONFIG3.S_MODE must be 00b. */
bool drv8244_config3_valid(uint8_t config3);

typedef struct {
    uint8_t step;
    uint32_t wait_start_us, wait_us;
    bool done, failed;
    uint8_t fault_summary; /* FAULT_SUMMARY read at the end */
} drv8244_init;

void drv8244_init_start(drv8244_init *ini, uint32_t now_us, uint32_t wake_wait_us);

/* Next frame to send, if any. last_sdo is the response to the previous frame. Runs with
   DRVOFF high: CLR_FLT re-enables the outputs. Unlocks CONFIG first because an MCU-only
   reset leaves the device locked from the previous run, and reads both CONFIG registers
   back. */
bool drv8244_init_next(drv8244_init *ini, uint32_t now_us, uint16_t last_sdo, uint8_t config3,
                       uint8_t config4, uint16_t *frame_out);

#endif
