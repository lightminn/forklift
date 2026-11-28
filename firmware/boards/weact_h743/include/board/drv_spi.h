#ifndef BOARD_DRV_SPI_H
#define BOARD_DRV_SPI_H

#include <stdbool.h>
#include <stdint.h>

void drv_spi_init(void);
/* fw_hal.drv_spi_start: false if busy or an answer is still untaken. */
bool drv_spi_begin(uint8_t ch, uint16_t tx, uint32_t tag);
/* fw_hal.drv_spi_take. */
bool drv_spi_collect(uint8_t *ch, uint16_t *rx, uint32_t *tag);

void kth_spi_init(void);
/* Starts one steering-then-lift read (board design 4.6); does nothing while one is running. */
void kth_spi_start_bundle(void);
/* Latest frames; fresh[i] is true once per completed read (generations, board design 4.5). */
void kth_spi_snapshot(uint32_t now, uint16_t frame[2], bool fresh[2], uint32_t time[2]);
#if defined(FW_BENCH_IMAGE)
/* bench7: each frame's bundle start (plan 4.3 A1b-2 decision 6), as kth_spi_snapshot left them */
void kth_spi_starts(uint32_t start[2]);
#endif

#endif
