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
void kth_spi_snapshot(uint16_t frame[2], bool fresh[2]);

#endif
