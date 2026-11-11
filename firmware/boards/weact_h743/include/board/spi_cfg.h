/* SPI_CFG1 value for the board's 16-bit master links. Pure (no device header) so the host tests
   check it; spi_common.h asserts the bit positions against CMSIS in the MCU build. */
#ifndef BOARD_SPI_CFG_H
#define BOARD_SPI_CFG_H

#include <stdint.h>

#define SPI_CFG1_MBR_POS 28u
#define SPI_CFG1_CRCSIZE_POS 16u
#define SPI_CFG1_DSIZE_POS 0u
/* RM0433 SPI_CFG1: keep CRCSIZE at its reset value (8-bit) while CRCEN = 0. */
#define SPI_CFG1_CRCSIZE_RESET 7u

/* mbr: divider 2 << mbr; 16-bit frames; CRC off with CRCSIZE at its reset value (Codex B2a review:
   writing 0 there is the erratum's corrupted-last-data condition). */
static inline uint32_t spi_cfg1_value(uint32_t mbr)
{
    return (mbr << SPI_CFG1_MBR_POS) | (SPI_CFG1_CRCSIZE_RESET << SPI_CFG1_CRCSIZE_POS) |
           (15u << SPI_CFG1_DSIZE_POS);
}

#endif
