/* One 16-bit SPI master transaction per enable (board design 4.2, 4.6). Following RM0433 50.4.12
   and the device erratum "master data transfer stall at system clock much faster than SCK"
   (ES0396 2.12.2 / ES0392, same die): the SPI is disabled after every EOT and enabled again for
   the next transaction; the EOT interrupt is enabled only while the SPI is enabled (ES0396 2.12.4).
   TSIZE is 1 and is written only while the SPI is disabled. */
#ifndef BOARD_SPI_COMMON_H
#define BOARD_SPI_COMMON_H

#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx.h"

/* The data registers take one 16-bit frame per access; address them as 16-bit MMIO. */
static inline volatile uint16_t *spi_reg16(SPI_TypeDef *spi, size_t offset)
{
    return (volatile uint16_t *)((uintptr_t)spi + offset);
}

/* SPI_CFG1.MBR value for a divider (2 << mbr). */
#define SPI_MBR_DIV16 3u
#define SPI_MBR_DIV32 4u

static inline void spi_setup(SPI_TypeDef *spi, uint32_t mbr, uint32_t cpol, uint32_t cpha)
{
    spi->CR1 = 0;
    spi->CFG1 = (mbr << SPI_CFG1_MBR_Pos) | (15u << SPI_CFG1_DSIZE_Pos); /* 16-bit frames */
    /* Master, software NSS held high internally, outputs kept driven while disabled (AFCNTR). */
    spi->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | (cpol ? SPI_CFG2_CPOL : 0u) |
                (cpha ? SPI_CFG2_CPHA : 0u);
    spi->CR1 = SPI_CR1_SSI;
    spi->CR2 = 1u; /* TSIZE = 1 */
}

static inline void spi_start(SPI_TypeDef *spi, uint16_t tx)
{
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->IER = SPI_IER_EOTIE;
    *spi_reg16(spi, offsetof(SPI_TypeDef, TXDR)) = tx; /* 16-bit access: one frame */
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_CSTART;
}

/* From the EOT interrupt: the received word; the SPI ends disabled with its interrupt off. Any
   error flag (overrun, mode fault, underrun) makes ok false. */
static inline uint16_t spi_finish(SPI_TypeDef *spi, int *ok)
{
    uint32_t sr = spi->SR;
    uint16_t rx = 0;
    if (sr & SPI_SR_RXP) {
        rx = *spi_reg16(spi, offsetof(SPI_TypeDef, RXDR));
    }
    *ok = (sr & SPI_SR_EOT) && (sr & SPI_SR_RXP) && !(sr & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR));
    spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_UDRC;
    spi->IER = 0;
    spi->CR1 = SPI_CR1_SSI; /* SPE = 0: flushes the FIFOs */
    return rx;
}

#endif
