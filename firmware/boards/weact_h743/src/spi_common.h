/* One 16-bit SPI master transaction per enable (board design 4.2, 4.6). Following RM0433 50.4.12
   and the device erratum "master data transfer stall at system clock much faster than SCK"
   (ES0396 2.12.2 / ES0392, same die): the SPI is disabled after every EOT and enabled again for
   the next transaction; the EOT interrupt is enabled only while the SPI is enabled (ES0396 2.12.4).
   TSIZE is 1 and is written only while the SPI is disabled. */
#ifndef BOARD_SPI_COMMON_H
#define BOARD_SPI_COMMON_H

#include <stddef.h>
#include <stdint.h>

#include "board/spi_cfg.h"
#include "stm32h7xx.h"

_Static_assert(SPI_CFG1_MBR_POS == SPI_CFG1_MBR_Pos &&
                   SPI_CFG1_CRCSIZE_POS == SPI_CFG1_CRCSIZE_Pos &&
                   SPI_CFG1_DSIZE_POS == SPI_CFG1_DSIZE_Pos,
               "SPI_CFG1 field positions");
_Static_assert(SPI_CR1_SSI_POS == SPI_CR1_SSI_Pos && SPI_CFG2_MASTER_POS == SPI_CFG2_MASTER_Pos &&
                   SPI_CFG2_CPHA_POS == SPI_CFG2_CPHA_Pos && SPI_CFG2_CPOL_POS == SPI_CFG2_CPOL_Pos &&
                   SPI_CFG2_SSM_POS == SPI_CFG2_SSM_Pos &&
                   SPI_CFG2_AFCNTR_POS == SPI_CFG2_AFCNTR_Pos,
               "SPI_CR1/SPI_CFG2 bit positions");

/* The data registers take one 16-bit frame per access; address them as 16-bit MMIO. */
static inline volatile uint16_t *spi_reg16(SPI_TypeDef *spi, size_t offset)
{
    return (volatile uint16_t *)((uintptr_t)spi + offset);
}

/* SPI_CFG1.MBR value for a divider (2 << mbr). */
#define SPI_MBR_DIV16 3u
#define SPI_MBR_DIV32 4u

/* Applies spi_setup_writes() in its order (SSI before MASTER: see spi_cfg.h). */
static inline void spi_setup(SPI_TypeDef *spi, uint32_t mbr, uint32_t cpol, uint32_t cpha)
{
    spi_setup_write w[SPI_SETUP_WRITES];
    spi_setup_writes(mbr, cpol, cpha, w);
    for (unsigned i = 0; i < SPI_SETUP_WRITES; i++) {
        switch (w[i].reg) {
        case SPI_SETUP_CR1:
            spi->CR1 = w[i].value;
            break;
        case SPI_SETUP_CFG1:
            spi->CFG1 = w[i].value;
            break;
        case SPI_SETUP_CFG2:
            spi->CFG2 = w[i].value;
            break;
        case SPI_SETUP_CR2:
            spi->CR2 = w[i].value;
            break;
        default:
            break;
        }
    }
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
