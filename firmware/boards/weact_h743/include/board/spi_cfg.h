/* SPI_CFG1 value and the setup write sequence for the board's 16-bit master links. Pure (no device
   header) so the host tests check them; spi_common.h asserts the bit positions against CMSIS in the
   MCU build. */
#ifndef BOARD_SPI_CFG_H
#define BOARD_SPI_CFG_H

#include <stdint.h>

#define SPI_CFG1_MBR_POS 28u
#define SPI_CFG1_CRCSIZE_POS 16u
#define SPI_CFG1_DSIZE_POS 0u
/* RM0433 SPI_CFG1: keep CRCSIZE at its reset value (8-bit) while CRCEN = 0. */
#define SPI_CFG1_CRCSIZE_RESET 7u

#define SPI_CR1_SSI_POS 12u
#define SPI_CFG2_MASTER_POS 22u
#define SPI_CFG2_CPHA_POS 24u
#define SPI_CFG2_CPOL_POS 25u
#define SPI_CFG2_SSM_POS 26u
#define SPI_CFG2_AFCNTR_POS 31u
#define SPI_CR1_SSI_BIT (1u << SPI_CR1_SSI_POS)
#define SPI_CFG2_MASTER_BIT (1u << SPI_CFG2_MASTER_POS)
#define SPI_CFG2_CPHA_BIT (1u << SPI_CFG2_CPHA_POS)
#define SPI_CFG2_CPOL_BIT (1u << SPI_CFG2_CPOL_POS)
#define SPI_CFG2_SSM_BIT (1u << SPI_CFG2_SSM_POS)
#define SPI_CFG2_AFCNTR_BIT (1u << SPI_CFG2_AFCNTR_POS)

/* mbr: divider 2 << mbr; 16-bit frames; CRC off with CRCSIZE at its reset value (Codex B2a review:
   writing 0 there is the erratum's corrupted-last-data condition). */
static inline uint32_t spi_cfg1_value(uint32_t mbr)
{
    return (mbr << SPI_CFG1_MBR_POS) | (SPI_CFG1_CRCSIZE_RESET << SPI_CFG1_CRCSIZE_POS) |
           (15u << SPI_CFG1_DSIZE_POS);
}

/* The master setup as an ordered list of register writes, so the host tests check the order.
   SSI goes first: with SSM = 1 the internal NSS is SSI, and per RM0433 (SPI mode fault) a master
   whose NSS reads low can take a mode fault that sets MODF, clears MASTER and blocks SPE until
   MODFC; raising SSI later does not undo it. ST's HAL sets SSI before MASTER too. Observed on the
   first board (2026-11-24): with the old order (CR1 = 0, CFG1, CFG2 = MASTER | SSM, then SSI)
   neither SPI1 (DRV8244) nor SPI2 (KTH7801) ever completed a transfer; with SSI first both did.
   The mode fault itself was not measured -- it is the RM0433 behaviour that matches. All writes
   happen with SPE = 0. */
enum spi_setup_reg { SPI_SETUP_CR1, SPI_SETUP_CFG1, SPI_SETUP_CFG2, SPI_SETUP_CR2 };
typedef struct {
    uint8_t reg; /* enum spi_setup_reg */
    uint32_t value;
} spi_setup_write;
#define SPI_SETUP_WRITES 4u

/* Master, software NSS held high internally, outputs kept driven while disabled (AFCNTR), 16-bit
   frames, one frame per enable (TSIZE = 1). */
static inline void spi_setup_writes(uint32_t mbr, uint32_t cpol, uint32_t cpha,
                                    spi_setup_write w[SPI_SETUP_WRITES])
{
    w[0] = (spi_setup_write){SPI_SETUP_CR1, SPI_CR1_SSI_BIT};
    w[1] = (spi_setup_write){SPI_SETUP_CFG1, spi_cfg1_value(mbr)};
    w[2] = (spi_setup_write){SPI_SETUP_CFG2, SPI_CFG2_MASTER_BIT | SPI_CFG2_SSM_BIT |
                                                 SPI_CFG2_AFCNTR_BIT |
                                                 (cpol ? SPI_CFG2_CPOL_BIT : 0u) |
                                                 (cpha ? SPI_CFG2_CPHA_BIT : 0u)};
    w[3] = (spi_setup_write){SPI_SETUP_CR2, 1u};
}

#endif
