#include "board/clock_config.h"
#include "board/reset_cause.h"
#include "fw/drive.h"
#include "fw_test.h"

CLK_STATIC_ASSERTS;

int main(void)
{
    /* Power-on sets POR, BOR and the pin flag together. */
    CHECK_EQ_U(reset_cause_from_rsr(RSR_PORRSTF | RSR_BORRSTF | RSR_PINRSTF), DRIVE_RESET_POWER);
    CHECK_EQ_U(reset_cause_from_rsr(RSR_BORRSTF | RSR_PINRSTF), DRIVE_RESET_POWER);
    /* The pin flag accompanies the IWDG and software resets: they win. */
    CHECK_EQ_U(reset_cause_from_rsr(RSR_IWDG1RSTF | RSR_PINRSTF), DRIVE_RESET_IWDG);
    CHECK_EQ_U(reset_cause_from_rsr(RSR_SFTRSTF | RSR_PINRSTF), DRIVE_RESET_SOFTWARE);
    CHECK_EQ_U(reset_cause_from_rsr(RSR_IWDG1RSTF | RSR_SFTRSTF | RSR_PINRSTF), DRIVE_RESET_IWDG);
    CHECK_EQ_U(reset_cause_from_rsr(RSR_IWDG1RSTF | RSR_PORRSTF), DRIVE_RESET_IWDG);
    /* The pin alone is the reset button / NRST. */
    CHECK_EQ_U(reset_cause_from_rsr(RSR_PINRSTF), DRIVE_RESET_PIN);
    /* Nothing recognised (e.g. only domain or WWDG flags). */
    CHECK_EQ_U(reset_cause_from_rsr(0), DRIVE_RESET_OTHER);
    CHECK_EQ_U(reset_cause_from_rsr(1u << 28), DRIVE_RESET_OTHER);
    /* Clock tree: both PLL sources give 400 MHz SYSCLK, 100 MHz PLL1Q, 1 MHz TIM2. */
    CHECK_EQ_U(CLK_SYSCLK_HZ, 400000000u);
    CHECK_EQ_U(CLK_PLL1Q_HZ, 100000000u);
    CHECK_EQ_U(CLK_US_PRESCALER, 200u);
    return FW_TEST_RESULT();
}
