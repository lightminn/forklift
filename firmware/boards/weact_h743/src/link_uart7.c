/* UART7 transport (board design 5, FW_LINK=uart7): PE7 RX / PE8 TX (AF7), 460800 bps 8N1, FIFO
   mode. Both directions run in the UART7 interrupt at priority 1, above the control tick: the
   16-byte receive FIFO lasts 350 us at this rate, shorter than a tick may take. The tick puts a
   frame in the send ring and enables the TX-FIFO-not-full interrupt; the interrupt moves bytes
   and disables it again when the ring is empty. The tick's read-modify-write of CR1 can be
   interrupted by that disable; it then writes the enable back and one interrupt finds the ring
   empty, which is harmless. A UART has no "port open" signal: frames go out as they are made,
   and a host that does not read loses them on the wire rather than finding them queued. */
#include "board/board.h"
#include "board/clock_config.h"
#include "board/gpio.h"
#include "board/link.h"
#include "board/link_cfg.h"
#include "stm32h7xx.h"

#define UART_IRQ_PRIORITY 1u

link_stats g_link_stats;

/* Valid from reset: the control tick may run before link_init. */
static uint8_t g_rx_mem[LINK_RX_RING_SIZE];
static uint8_t g_tx_mem[LINK_TX_RING_SIZE];
static link_ring g_rx = {0, 0, g_rx_mem, LINK_RX_RING_SIZE - 1u};
static link_ring g_tx = {0, 0, g_tx_mem, LINK_TX_RING_SIZE - 1u};
static volatile uint32_t g_ready; /* UART7 is clocked and set up */

void link_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    RCC->APB1LENR |= RCC_APB1LENR_UART7EN; /* kernel clock rcc_pclk1 (D2CCIP2R reset value) */
    (void)RCC->APB1LENR;
    gpio_af(GPIOE, 7, 7, GPIO_SPEED_MEDIUM);         /* UART7_RX */
    gpio_mode(GPIOE, 7, GPIO_MODE_AF, GPIO_PULL_UP); /* idle high when nothing is wired */
    gpio_af(GPIOE, 8, 7, GPIO_SPEED_MEDIUM);         /* UART7_TX */
    UART7->CR1 = 0;
    UART7->BRR = uart_brr(CLK_PCLK_HZ, LINK_UART_BAUD);
    UART7->CR2 = 0;                                              /* 1 stop bit */
    UART7->CR3 = 0;                                              /* no flow control */
    UART7->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE; /* FIFOEN only while UE = 0 */
    UART7->CR1 |= USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
    NVIC_SetPriority(UART7_IRQn, UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(UART7_IRQn);
    g_ready = 1;
}

void UART7_IRQHandler(void)
{
    const uint32_t errors = USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE;
    if (UART7->ISR & errors) {
        UART7->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        g_link_stats.uart_errors++;
    }
    while (UART7->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t b = (uint8_t)UART7->RDR;
        if (link_ring_put_some(&g_rx, &b, 1) == 0) {
            g_link_stats.rx_dropped++; /* the decoder resynchronises on the next delimiter */
        }
    }
    if (UART7->CR1 & USART_CR1_TXEIE_TXFNFIE) {
        while (UART7->ISR & USART_ISR_TXE_TXFNF) {
            const uint8_t *p;
            if (link_ring_peek(&g_tx, &p) == 0) {
                UART7->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
                break;
            }
            UART7->TDR = *p;
            link_ring_consume(&g_tx, 1);
        }
    }
}

void link_poll(void) {}

size_t link_read(uint8_t *buf, size_t cap)
{
    return link_ring_get(&g_rx, buf, (uint32_t)cap);
}

bool link_write_frame(const uint8_t *frame, size_t len)
{
    if (!g_ready || len > LINK_TX_RING_SIZE || !link_ring_put_frame(&g_tx, frame, (uint32_t)len)) {
        return false;
    }
    UART7->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return true;
}
