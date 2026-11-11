/* USB CDC transport (board design 5, FW_LINK=usb): TinyUSB on USB2 OTG FS, PA11 DM / PA12 DP.

   Contexts: the OTG interrupt (priority 3, below the control tick) only queues events; tud_task,
   the receive copy and the send pump run in the main loop. The control tick touches the rings
   only. TinyUSB waits without a bound in a few places (core reset, endpoint disable); all of them
   run in the main loop or at priority 3, so the control tick, the command expiry and the IWDG
   keep running past them: a stuck USB stack silences the link and the vehicle stops on expiry.

   Stale frames: while no host has the port open (DTR low), frames are dropped as they come; on
   a DTR rising edge whatever is left in the send ring and in TinyUSB's FIFO is dropped too, so a
   reopened port starts with fresh STATE frames (board design 5). The receive side keeps bytes in
   TinyUSB's FIFO (the endpoint NAKs) when the ring is full: nothing is lost there. */
#include "board/board.h"
#include "board/clock_config.h"
#include "board/gpio.h"
#include "board/link.h"
#include "board/link_cfg.h"
#include "stm32h7xx.h"
#include "tusb.h"

#define USB_IRQ_PRIORITY 3u

link_stats g_link_stats;

/* TinyUSB's STM32 port reads the CMSIS clock variable (USB turnaround time, remote wakeup
   delay). The startup code does not use the ST system file, so it is defined here, fixed: the
   clock tree is the same from either PLL source (clock_config.h). */
uint32_t SystemCoreClock = CLK_SYSCLK_HZ;

/* Valid from reset: the control tick may run before link_init. */
static uint8_t g_rx_mem[LINK_RX_RING_SIZE];
static uint8_t g_tx_mem[LINK_TX_RING_SIZE];
static link_ring g_rx = {0, 0, g_rx_mem, LINK_RX_RING_SIZE - 1u};
static link_ring g_tx = {0, 0, g_tx_mem, LINK_TX_RING_SIZE - 1u};
static bool g_dtr;

/* TinyUSB's millisecond clock (OPT_OS_NONE), main context only: the microsecond counter's
   wrap is carried so the milliseconds never step back. */
uint32_t tusb_time_millis_api(void)
{
    static uint32_t last_us, ms, rem_us;
    uint32_t now = timebase_now_us();
    uint32_t d = now - last_us + rem_us;
    last_us = now;
    ms += d / 1000u;
    rem_us = d % 1000u;
    return ms;
}

void link_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    (void)RCC->AHB4ENR;
    gpio_af(GPIOA, 11, 10, GPIO_SPEED_HIGH); /* OTG_FS_DM, AF10 */
    gpio_af(GPIOA, 12, 10, GPIO_SPEED_HIGH); /* OTG_FS_DP, AF10 */
    if (!clock_usb_init()) {
        return; /* no USB clock: the link stays silent, ARM never arrives */
    }
    /* VDD33USB comes from the board's 3.3 V: its level detector must be on before the
       transceiver is used (RM0433 6.4.4). */
    PWR->CR3 |= PWR_CR3_USB33DEN;
    if (!cycles_wait_flag(&PWR->CR3, PWR_CR3_USB33RDY, CLK_SYSCLK_HZ / 100u)) {
        return;
    }
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(OTG_FS_IRQn, USB_IRQ_PRIORITY); /* TinyUSB enables it in dcd_init */
    const tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    if (tusb_init(BOARD_TUD_RHPORT, &init)) {
        g_link_stats.usb_started = 1;
    }
}

void OTG_FS_IRQHandler(void)
{
    tud_int_handler(BOARD_TUD_RHPORT);
}

/* DTR edge, called from tud_task (main loop, the send ring's consumer). */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;
    if (dtr && !g_dtr) {
        g_link_stats.reopened++;
        g_link_stats.tx_dropped_idle += link_ring_drop_all(&g_tx);
        (void)tud_cdc_write_clear();
    }
    g_dtr = dtr;
}

static bool sink_connected(void *ctx)
{
    (void)ctx;
    return tud_cdc_connected();
}

static uint32_t sink_room(void *ctx)
{
    (void)ctx;
    return tud_cdc_write_available();
}

static uint32_t sink_write(void *ctx, const uint8_t *p, uint32_t n)
{
    (void)ctx;
    return tud_cdc_write(p, n);
}

static void sink_flush(void *ctx)
{
    (void)ctx;
    (void)tud_cdc_write_flush();
}

static const link_sink g_sink = {0, sink_connected, sink_room, sink_write, sink_flush};

void link_poll(void)
{
    if (!g_link_stats.usb_started) {
        return;
    }
    tud_task();
    uint8_t buf[CFG_TUD_CDC_EP_BUFSIZE];
    for (;;) {
        uint32_t n = link_ring_space(&g_rx);
        uint32_t avail = tud_cdc_available();
        if (n > avail) {
            n = avail;
        }
        if (n > sizeof buf) {
            n = sizeof buf;
        }
        if (n == 0) {
            break;
        }
        uint32_t got = tud_cdc_read(buf, n);
        (void)link_ring_put_some(&g_rx, buf, got);
        if (got < n) {
            break;
        }
    }
    g_link_stats.tx_dropped_idle += link_tx_pump(&g_tx, &g_sink);
}

size_t link_read(uint8_t *buf, size_t cap)
{
    return link_ring_get(&g_rx, buf, (uint32_t)cap);
}

bool link_write_frame(const uint8_t *frame, size_t len)
{
    return len <= LINK_TX_RING_SIZE && link_ring_put_frame(&g_tx, frame, (uint32_t)len);
}
