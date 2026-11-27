/* USB CDC transport (board design 5, FW_LINK=usb): TinyUSB on USB2 OTG FS, PA11 DM / PA12 DP.

   Contexts: the OTG interrupt (priority 3, below the control tick) only queues events; tud_task,
   the receive copy and the send pump run in the main loop. The control tick touches the rings
   only. TinyUSB waits without a bound in a few places (core reset, endpoint disable); all of them
   run in the main loop or at priority 3, so the control tick, the command expiry and the IWDG
   keep running past them: a stuck USB stack silences the link and the vehicle stops on expiry.

   Stale frames: while no host has the port open, frames are dropped as they come; when it is open
   again a delimiter goes first, so a reopened port sees at most TinyUSB's FIFO and one packet in
   the controller of old bytes, closed as one broken frame, then fresh STATE frames (board
   design 5.1, link_tx_pump). Every connection life-cycle callback marks the next opening for that
   delimiter too, so a resume with DTR held or a reset between two polls is not missed. TinyUSB's
   FIFO is never cleared. The receive side keeps bytes in TinyUSB's FIFO (the endpoint NAKs) when
   the ring is full: nothing is lost there. */
#include "board/board.h"
#include "board/clock_config.h"
#include "board/gpio.h"
#include "board/link.h"
#include "board/link_cfg.h"
#include "stm32h7xx.h"
#include "tusb.h"
#if defined(FW_BENCH_IMAGE)
#include "device/dcd.h" /* dcd_int_disable/enable around the receive FIFO clear */
#endif

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
static bool g_resync = true; /* main loop only (tud_task callbacks and link_poll) */

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
    /* The VDD33USB level detector must be on before the transceiver is used (RM0433 6.4.4); the
       USB regulator stays off (USBREGEN reset value). On LQFP100 VDD33USB is not a pin: it is
       bonded to VDD (DS12110 Rev 7 revision history, figure 5), and ST reports USB33RDY does not
       rise in that supply configuration, so it is recorded, never waited on (Codex B3 review). */
    PWR->CR3 |= PWR_CR3_USB33DEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(OTG_FS_IRQn, USB_IRQ_PRIORITY); /* TinyUSB enables it in dcd_init */
    const tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    if (tusb_init(BOARD_TUD_RHPORT, &init)) {
        g_link_stats.usb_started = 1;
    }
    g_link_stats.usb33rdy = (PWR->CR3 & PWR_CR3_USB33RDY) != 0u;
}

#if defined(FW_BENCH_IMAGE)
static void conn_event_from_isr(void);
static volatile bool g_otg_in_isr; /* the tick (priority 2) may preempt this interrupt (3) mid-way */
static volatile uint32_t g_otg_isr_seq; /* bumped at every OTG interrupt entry: tells one held-up call from two */

static uint32_t usb_dev_addr(void)
{
    const USB_OTG_DeviceTypeDef *dev = (const USB_OTG_DeviceTypeDef *)(USB2_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
    return dev->DCFG & USB_OTG_DCFG_DAD_Msk;
}
#endif

void OTG_FS_IRQHandler(void)
{
#if defined(FW_BENCH_IMAGE)
    /* A bus reset handled in this interrupt zeroes the device address (TinyUSB handle_bus_reset): an address that
       was set before and is 0 after is a connection change, counted here whatever TinyUSB's event queue does
       (Codex A1a rounds 5-6). While this runs, the tick reads no byte (link_read). */
    g_otg_isr_seq++;
    g_otg_in_isr = true;
    if ((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) != 0u) {
        conn_event_from_isr(); /* before TinyUSB clears it and waits on the FIFO flushes (round 8) */
    }
    uint32_t addr_before = usb_dev_addr();
    tud_int_handler(BOARD_TUD_RHPORT);
    if (addr_before != 0u && usb_dev_addr() == 0u) {
        conn_event_from_isr();
    }
    g_otg_in_isr = false;
#else
    tud_int_handler(BOARD_TUD_RHPORT);
#endif
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

/* Connection life cycle, called from tud_task (main loop, the send ring's consumer): each drops
   the ring and marks the next opening for a resynchronising delimiter (link_tx_pump), so the
   only old bytes a reopened port sees are TinyUSB's FIFO plus one packet in the controller's TX
   FIFO, even when the poll never saw the port closed (Codex B3 review 3). */
#if defined(FW_BENCH_IMAGE)
/* Bench image (contract 8): the connection generation. The main loop bumps begin first, empties TinyUSB's
   receive FIFO, then settles end; the control tick (TIM6, which preempts the main loop) reads nothing while
   they differ and, once settled on a new value, drops what the receive ring still holds. The send FIFO is
   left alone: a DWC2 IN transfer reads it in place (board design 5.1). */
static volatile uint32_t g_conn_begin, g_conn_end;
/* Each context counts changes in its own counter and stamps its own start time, single writer each (Codex A1a
   rounds 2-8): the OTG interrupt (bus reset, unplug, suspend), the tick (a USBRST still pending, or the OTG
   interrupt seen mid-way two ticks running) and the main loop's lifecycle (DTR, mount; it also settles the
   others). The tick takes the earliest start of whatever is pending. */
static volatile uint32_t g_conn_events, g_conn_handled, g_since_isr;
static volatile uint32_t g_conn_tick_events, g_conn_tick_handled, g_since_tick;
static volatile uint32_t g_since_main;    /* the earliest start of the change the lifecycle settles */
static volatile uint32_t g_conn_observed; /* the generation the tick last saw settled */
static bool g_last_dtr;

static uint32_t earlier(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0 ? a : b;
}

static void conn_event_from_isr(void)
{
    if (g_conn_events == g_conn_handled) {
        g_since_isr = timebase_now_us();
    }
    g_conn_events++;
}

static void conn_event_from_tick(void)
{
    if (g_conn_tick_events == g_conn_tick_handled) {
        g_since_tick = timebase_now_us();
    }
    g_conn_tick_events++;
}

void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr)
{
    (void)rhport;
    if (in_isr && (eventid == DCD_EVENT_BUS_RESET || eventid == DCD_EVENT_UNPLUGGED ||
                   eventid == DCD_EVENT_SUSPEND)) {
        conn_event_from_isr();
    }
}

/* Control tick only. */
uint32_t link_conn_gen(bool *busy, uint32_t *since_us)
{
    static uint8_t in_isr_ticks;
    static uint32_t in_isr_seq;
    if (g_link_stats.usb_started && (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) != 0u) {
        conn_event_from_tick();
    }
    /* An OTG interrupt is a few microseconds: one still mid-way at two ticks in a row is held up (e.g. in
       TinyUSB's reset FIFO flushes); the arming ends rather than waiting for it (round 8). */
    uint32_t seq = g_otg_isr_seq;
    if (!g_otg_in_isr) {
        in_isr_ticks = 0u;
    } else if (in_isr_ticks == 0u || seq != in_isr_seq) {
        in_isr_ticks = 1u; /* a new call (two short calls in a row are not one held-up call) */
    } else if (in_isr_ticks < 255u) {
        in_isr_ticks++;
    }
    in_isr_seq = seq;
    if (in_isr_ticks == 2u) {
        conn_event_from_tick();
    }
    bool isr_pending = g_conn_events != g_conn_handled;
    bool tick_pending = g_conn_tick_events != g_conn_tick_handled;
    uint32_t begin = g_conn_begin, end = g_conn_end;
    bool main_pending = begin != end;
    *busy = isr_pending || tick_pending || main_pending;
    uint32_t since = g_since_main;
    if (*busy) {
        uint32_t now = timebase_now_us();
        since = now;
        if (isr_pending) {
            since = earlier(since, g_since_isr);
        }
        if (tick_pending) {
            since = earlier(since, g_since_tick);
        }
        if (main_pending || end != g_conn_observed) { /* incl. a settled change the tick has not seen yet */
            since = earlier(since, g_since_main);
        }
    } else {
        g_conn_observed = end;
    }
    *since_us = since;
    return end;
}
#endif

static void lifecycle(void)
{
#if defined(FW_BENCH_IMAGE)
    /* With the OTG interrupt off: the event snapshot, the start stamp and the begin mark cannot interleave with
       an interrupt's own stamp (Codex A1a round 4), and the CDC receive FIFO, which that interrupt writes in
       place (DWC2 slave), is cleared on whole indices (round 1). */
    dcd_int_disable(0);
    uint32_t tick_events = g_conn_tick_events; /* the tick preempts this; a later count keeps busy */
    uint32_t events = g_conn_events;
    uint32_t start = timebase_now_us(); /* this change's start: the earliest of what it settles */
    if (tick_events != g_conn_tick_handled) {
        start = earlier(start, g_since_tick);
    }
    if (events != g_conn_handled) {
        start = earlier(start, g_since_isr);
    }
    if (g_conn_end != g_conn_observed) { /* the last settled change not yet seen by the tick: keep its start */
        start = earlier(start, g_since_main);
    }
    g_since_main = start;
    g_conn_begin = g_conn_end + 1u;
    /* The receive ring's consumer is the tick, which reads nothing while begin != end and cannot run inside this
       (it preempts the main loop, not the other way): dropping the old connection's bytes here is safe, and does
       not depend on the tick's own drain, which link_read may hold back while the OTG interrupt runs. */
    (void)link_ring_drop_all(&g_rx);
    if (tud_mounted()) { /* unmounted (reset, unplug): the class reset closed the stream; re-arming it would fail */
        tud_cdc_read_flush();
    }
    dcd_int_enable(0);
#endif
    g_link_stats.tx_dropped_idle += link_ring_drop_all(&g_tx);
    g_resync = true;
#if defined(FW_BENCH_IMAGE)
    g_conn_handled = events;
    g_conn_tick_handled = tick_events;
    g_conn_end = g_conn_begin;
#endif
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;
#if defined(FW_BENCH_IMAGE)
    /* Only a change of DTR is a connection change: TinyUSB calls this for every SET_CONTROL_LINE_STATE, and an
       RTS-only request or the same DTR again must not end a run (Codex A1a round 3). */
    if (dtr != g_last_dtr) {
        g_last_dtr = dtr;
        lifecycle(); /* first: the bench image marks the change before anything else */
        if (dtr) {
            g_link_stats.reopened++;
        }
    }
#else
    if (dtr) {
        g_link_stats.reopened++;
    }
    lifecycle();
#endif
}

void tud_mount_cb(void)
{
    lifecycle();
}

void tud_umount_cb(void)
{
    lifecycle();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    lifecycle();
}

void tud_resume_cb(void)
{
    lifecycle();
}

void link_poll(void)
{
    if (!g_link_stats.usb_started) {
        return;
    }
    tud_task();
#if defined(FW_BENCH_IMAGE)
    /* A USB bus reset clears the device state without an unmount callback (TinyUSB usbd), so the bench
       image also takes the loss of the configuration as a connection change (Codex A1a P1). */
    static bool was_mounted;
    bool mounted = tud_mounted();
    if ((was_mounted && !mounted) || g_conn_events != g_conn_handled ||
        g_conn_tick_events != g_conn_tick_handled) {
        lifecycle();
    }
    was_mounted = mounted;
#endif
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
    g_link_stats.tx_dropped_idle += link_tx_pump(&g_tx, &g_sink, &g_resync);
}

size_t link_read(uint8_t *buf, size_t cap)
{
#if defined(FW_BENCH_IMAGE)
    /* Not one byte while the OTG interrupt is mid-way (a reset it handles is counted when it returns) or while a
       USBRST waits for it; the USB registers only once the clock is on (RM0433). */
    if (g_otg_in_isr || !g_link_stats.usb_started || (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) != 0u) {
        return 0;
    }
#endif
    return link_ring_get(&g_rx, buf, (uint32_t)cap);
}

bool link_write_frame(const uint8_t *frame, size_t len)
{
    return len <= LINK_TX_RING_SIZE && link_ring_put_frame(&g_tx, frame, (uint32_t)len);
}
