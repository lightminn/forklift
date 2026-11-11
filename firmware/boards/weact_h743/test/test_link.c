/* Link rings and transport arithmetic (board design 5), on the host. */
#include <string.h>

#include "board/clock_config.h"
#include "board/link_cfg.h"
#include "board/link_ring.h"
#include "fw_test.h"

CLK_STATIC_ASSERTS;

static void frames_go_in_whole_or_not_at_all(void)
{
    uint8_t mem[16];
    link_ring r;
    link_ring_init(&r, mem, sizeof mem);
    const uint8_t a[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    CHECK(link_ring_put_frame(&r, a, 10));
    CHECK(!link_ring_put_frame(&r, a, 7)); /* 6 free: refused, nothing written */
    CHECK_EQ_U(r.head, 10);
    CHECK(link_ring_put_frame(&r, a, 6));
    uint8_t out[32];
    CHECK_EQ_U(link_ring_get(&r, out, sizeof out), 16);
    CHECK(memcmp(out, a, 10) == 0 && memcmp(out + 10, a, 6) == 0);
    CHECK_EQ_U(link_ring_get(&r, out, sizeof out), 0);
}

/* The counters run free through 2^32 and the copy wraps around the buffer end. */
static void counters_wrap(void)
{
    uint8_t mem[8];
    link_ring r;
    link_ring_init(&r, mem, sizeof mem);
    r.head = r.tail = 0xFFFFFFFCu;
    const uint8_t a[6] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
    CHECK(link_ring_put_frame(&r, a, 6));
    CHECK_EQ_U(r.head, 2u);
    CHECK_EQ_U(link_ring_space(&r), 2);
    const uint8_t *p;
    CHECK_EQ_U(link_ring_peek(&r, &p), 4); /* up to the buffer end */
    CHECK(p == mem + 4);
    uint8_t out[8];
    CHECK_EQ_U(link_ring_get(&r, out, 3), 3);
    CHECK_EQ_U(link_ring_get(&r, out + 3, 8), 3);
    CHECK(memcmp(out, a, 6) == 0);
}

/* A frame the producer is still copying is invisible until head moves; drop_all leaves it. */
static void unpublished_bytes_are_invisible(void)
{
    uint8_t mem[16];
    link_ring r;
    link_ring_init(&r, mem, sizeof mem);
    const uint8_t a[4] = {1, 2, 3, 4};
    CHECK(link_ring_put_frame(&r, a, 4));
    link_ring_copy_in(&r, r.head, a, 3); /* producer stopped before publishing */
    const uint8_t *p;
    CHECK_EQ_U(link_ring_peek(&r, &p), 4);
    CHECK_EQ_U(link_ring_drop_all(&r), 4);
    CHECK_EQ_U(link_ring_peek(&r, &p), 0);
    r.head = r.head + 3; /* published now */
    CHECK_EQ_U(link_ring_peek(&r, &p), 3);
}

static void put_some_takes_what_fits(void)
{
    uint8_t mem[8];
    link_ring r;
    link_ring_init(&r, mem, sizeof mem);
    const uint8_t a[12] = {0};
    CHECK_EQ_U(link_ring_put_some(&r, a, 12), 8);
    CHECK_EQ_U(link_ring_put_some(&r, a, 1), 0);
}

/* A sink standing in for the CDC stack. */
typedef struct {
    bool connected;
    uint32_t room;
    uint8_t got[64];
    uint32_t n;
    int flushes;
} fake_sink;

static bool fs_connected(void *c)
{
    return ((fake_sink *)c)->connected;
}

static uint32_t fs_room(void *c)
{
    return ((fake_sink *)c)->room;
}

static uint32_t fs_write(void *c, const uint8_t *p, uint32_t n)
{
    fake_sink *f = c;
    if (n > f->room) {
        n = f->room;
    }
    memcpy(f->got + f->n, p, n);
    f->n += n;
    f->room -= n;
    return n;
}

static void fs_flush(void *c)
{
    ((fake_sink *)c)->flushes++;
}

static void tx_pump(void)
{
    uint8_t mem[16];
    link_ring r;
    link_ring_init(&r, mem, sizeof mem);
    fake_sink f = {.connected = false, .room = 64};
    link_sink s = {&f, fs_connected, fs_room, fs_write, fs_flush};
    bool resync = false;
    const uint8_t old[5] = {9, 9, 9, 9, 0};
    CHECK(link_ring_put_frame(&r, old, 5));
    /* Nobody has the port open: the frame is dropped, nothing reaches the stack, and the next
       opening must resynchronise. The stack's own FIFO is never cleared: with the DWC2 slave
       port an IN transfer reads it in place (Codex B3 review 2). */
    CHECK_EQ_U(link_tx_pump(&r, &s, &resync), 5);
    CHECK_EQ_U(f.n, 0);
    CHECK_EQ_U(f.flushes, 0);
    CHECK(resync);
    /* Port open: one delimiter first, then only frames made since then, in order, across the
       buffer end and across a full stack. */
    f.connected = true;
    const uint8_t a[6] = {1, 2, 3, 4, 5, 0};
    const uint8_t b[6] = {6, 7, 8, 9, 10, 0};
    CHECK(link_ring_put_frame(&r, a, 6));
    CHECK(link_ring_put_frame(&r, b, 6));
    f.room = 9;
    CHECK_EQ_U(link_tx_pump(&r, &s, &resync), 0);
    CHECK(!resync);
    CHECK_EQ_U(f.n, 9);
    CHECK_EQ_U(f.got[0], 0x00);
    CHECK_EQ_U(f.flushes, 1);
    CHECK_EQ_U(link_tx_pump(&r, &s, &resync), 0); /* stack full: nothing moves, no flush */
    CHECK_EQ_U(f.flushes, 1);
    f.room = 64;
    CHECK_EQ_U(link_tx_pump(&r, &s, &resync), 0);
    CHECK_EQ_U(f.n, 13);
    CHECK(memcmp(f.got + 1, a, 6) == 0 && memcmp(f.got + 7, b, 6) == 0);
}

/* A life-cycle event the poll never saw as "closed" (resume with DTR held, a reset and
   re-enumeration between two polls) sets resync from the transport's callbacks: the delimiter
   still goes first. With the stack full the delimiter waits, and no frame overtakes it. */
static void tx_resync_from_callback(void)
{
    uint8_t mem[16];
    link_ring r;
    link_ring_init(&r, mem, sizeof mem);
    fake_sink f = {.connected = true, .room = 0};
    link_sink s = {&f, fs_connected, fs_room, fs_write, fs_flush};
    bool resync = true; /* set by tud_resume_cb */
    const uint8_t a[4] = {1, 2, 3, 0};
    CHECK(link_ring_put_frame(&r, a, 4));
    CHECK_EQ_U(link_tx_pump(&r, &s, &resync), 0);
    CHECK(resync);
    CHECK_EQ_U(f.n, 0);
    f.room = 64;
    CHECK_EQ_U(link_tx_pump(&r, &s, &resync), 0);
    CHECK(!resync);
    CHECK(f.n == 5 && f.got[0] == 0x00 && memcmp(f.got + 1, a, 4) == 0);
}

static void uart_divider(void)
{
    /* UART7 kernel clock = rcc_pclk1 (D2CCIP2R reset value) = 100 MHz. */
    CHECK_EQ_U(uart_brr(CLK_PCLK_HZ, LINK_UART_BAUD), 217);
    int32_t e = uart_baud_error_ppm(CLK_PCLK_HZ, LINK_UART_BAUD);
    CHECK(e > -1000 && e < 1000); /* +-0.1 %: far inside 8N1's tolerance */
}

static void usb_strings(void)
{
    const uint32_t uid[3] = {0x0123ABCDu, 0x00000000u, 0xFFFFFFFFu};
    char s[25];
    usb_serial_hex(uid, s);
    CHECK(strcmp(s, "0123ABCD00000000FFFFFFFF") == 0);

    uint16_t d[130];
    CHECK_EQ_U(usb_string_desc("ab", d, 130), 2);
    CHECK_EQ_U(d[0], 0x0306u);
    CHECK_EQ_U(d[1], 'a');
    char longer[200];
    memset(longer, 'x', sizeof longer - 1);
    longer[sizeof longer - 1] = '\0';
    CHECK_EQ_U(usb_string_desc(longer, d, 130), 126); /* 255-byte descriptor limit */
    CHECK_EQ_U(d[0] & 0xFFu, 254u);
    CHECK_EQ_U(usb_string_desc(longer, d, 10), 9); /* the caller's buffer */
}

static void usb_clock(void)
{
    CHECK_EQ_U(CLK_USB_HZ, 48000000u);
    CHECK_EQ_U(CLK_HSE_HZ / CLK_PLL_HSE_M * CLK_PLL3_HSE_N / CLK_PLL3_Q, 48000000u);
    CHECK_EQ_U(CLK_HSI_HZ / CLK_PLL_HSI_M * CLK_PLL3_HSI_N / CLK_PLL3_Q, 48000000u);
}

int main(void)
{
    frames_go_in_whole_or_not_at_all();
    counters_wrap();
    unpublished_bytes_are_invisible();
    put_some_takes_what_fits();
    tx_pump();
    tx_resync_from_callback();
    uart_divider();
    usb_strings();
    usb_clock();
    return FW_TEST_RESULT();
}
