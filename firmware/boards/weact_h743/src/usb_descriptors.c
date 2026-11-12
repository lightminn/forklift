/* USB descriptors (board design 5): one CDC-ACM port. VID/PID are ST's VCP pair; the host picks
   the device by serial number (the 96-bit UID) and product string (the build identity), so the
   ROM DFU device (0483:df11) is never mistaken for the link. */
#include "board/link_cfg.h"
#include "build_info.h"
#include "fw/link.h"
#include "stm32h7xx.h"
#include "tusb.h"

#define USB_VID 0x0483u
#define USB_PID 0x5740u

enum { STR_LANGID = 0, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL, STR_INTERFACE };

static const tusb_desc_device_t g_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC, /* IAD: one function of two interfaces */
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = LINK_VERSION << 8, /* mcu-link/3 -> 0x0300 */
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&g_device;
}

enum { ITF_CDC = 0, ITF_CDC_DATA, ITF_TOTAL };
#define EP_NOTIF 0x81
#define EP_OUT 0x02
#define EP_IN 0x82
#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t g_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC, STR_INTERFACE, EP_NOTIF, 8, EP_OUT, EP_IN, CFG_TUD_CDC_EP_BUFSIZE),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return g_config;
}

static uint16_t g_str[USB_STRING_MAX_CHARS + 1u];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    char serial[25];
    const char *s;
    switch (index) {
    case STR_LANGID:
        g_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | 4u);
        g_str[1] = 0x0409; /* English (US) */
        return g_str;
    case STR_MANUFACTURER:
        s = "forklift";
        break;
    case STR_PRODUCT:
        s = FW_IDENTITY;
        break;
    case STR_SERIAL: {
        const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE;
        const uint32_t words[3] = {uid[0], uid[1], uid[2]};
        usb_serial_hex(words, serial);
        s = serial;
        break;
    }
    case STR_INTERFACE:
        s = "mcu-link";
        break;
    default:
        return 0;
    }
    (void)usb_string_desc(s, g_str, (uint32_t)(sizeof g_str / sizeof g_str[0]));
    return g_str;
}
