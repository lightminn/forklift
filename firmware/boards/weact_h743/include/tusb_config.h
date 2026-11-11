/* TinyUSB configuration (board design 5): one CDC port on USB2 OTG FS (PA11/PA12, internal FS
   PHY), device only, no RTOS, no DMA (slave mode: the CPU fills the FIFOs, buffers stay in
   DTCM), VBUS sensing off. */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU OPT_MCU_STM32H7
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0

/* TinyUSB numbers the STM32 controllers OTG_FS = 0, OTG_HS = 1 (dwc2_stm32.h). */
#define BOARD_TUD_RHPORT 0
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUD_DWC2_DMA_ENABLE 0
#define CFG_TUD_VBUS_DETECT_HW 0 /* no VBUS sensing: B-session valid is forced (dcd_dwc2.c) */

#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64

#endif
