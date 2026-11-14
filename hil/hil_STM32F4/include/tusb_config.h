/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Target & Operating System Setup
//--------------------------------------------------------------------+
#define CFG_TUSB_MCU                OPT_MCU_STM32F4
#define CFG_TUSB_OS                 OPT_OS_NONE      /* Pure bare-metal */
#define CFG_TUSB_DEBUG              0

#define CFG_TUD_ENABLED             1
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE
#define BOARD_TUD_RHPORT            0                /* Port 0: OTG_FS */
#define BOARD_TUD_MAX_SPEED         OPT_MODE_DEFAULT_SPEED

#define CFG_TUD_ENDPOINT0_SIZE      64

//--------------------------------------------------------------------+
// Class Driver Selection
//--------------------------------------------------------------------+
#define CFG_TUD_CDC                 1                /* Enable CDC class */
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

//--------------------------------------------------------------------+
// CDC FIFO & Packet Buffer Sizes
//--------------------------------------------------------------------+
#define CFG_TUD_CDC_RX_BUFSIZE      64
#define CFG_TUD_CDC_TX_BUFSIZE      64
#define CFG_TUD_CDC_EP_BUFSIZE      64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */