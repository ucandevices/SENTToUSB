/*
 * sent_app_config.h — build-time sizes of the application layer.
 *
 * SENT_NUM_CHANNELS is 1 for the STM32F042 board (v1.1 hardware) and 4 for
 * the STM32C071 board.  Everything below can be overridden on the compiler
 * command line (the host tests build the application core both ways).
 */
#ifndef SENT_APP_CONFIG_H
#define SENT_APP_CONFIG_H

#ifndef SENT_NUM_CHANNELS
#if defined(STM32C071xx)
#define SENT_NUM_CHANNELS 4U
#else
#define SENT_NUM_CHANNELS 1U
#endif
#endif

/* The channel number travels in a nibble, and the per-channel CAN IDs
 * (0x510 + 0x40*n ...) stay below the control/learn IDs only up to 4. */
#if (SENT_NUM_CHANNELS < 1) || (SENT_NUM_CHANNELS > 4)
#error "SENT_NUM_CHANNELS must be 1..4"
#endif

/* Firmware version reported by 'V' / 'v' (4 digits). */
#define SENT_FW_VERSION "0200"

/* USB TX ring (main loop only).  Worst case output is about 31 kB/s per
 * channel; the F042 has no RAM for more than v1.1's 384 bytes. */
#ifndef SENT_USB_TX_BUF_SIZE
#if SENT_NUM_CHANNELS > 1
#define SENT_USB_TX_BUF_SIZE 2048U
#else
#define SENT_USB_TX_BUF_SIZE 384U
#endif
#endif

/* USB RX byte ring, filled by the USB interrupt.  Power of two. */
#ifndef SENT_USB_RX_RING_SIZE
#if SENT_NUM_CHANNELS > 1
#define SENT_USB_RX_RING_SIZE 512U
#else
#define SENT_USB_RX_RING_SIZE 128U
#endif
#endif

/* Edges handed to one channel's bridge per main-loop pass, so that one busy
 * or noisy channel cannot starve the others or the USB flush.  With one
 * channel the whole capture ring is drained every pass, as in v1.1. */
#ifndef SENT_EDGE_BUDGET
#if SENT_NUM_CHANNELS > 1
#define SENT_EDGE_BUDGET 16U
#else
#define SENT_EDGE_BUDGET 64U
#endif
#endif

#endif /* SENT_APP_CONFIG_H */
