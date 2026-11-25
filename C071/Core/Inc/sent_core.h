#ifndef SENT_CORE_H
#define SENT_CORE_H

/*
 * sent_core.h — MCU-independent part of the SENT-to-USB application.
 *
 * Owns one sent_bridge_t per channel, the learn storage they share, the USB
 * byte rings, the SLCAN command dispatch with channel addressing, the
 * per-channel diagnostics and the round-robin main-loop pump.  The platform
 * (sent_app.c for the F042, sent_app_c071.c for the C071, or a host test)
 * supplies the RX/TX HAL tables and the few hooks in sent_core_platform_t.
 *
 * Interrupt discipline (kept from v1.1): sent_core_on_usb_rx() is the only
 * function that may run in interrupt context, and it only stores bytes.
 * Parsing, every bridge state change and every USB reply happen in
 * sent_core_process(), i.e. in the main loop.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sent_app_config.h"
#include "sent_bridge.h"
#include "sent/hal.h"
#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sensor-supply status (frame SENT_CAN_ID_SENT_SUPPLY).  sent_core pre-fills
 * the rev 2d fields with their "not measured" values, so a platform only
 * sets what it has. */
typedef struct {
    uint16_t vs_mv;     /* measured sensor supply VS, mV */
    uint8_t  flags;     /* SENT_CORE_SUPPLY_* bits */
    uint16_t is_ma;     /* rev 2d: sensor-rail current, mA; SENT_CORE_SUPPLY_IS_NA = not measured */
    uint8_t  source;    /* rev 2d: SENT_CORE_SUPPLY_SRC_* */
} sent_core_supply_t;

typedef struct {
    /* Millisecond tick (diagnostic rate limit). */
    uint32_t (*now_ms)(void);
    /* Hand a contiguous block to the USB device.  true = accepted (the
     * buffer must stay untouched until the transfer is done, which the ring
     * guarantees by not reusing it before the next call succeeds). */
    bool (*usb_transmit)(const uint8_t* data, uint16_t len);
    /* Monotonic count of edges lost upstream on a channel (capture ring
     * full, timer over-capture).  May be NULL. */
    uint32_t (*rx_lost_edges)(uint8_t channel);
    /* A TX frame may have been queued on the channel: start the timer if it
     * is idle.  May be NULL. */
    void (*tx_kick)(uint8_t channel);

    /* ── Added in rev 2b; all optional (NULL / 0 on the F042 build). ── */

    /* The line state of a channel changed: its mode, or whether its
     * termination should be on.  Called from the main loop, once per channel
     * at sent_core_init() (mode OFF, termination off) and then on every
     * change, always before a TX frame is handed to tx_kick().  The platform
     * drives TX_EN, TERM_EN and the channel's capture interrupt from this. */
    void (*line_control)(uint8_t channel, sent_bridge_mode_t mode, bool termination);
    /* Monotonic count of edge-rate guard trips on a channel.  May be NULL. */
    uint32_t (*rx_guard_trips)(uint8_t channel);
    /* Sensor-supply status for the SENT_CAN_ID_SENT_SUPPLY frame.  Return
     * false when there is nothing to report (USB not configured).  Fill in
     * *status (see sent_core_supply_t).  May be NULL. */
    bool (*supply_status)(sent_core_supply_t* status);
    /* SENT_CORE_FEATURE_* bits, reported in the info frame (data[6]). */
    uint8_t features;
} sent_core_platform_t;

#define SENT_CORE_FEATURE_TERMINATION   0x01U  /* switchable termination (TERM_EN) */
#define SENT_CORE_FEATURE_SUPPLY_SENSE  0x02U  /* VS measured, 0x60E sent */
#define SENT_CORE_FEATURE_TX_BUFFER     0x04U  /* push-pull TX buffer with TX_EN */

#define SENT_CORE_SUPPLY_EN    0x01U  /* USB supply path switched on (VS_EN) */
#define SENT_CORE_SUPPLY_FAULT 0x02U  /* load switch reports a fault */

#define SENT_CORE_SUPPLY_IS_NA     0xFFFFU  /* is_ma: current not measured */
#define SENT_CORE_SUPPLY_SRC_USB   0x00U    /* VS from the USB path (TPS2121 ST low) */
#define SENT_CORE_SUPPLY_SRC_EXT   0x01U    /* VS from EXT_IN, or no source on (ST high) */
#define SENT_CORE_SUPPLY_SRC_NA    0xFFU    /* source not reported */

#define SENT_CORE_SUPPLY_PERIOD_MS 500U

/* Initialise all channels with default_config.  rx_hal / tx_hal hold one
 * entry per channel (an entry with a NULL context is treated as absent). */
void sent_core_init(const sent_core_platform_t* platform,
                    const sent_config_t* default_config,
                    const sent_rx_hal_t rx_hal[SENT_NUM_CHANNELS],
                    const sent_tx_hal_t tx_hal[SENT_NUM_CHANNELS],
                    uint16_t serial_number);

/* USB receive callback (interrupt context): queue bytes only. */
void sent_core_on_usb_rx(const uint8_t* data, uint32_t len);

/* Main-loop pump: host commands, edges of every channel, diagnostics, USB flush. */
void sent_core_process(void);

/* Access for the platform layer and tests. */
sent_bridge_t* sent_core_bridge(uint8_t channel);
uint32_t sent_core_usb_overflows(uint8_t channel);
/* Termination setting (SENT_BRIDGE_TERM_*) and its current effective state. */
uint8_t sent_core_termination_setting(uint8_t channel);
bool sent_core_termination_on(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* SENT_CORE_H */
