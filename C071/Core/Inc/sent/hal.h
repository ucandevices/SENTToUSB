/* Abstract HAL interface for SENT protocol receive and transmit.
 *
 * Each backend (host simulation, STM32F042, …) fills a sent_rx_hal_t or
 * sent_tx_hal_t with concrete function pointers and a driver-private context
 * pointer.  Upper layers call these through the interface without knowing the
 * underlying hardware. */
#ifndef SENT_HAL_H
#define SENT_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/hal_config.h"
#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enable the hardware capture path; returns false if the HAL is misconfigured. */
typedef bool (*sent_rx_start_fn)(void* context);
/* Disable capture and discard buffered data. */
typedef void (*sent_rx_stop_fn)(void* context);
/* Dequeue falling-edge timestamps from the driver buffer.  The unit is
 * 1/ticks_per_us microseconds (see sent_rx_hal_t.ticks_per_us).  A backend may
 * return frame-aligned batches or a raw, unaligned run of edges.
 * inout_timestamp_count: in = buffer capacity, out = actual timestamps written.
 * Returns true when at least one timestamp was copied. */
typedef bool (*sent_rx_poll_timestamps_fn)(void* context,
                                           uint32_t* out_timestamps_us,
                                           size_t* inout_timestamp_count);
/* Optional: update capture_batch_size when data_nibbles changes.
 * Called by bridge config handler and bridge_start_rx. May be NULL. */
typedef void (*sent_rx_set_data_nibbles_fn)(void* context, uint8_t data_nibbles);

/* Optional: update sync-detection threshold [µs] to match the configured tick
 * range.  Called by bridge when min/max_tick_x10_us changes.  May be NULL. */
typedef void (*sent_rx_set_sync_min_us_fn)(void* context, uint32_t sync_min_us);

/* Enable the transmit path; returns false if the HAL is misconfigured. */
typedef bool (*sent_tx_start_fn)(void* context);
/* Disable transmit; a frame already in flight completes before the pin idles. */
typedef void (*sent_tx_stop_fn)(void* context);
/* Encode and queue one frame for transmission.
 * pause_ticks: inter-frame gap duration [ticks]; 0 uses the HAL default.
 * Returns false if busy or if encoding fails. */
typedef bool (*sent_tx_submit_frame_fn)(void* context,
                                        const sent_frame_t* frame,
                                        const sent_config_t* config,
                                        uint16_t pause_ticks);
/* Optional: set the TX tick period (HAL reprograms its timebase).
 * tick_x10_us is in 0.1-us units (e.g. 30 = 3.0 us).  May be NULL. */
typedef bool (*sent_tx_set_tick_fn)(void* context, uint16_t tick_x10_us);

/* Generic RX HAL: function-pointer table populated by each backend. */
typedef struct {
    void* context;                                  /* driver-private instance */
    sent_rx_start_fn start_rx;
    sent_rx_stop_fn stop_rx;
    sent_rx_poll_timestamps_fn poll_timestamps_us;
    sent_rx_set_data_nibbles_fn set_data_nibbles;   /* optional, NULL if not supported */
    sent_rx_set_sync_min_us_fn  set_sync_min_us;    /* optional, NULL if not supported */
    uint32_t ticks_per_us;                          /* timestamp resolution [counts per us]; 0 means 1 */
} sent_rx_hal_t;

/* Generic TX HAL: function-pointer table populated by each backend. */
typedef struct {
    void* context;                                  /* driver-private instance */
    sent_tx_start_fn start_tx;
    sent_tx_stop_fn stop_tx;
    sent_tx_submit_frame_fn submit_frame;
    sent_tx_set_tick_fn set_tick_x10_us;            /* optional, NULL if not supported */
} sent_tx_hal_t;

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_H */
