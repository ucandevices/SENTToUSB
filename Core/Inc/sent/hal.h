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

typedef bool (*sent_rx_start_fn)(void* context);
typedef void (*sent_rx_stop_fn)(void* context);
typedef bool (*sent_rx_poll_timestamps_fn)(void* context,
                                           uint32_t* out_timestamps_us,
                                           size_t* inout_timestamp_count);
/* Optional: update capture_batch_size when data_nibbles changes.
 * Called by bridge config handler and bridge_start_rx. May be NULL. */
typedef void (*sent_rx_set_data_nibbles_fn)(void* context, uint8_t data_nibbles);

/* Optional: update sync-detection threshold [µs] to match the configured tick
 * range.  Called by bridge when min/max_tick_x10_us changes.  May be NULL. */
typedef void (*sent_rx_set_sync_min_us_fn)(void* context, uint32_t sync_min_us);

typedef bool (*sent_tx_start_fn)(void* context);
typedef void (*sent_tx_stop_fn)(void* context);
typedef bool (*sent_tx_submit_frame_fn)(void* context,
                                        const sent_frame_t* frame,
                                        const sent_config_t* config,
                                        uint16_t pause_ticks);
/* Optional: set the TX tick period (HAL reprograms its timebase).
 * tick_x10_us is in 0.1-us units (e.g. 30 = 3.0 us).  May be NULL. */
typedef bool (*sent_tx_set_tick_fn)(void* context, uint16_t tick_x10_us);

typedef struct {
    void* context;
    sent_rx_start_fn start_rx;
    sent_rx_stop_fn stop_rx;
    sent_rx_poll_timestamps_fn poll_timestamps_us;
    sent_rx_set_data_nibbles_fn set_data_nibbles;  /* optional, NULL if not supported */
    sent_rx_set_sync_min_us_fn  set_sync_min_us;   /* optional, NULL if not supported */
} sent_rx_hal_t;

typedef struct {
    void* context;
    sent_tx_start_fn start_tx;
    sent_tx_stop_fn stop_tx;
    sent_tx_submit_frame_fn submit_frame;
    sent_tx_set_tick_fn set_tick_x10_us;  /* optional, NULL if not supported */
} sent_tx_hal_t;

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_H */
