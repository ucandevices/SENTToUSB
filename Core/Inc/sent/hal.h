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

typedef bool (*sent_tx_start_fn)(void* context);
typedef void (*sent_tx_stop_fn)(void* context);
typedef bool (*sent_tx_submit_frame_fn)(void* context,
                                        const sent_frame_t* frame,
                                        const sent_config_t* config,
                                        uint16_t pause_ticks);

typedef struct {
    void* context;
    sent_rx_start_fn start_rx;
    sent_rx_stop_fn stop_rx;
    sent_rx_poll_timestamps_fn poll_timestamps_us;
} sent_rx_hal_t;

typedef struct {
    void* context;
    sent_tx_start_fn start_tx;
    sent_tx_stop_fn stop_tx;
    sent_tx_submit_frame_fn submit_frame;
} sent_tx_hal_t;

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_H */
