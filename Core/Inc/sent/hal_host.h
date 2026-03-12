#ifndef SENT_HAL_HOST_H
#define SENT_HAL_HOST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/hal.h"

#if defined(SENT_HAL_HOST)
#include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_HOST_RX_MAX_BATCHES 64U
#define SENT_HOST_RX_MAX_TIMESTAMPS 32U

typedef struct {
    uint32_t timestamps_us[SENT_HOST_RX_MAX_TIMESTAMPS];
    size_t count;
} sent_host_rx_batch_t;

typedef struct {
#if defined(SENT_HAL_HOST)
    pthread_mutex_t lock;
#endif
    bool running;
    sent_host_rx_batch_t queue[SENT_HOST_RX_MAX_BATCHES];
    size_t head;
    size_t tail;
} sent_host_rx_hal_t;

typedef struct {
#if defined(SENT_HAL_HOST)
    pthread_mutex_t lock;
#endif
    bool running;
    bool has_last_frame;
    sent_frame_t last_frame;
    uint16_t last_intervals_ticks[SENT_MAX_INTERVALS];
    size_t last_intervals_count;
} sent_host_tx_hal_t;

void sent_host_rx_hal_init(sent_host_rx_hal_t* hal);
void sent_host_rx_hal_deinit(sent_host_rx_hal_t* hal);
bool sent_host_rx_hal_inject(sent_host_rx_hal_t* hal,
                             const uint32_t* timestamps_us,
                             size_t timestamp_count);
bool sent_host_rx_hal_running(const sent_host_rx_hal_t* hal);
size_t sent_host_rx_hal_pending_batches(const sent_host_rx_hal_t* hal);
void sent_host_make_rx_hal(sent_host_rx_hal_t* impl, sent_rx_hal_t* out_hal);

void sent_host_tx_hal_init(sent_host_tx_hal_t* hal);
void sent_host_tx_hal_deinit(sent_host_tx_hal_t* hal);
bool sent_host_tx_hal_running(const sent_host_tx_hal_t* hal);
bool sent_host_tx_hal_last_frame(const sent_host_tx_hal_t* hal, sent_frame_t* out_frame);
size_t sent_host_tx_hal_last_intervals(const sent_host_tx_hal_t* hal,
                                       uint16_t* out_intervals,
                                       size_t max_count);
void sent_host_make_tx_hal(sent_host_tx_hal_t* impl, sent_tx_hal_t* out_hal);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_HOST_H */
