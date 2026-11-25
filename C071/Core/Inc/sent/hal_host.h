/* Host (x86/Linux) simulation HAL for SENT RX and TX.
 *
 * Implements the sent_rx_hal_t / sent_tx_hal_t interfaces using a
 * mutex-protected ring buffer (RX) and a last-frame store (TX).
 * Intended for unit tests and desktop simulation; not used on embedded targets. */
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

#define SENT_HOST_RX_MAX_BATCHES    64U  /* ring-buffer capacity (power-of-two not required) */
#define SENT_HOST_RX_MAX_TIMESTAMPS 32U  /* max timestamps per injected batch */

/* One queued batch of falling-edge timestamps injected by the test harness. */
typedef struct {
    uint32_t timestamps_us[SENT_HOST_RX_MAX_TIMESTAMPS];
    size_t count;
} sent_host_rx_batch_t;

/* State for the host RX HAL instance.  Thread-safe via lock. */
typedef struct {
#if defined(SENT_HAL_HOST)
    pthread_mutex_t lock;
#endif
    bool running;                                    /* set by start_rx / cleared by stop_rx */
    sent_host_rx_batch_t queue[SENT_HOST_RX_MAX_BATCHES];  /* FIFO ring buffer */
    size_t head;                                     /* producer (inject) index */
    size_t tail;                                     /* consumer (poll) index */
} sent_host_rx_hal_t;

/* State for the host TX HAL instance.  Thread-safe via lock. */
typedef struct {
#if defined(SENT_HAL_HOST)
    pthread_mutex_t lock;
#endif
    bool running;                                    /* set by start_tx / cleared by stop_tx */
    bool has_last_frame;                             /* true after first successful submit */
    sent_frame_t last_frame;                         /* most recently submitted frame */
    uint16_t last_intervals_ticks[SENT_MAX_INTERVALS];  /* encoded tick intervals for that frame */
    size_t last_intervals_count;
} sent_host_tx_hal_t;

/* Initialize hal and its mutex; must be called before any other rx function. */
void sent_host_rx_hal_init(sent_host_rx_hal_t* hal);
/* Destroy the mutex; pair with init. */
void sent_host_rx_hal_deinit(sent_host_rx_hal_t* hal);
/* Enqueue a synthetic timestamp batch for the decoder to consume; returns false if full. */
bool sent_host_rx_hal_inject(sent_host_rx_hal_t* hal,
                             const uint32_t* timestamps_us,
                             size_t timestamp_count);
/* Returns true if start_rx has been called and stop_rx has not. */
bool sent_host_rx_hal_running(const sent_host_rx_hal_t* hal);
/* Returns the number of injected batches not yet consumed by poll. */
size_t sent_host_rx_hal_pending_batches(const sent_host_rx_hal_t* hal);
/* Populate out_hal with host RX function pointers backed by impl. */
void sent_host_make_rx_hal(sent_host_rx_hal_t* impl, sent_rx_hal_t* out_hal);

/* Initialize hal and its mutex; must be called before any other tx function. */
void sent_host_tx_hal_init(sent_host_tx_hal_t* hal);
/* Destroy the mutex; pair with init. */
void sent_host_tx_hal_deinit(sent_host_tx_hal_t* hal);
/* Returns true if start_tx has been called and stop_tx has not. */
bool sent_host_tx_hal_running(const sent_host_tx_hal_t* hal);
/* Copy the most recently submitted frame into out_frame; returns false if none yet. */
bool sent_host_tx_hal_last_frame(const sent_host_tx_hal_t* hal, sent_frame_t* out_frame);
/* Copy the tick-interval array for the last submitted frame; returns the count copied. */
size_t sent_host_tx_hal_last_intervals(const sent_host_tx_hal_t* hal,
                                       uint16_t* out_intervals,
                                       size_t max_count);
/* Populate out_hal with host TX function pointers backed by impl. */
void sent_host_make_tx_hal(sent_host_tx_hal_t* impl, sent_tx_hal_t* out_hal);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_HOST_H */
