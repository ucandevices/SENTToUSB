#include "sent/hal_host.h"

#include <string.h>

#include "sent/sent_encoder.h"

#if !defined(SENT_HAL_HOST)
#error "src/hal_host.c is host-only. Build with SENT_HAL_HOST enabled."
#endif

static size_t queue_next(size_t index) {
    return (index + 1U) % SENT_HOST_RX_MAX_BATCHES;
}

static bool host_rx_start(void* context) {
    sent_host_rx_hal_t* hal = (sent_host_rx_hal_t*)context;
    if (hal == NULL) {
        return false;
    }

    pthread_mutex_lock(&hal->lock);
    hal->running = true;
    hal->head = 0U;
    hal->tail = 0U;
    pthread_mutex_unlock(&hal->lock);
    return true;
}

static void host_rx_stop(void* context) {
    sent_host_rx_hal_t* hal = (sent_host_rx_hal_t*)context;
    if (hal == NULL) {
        return;
    }

    pthread_mutex_lock(&hal->lock);
    hal->running = false;
    hal->head = 0U;
    hal->tail = 0U;
    pthread_mutex_unlock(&hal->lock);
}

static bool host_rx_poll(void* context,
                         uint32_t* out_timestamps_us,
                         size_t* inout_timestamp_count) {
    sent_host_rx_hal_t* hal = (sent_host_rx_hal_t*)context;
    if (hal == NULL || out_timestamps_us == NULL || inout_timestamp_count == NULL) {
        return false;
    }

    pthread_mutex_lock(&hal->lock);
    if (!hal->running || hal->head == hal->tail) {
        pthread_mutex_unlock(&hal->lock);
        return false;
    }

    sent_host_rx_batch_t batch = hal->queue[hal->tail];
    hal->tail = queue_next(hal->tail);
    pthread_mutex_unlock(&hal->lock);

    if (*inout_timestamp_count < batch.count) {
        return false;
    }

    for (size_t i = 0; i < batch.count; ++i) {
        out_timestamps_us[i] = batch.timestamps_us[i];
    }
    *inout_timestamp_count = batch.count;
    return true;
}

void sent_host_rx_hal_init(sent_host_rx_hal_t* hal) {
    if (hal == NULL) {
        return;
    }
    memset(hal, 0, sizeof(*hal));
    pthread_mutex_init(&hal->lock, NULL);
}

void sent_host_rx_hal_deinit(sent_host_rx_hal_t* hal) {
    if (hal == NULL) {
        return;
    }
    pthread_mutex_destroy(&hal->lock);
}

bool sent_host_rx_hal_inject(sent_host_rx_hal_t* hal,
                             const uint32_t* timestamps_us,
                             size_t timestamp_count) {
    if (hal == NULL || timestamps_us == NULL || timestamp_count == 0U ||
        timestamp_count > SENT_HOST_RX_MAX_TIMESTAMPS) {
        return false;
    }

    pthread_mutex_lock(&hal->lock);
    size_t next_head = queue_next(hal->head);
    if (next_head == hal->tail) {
        pthread_mutex_unlock(&hal->lock);
        return false;
    }

    sent_host_rx_batch_t* slot = &hal->queue[hal->head];
    slot->count = timestamp_count;
    for (size_t i = 0; i < timestamp_count; ++i) {
        slot->timestamps_us[i] = timestamps_us[i];
    }
    hal->head = next_head;
    pthread_mutex_unlock(&hal->lock);
    return true;
}

bool sent_host_rx_hal_running(const sent_host_rx_hal_t* hal) {
    if (hal == NULL) {
        return false;
    }

    pthread_mutex_lock((pthread_mutex_t*)&hal->lock);
    bool running = hal->running;
    pthread_mutex_unlock((pthread_mutex_t*)&hal->lock);
    return running;
}

size_t sent_host_rx_hal_pending_batches(const sent_host_rx_hal_t* hal) {
    if (hal == NULL) {
        return 0U;
    }

    pthread_mutex_lock((pthread_mutex_t*)&hal->lock);
    size_t head = hal->head;
    size_t tail = hal->tail;
    pthread_mutex_unlock((pthread_mutex_t*)&hal->lock);

    if (head >= tail) {
        return head - tail;
    }
    return (SENT_HOST_RX_MAX_BATCHES - tail) + head;
}

void sent_host_make_rx_hal(sent_host_rx_hal_t* impl, sent_rx_hal_t* out_hal) {
    if (out_hal == NULL) {
        return;
    }

    out_hal->context = impl;
    out_hal->start_rx = host_rx_start;
    out_hal->stop_rx = host_rx_stop;
    out_hal->poll_timestamps_us = host_rx_poll;
}

static bool host_tx_start(void* context) {
    sent_host_tx_hal_t* hal = (sent_host_tx_hal_t*)context;
    if (hal == NULL) {
        return false;
    }

    pthread_mutex_lock(&hal->lock);
    hal->running = true;
    pthread_mutex_unlock(&hal->lock);
    return true;
}

static void host_tx_stop(void* context) {
    sent_host_tx_hal_t* hal = (sent_host_tx_hal_t*)context;
    if (hal == NULL) {
        return;
    }

    pthread_mutex_lock(&hal->lock);
    hal->running = false;
    pthread_mutex_unlock(&hal->lock);
}

static bool host_tx_submit(void* context,
                           const sent_frame_t* frame,
                           const sent_config_t* config,
                           uint16_t pause_ticks) {
    sent_host_tx_hal_t* hal = (sent_host_tx_hal_t*)context;
    if (hal == NULL || frame == NULL || config == NULL) {
        return false;
    }

    pthread_mutex_lock(&hal->lock);
    if (!hal->running) {
        pthread_mutex_unlock(&hal->lock);
        return false;
    }

    size_t count = 0U;
    bool ok = sent_build_intervals_ticks(frame,
                                         config,
                                         pause_ticks,
                                         hal->last_intervals_ticks,
                                         &count);
    if (ok) {
        hal->last_frame = *frame;
        hal->has_last_frame = true;
        hal->last_intervals_count = count;
    }
    pthread_mutex_unlock(&hal->lock);
    return ok;
}

void sent_host_tx_hal_init(sent_host_tx_hal_t* hal) {
    if (hal == NULL) {
        return;
    }
    memset(hal, 0, sizeof(*hal));
    pthread_mutex_init(&hal->lock, NULL);
}

void sent_host_tx_hal_deinit(sent_host_tx_hal_t* hal) {
    if (hal == NULL) {
        return;
    }
    pthread_mutex_destroy(&hal->lock);
}

bool sent_host_tx_hal_running(const sent_host_tx_hal_t* hal) {
    if (hal == NULL) {
        return false;
    }

    pthread_mutex_lock((pthread_mutex_t*)&hal->lock);
    bool running = hal->running;
    pthread_mutex_unlock((pthread_mutex_t*)&hal->lock);
    return running;
}

bool sent_host_tx_hal_last_frame(const sent_host_tx_hal_t* hal, sent_frame_t* out_frame) {
    if (hal == NULL || out_frame == NULL) {
        return false;
    }

    pthread_mutex_lock((pthread_mutex_t*)&hal->lock);
    if (!hal->has_last_frame) {
        pthread_mutex_unlock((pthread_mutex_t*)&hal->lock);
        return false;
    }
    *out_frame = hal->last_frame;
    pthread_mutex_unlock((pthread_mutex_t*)&hal->lock);
    return true;
}

size_t sent_host_tx_hal_last_intervals(const sent_host_tx_hal_t* hal,
                                       uint16_t* out_intervals,
                                       size_t max_count) {
    if (hal == NULL) {
        return 0U;
    }

    pthread_mutex_lock((pthread_mutex_t*)&hal->lock);
    size_t count = hal->last_intervals_count;
    if (out_intervals != NULL && max_count > 0U) {
        if (count > max_count) {
            count = max_count;
        }
        for (size_t i = 0; i < count; ++i) {
            out_intervals[i] = hal->last_intervals_ticks[i];
        }
    }
    pthread_mutex_unlock((pthread_mutex_t*)&hal->lock);
    return count;
}

void sent_host_make_tx_hal(sent_host_tx_hal_t* impl, sent_tx_hal_t* out_hal) {
    if (out_hal == NULL) {
        return;
    }

    out_hal->context = impl;
    out_hal->start_tx = host_tx_start;
    out_hal->stop_tx = host_tx_stop;
    out_hal->submit_frame = host_tx_submit;
}
