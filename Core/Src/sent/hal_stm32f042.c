#include "sent/hal_stm32f042.h"

#include "sent/sent_assert.h"
#include <string.h>

#include "sent/sent_encoder.h"

#if !defined(SENT_HAL_STM32F042)
#error "src/hal_stm32f042.c requires SENT_HAL_STM32F042 to be enabled."
#endif

#define TICKS_TO_US_Q12_SCALE 4096000000U  /* (1_000_000 << 12) */

static uint8_t queue_next_idx(uint8_t idx, uint8_t depth) {
    uint8_t next = (uint8_t)(idx + 1U);
    return (next >= depth) ? 0U : next;
}

/* Convert timer ticks to microseconds using Q12 fixed-point multiply (no division in ISR).
 * @param hal    RX HAL instance (carries fractional accumulator state)
 * @param ticks  delta timer ticks since last edge [timer clock ticks]
 * @return       delta time [us] */
__attribute__((always_inline))
static inline uint32_t rx_ticks_to_us_no_div(sent_stm32f042_rx_hal_t* hal, uint32_t ticks) {
    SENT_ASSERT(hal != NULL);
    if (ticks == 0U || hal->ticks_to_us_mul_q12 == 0U) {
        return 0U;
    }

    uint32_t mul_q12 = hal->ticks_to_us_mul_q12;
    uint32_t max_delta = hal->ticks_to_us_max_delta;
    uint32_t frac_q12 = hal->ticks_to_us_frac_q12;
    uint32_t us = 0U;

    while (ticks > max_delta) {
        uint32_t scaled = (max_delta * mul_q12) + frac_q12;
        us += (scaled >> 12U);
        frac_q12 = (scaled & 0x0FFFU);
        ticks -= max_delta;
    }

    uint32_t scaled = (ticks * mul_q12) + frac_q12;
    us += (scaled >> 12U);
    hal->ticks_to_us_frac_q12 = (scaled & 0x0FFFU);
    return us;
}

/* Move the active timestamp buffer into the ready queue (called from ISR context).
 * @param hal  RX HAL instance with active_timestamps_us filled */
static void rx_push_active_batch_from_isr(sent_stm32f042_rx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);
    if (hal->active_count == 0U) {
        return;
    }

    uint8_t head = hal->ready_head;
    uint8_t tail = hal->ready_tail;
    uint8_t next_head = queue_next_idx(head, hal->config.ready_queue_depth);

    if (next_head == tail) {
        hal->dropped_batches++;
        hal->active_count = 0U;
        return;
    }

    sent_stm32f042_rx_batch_t* slot = &hal->ready_batches[head];
    slot->count = hal->active_count;
    for (uint8_t i = 0U; i < hal->active_count; ++i) {
        slot->timestamps_us[i] = hal->active_timestamps_us[i];
    }

    /* Retain the last captured edge as ts[0] for the next batch.
     * After a push, the SENT bus is idle for ~99 ms (inter-frame gap) and
     * the very next edge is the sync-end of the next frame.  If active_count
     * were reset to 0 here, the sync detection guard (active_count > 0) would
     * fail on that sync-end, placing it at ts[0] instead of ts[1] and
     * permanently misaligning every subsequent batch.  With active_count=1,
     * sync detection fires correctly and aligns the batch on the next frame. */
    hal->active_timestamps_us[0] = hal->active_timestamps_us[hal->active_count - 1U];
    hal->active_count = 1U;
    hal->ready_head = next_head;
}

/* Start STM32 RX HAL: validate config, compute Q12 multiplier, reset state.
 * @param context  sent_stm32f042_rx_hal_t pointer
 * @return         true if config is valid and HAL started */
static bool stm32_rx_start(void* context) {
    sent_stm32f042_rx_hal_t* hal = (sent_stm32f042_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL);

    if (hal->config.timer_clock_hz == 0U ||
        hal->config.capture_batch_size == 0U ||
        hal->config.capture_batch_size > SENT_STM32F042_RX_MAX_BATCH_SIZE ||
        hal->config.ready_queue_depth == 0U ||
        hal->config.ready_queue_depth > SENT_STM32F042_RX_MAX_READY_BATCHES) {
        return false;
    }

    hal->ready_head = 0U;
    hal->ready_tail = 0U;
    hal->dropped_batches = 0U;

    hal->overflow_count = 0U;
    hal->last_counter_ticks = 0U;
    hal->last_timestamp_us = 0U;
    hal->ticks_to_us_frac_q12 = 0U;
    /*
     * Q12 fixed-point conversion:
     * us ~= ticks * ((1_000_000 << 12) / timer_clock_hz)
     * Division is done once at start, not in ISR.
     */
    hal->ticks_to_us_mul_q12 = (TICKS_TO_US_Q12_SCALE + (hal->config.timer_clock_hz / 2U)) /
                               hal->config.timer_clock_hz;
    if (hal->ticks_to_us_mul_q12 == 0U) {
        hal->ticks_to_us_mul_q12 = 1U;
    }
    hal->ticks_to_us_max_delta = UINT32_MAX / hal->ticks_to_us_mul_q12;
    if (hal->ticks_to_us_max_delta == 0U) {
        hal->ticks_to_us_max_delta = 1U;
    }
    hal->active_count = 0U;

    hal->running = true;
    return true;
}

/* Stop STM32 RX HAL: clear running flag.
 * @param context  sent_stm32f042_rx_hal_t pointer */
static void stm32_rx_stop(void* context) {
    sent_stm32f042_rx_hal_t* hal = (sent_stm32f042_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    hal->running = false;
}

/* Dequeue next ready timestamp batch from RX queue (called from main loop).
 * @param context               sent_stm32f042_rx_hal_t pointer
 * @param out_timestamps_us     [out] array of edge timestamps [us]
 * @param inout_timestamp_count [in] max capacity, [out] actual count
 * @return                      true if a batch was dequeued */
static bool stm32_rx_poll(void* context,
                          uint32_t* out_timestamps_us,
                          size_t* inout_timestamp_count) {
    sent_stm32f042_rx_hal_t* hal = (sent_stm32f042_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL && out_timestamps_us != NULL && inout_timestamp_count != NULL);
    if (!hal->running) {
        return false;
    }

    uint8_t tail = hal->ready_tail;
    uint8_t head = hal->ready_head;
    if (tail == head) {
        return false;
    }

    sent_stm32f042_rx_batch_t* batch = &hal->ready_batches[tail];
    uint8_t batch_count = batch->count;
    if (*inout_timestamp_count < batch_count) {
        return false;
    }

    for (uint8_t i = 0U; i < batch_count; ++i) {
        out_timestamps_us[i] = batch->timestamps_us[i];
    }
    *inout_timestamp_count = batch_count;
    hal->ready_tail = queue_next_idx(tail, hal->config.ready_queue_depth);
    return true;
}

/* Initialize STM32 RX HAL with timer configuration (NULL config uses 48 MHz defaults).
 * @param hal     pointer to RX HAL instance
 * @param config  timer config (clock_hz [Hz], autoreload, batch_size, queue_depth) */
void sent_stm32f042_rx_hal_init(sent_stm32f042_rx_hal_t* hal,
                                const sent_stm32f042_rx_config_t* config) {
    SENT_ASSERT(hal != NULL);

    memset(hal, 0, sizeof(*hal));
    if (config != NULL) {
        hal->config = *config;
    } else {
        hal->config.timer_clock_hz = 48000000U;
        hal->config.timer_autoreload = 0xFFFFU;
        hal->config.capture_batch_size = SENT_STM32F042_RX_MAX_BATCH_SIZE;
        hal->config.ready_queue_depth = SENT_STM32F042_RX_MAX_READY_BATCHES;
    }
}

/* Update capture_batch_size when data_nibbles changes.
 * Safe to call from main context while RX is running: uint8_t writes are atomic
 * on Cortex-M0 and the ISR will pick up the new size on the next edge.
 * Resets active_count to discard any partial batch built with the old size. */
static void stm32_rx_set_data_nibbles(void* context, uint8_t data_nibbles) {
    sent_stm32f042_rx_hal_t* hal = (sent_stm32f042_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    hal->config.capture_batch_size = (uint8_t)(data_nibbles + 4U);
    hal->active_count = 0U;
}

/* Update sync-detection threshold.  Safe to call while RX is running: uint32_t
 * writes are atomic on Cortex-M0 and the ISR reads the new value on the next
 * edge.  Resets active_count to discard any partial batch aligned to the old
 * threshold. */
static void stm32_rx_set_sync_min_us(void* context, uint32_t sync_min_us) {
    sent_stm32f042_rx_hal_t* hal = (sent_stm32f042_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    hal->config.sync_min_us = sync_min_us;
    hal->active_count = 0U;
}

/* Wire STM32 RX HAL implementation into the generic RX HAL interface.
 * @param impl     STM32 RX HAL instance
 * @param out_hal  [out] generic RX HAL function-pointer struct to populate */
void sent_stm32f042_make_rx_hal(sent_stm32f042_rx_hal_t* impl, sent_rx_hal_t* out_hal) {
    SENT_ASSERT(out_hal != NULL);

    out_hal->context = impl;
    out_hal->start_rx = stm32_rx_start;
    out_hal->stop_rx = stm32_rx_stop;
    out_hal->poll_timestamps_us = stm32_rx_poll;
    out_hal->set_data_nibbles = stm32_rx_set_data_nibbles;
    out_hal->set_sync_min_us = stm32_rx_set_sync_min_us;
}

/* ISR handler: process a captured falling-edge counter value, convert to timestamp.
 *
 * Sync detection (when config.sync_min_us > 0):
 *   A SENT sync pulse (56 ticks) is much longer than any data nibble (12-27 ticks).
 *   When the interval from the previous edge to this edge is >= sync_min_us AND there
 *   are already edges in the active batch, the previous edge was the sync falling-edge.
 *   We discard the partial batch and restart it with that sync edge as timestamp[0].
 *
 *   If two consecutive long intervals arrive (long pause then sync), the second
 *   long interval re-fires sync detection and corrects the alignment automatically.
 *
 * @param hal               RX HAL instance
 * @param captured_counter  raw 16-bit timer capture register value [timer ticks] */
void sent_stm32f042_rx_on_capture_edge_isr(sent_stm32f042_rx_hal_t* hal,
                                            uint16_t captured_counter) {
    SENT_ASSERT(hal != NULL);
    if (!hal->running) {
        return;
    }

    uint32_t period = (uint32_t)hal->config.timer_autoreload + 1U;
    uint64_t counter_ticks = ((uint64_t)hal->overflow_count * period) + captured_counter;

    uint32_t delta_ticks = (uint32_t)(counter_ticks - hal->last_counter_ticks);
    hal->last_counter_ticks = counter_ticks;

    uint32_t prev_timestamp_us = hal->last_timestamp_us;
    hal->last_timestamp_us += rx_ticks_to_us_no_div(hal, delta_ticks);

    /* Sync detection: long interval => previous edge was the sync start.
     * Reset the batch and seed it with that sync edge (prev_timestamp_us). */
    if (hal->config.sync_min_us > 0U &&
        hal->active_count > 0U &&
        (hal->last_timestamp_us - prev_timestamp_us) >= hal->config.sync_min_us) {
        hal->active_count = 0U;
        hal->active_timestamps_us[0] = prev_timestamp_us;
        hal->active_count = 1U;
    }

    if (hal->active_count < hal->config.capture_batch_size &&
        hal->active_count < SENT_STM32F042_RX_MAX_BATCH_SIZE) {
        hal->active_timestamps_us[hal->active_count++] = hal->last_timestamp_us;
    }

    if (hal->active_count >= hal->config.capture_batch_size) {
        rx_push_active_batch_from_isr(hal);
    }
}

/* ISR handler: increment overflow counter on timer period rollover.
 * @param hal  RX HAL instance */
void sent_stm32f042_rx_on_overflow_isr(sent_stm32f042_rx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);
    if (!hal->running) {
        return;
    }
    hal->overflow_count++;
}

/* Return the number of timestamp batches dropped due to full ready queue.
 * @param hal  RX HAL instance
 * @return     dropped batch count */
uint32_t sent_stm32f042_rx_dropped_batches(const sent_stm32f042_rx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);
    return hal->dropped_batches;
}

/* Start STM32 TX HAL: reset state.
 * @param context  sent_stm32f042_tx_hal_t pointer
 * @return         always true */
static bool stm32_tx_start(void* context) {
    sent_stm32f042_tx_hal_t* hal = (sent_stm32f042_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL);

    hal->active_index = 0U;
    hal->count = 0U;
    hal->running = true;
    return true;
}

/* Stop STM32 TX HAL: clear running flag.
 * @param context  sent_stm32f042_tx_hal_t pointer */
static void stm32_tx_stop(void* context) {
    sent_stm32f042_tx_hal_t* hal = (sent_stm32f042_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    hal->running = false;
}

/* Encode a SENT frame into the TX interval buffer for ISR-driven transmission.
 * Returns false if the previous frame is still in progress or encoding failed.
 * @param context      sent_stm32f042_tx_hal_t pointer
 * @param frame        SENT frame to transmit
 * @param config       SENT protocol configuration
 * @param pause_ticks  pause pulse duration [ticks, 0 = use default]
 * @return             true if accepted, false if busy or encoding failed */
static bool stm32_tx_submit(void* context,
                            const sent_frame_t* frame,
                            const sent_config_t* config,
                            uint16_t pause_ticks) {
    sent_stm32f042_tx_hal_t* hal = (sent_stm32f042_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL && frame != NULL && config != NULL);
    if (!hal->running) {
        return false;
    }

    if (hal->active_index < hal->count) {
        return false;  /* previous frame still in progress */
    }

    uint16_t effective_pause = pause_ticks == 0U ? hal->config.default_pause_ticks : pause_ticks;
    uint16_t raw[SENT_STM32F042_TX_MAX_INTERVALS];
    size_t raw_count = 0U;
    if (!sent_build_intervals_ticks(frame, config, effective_pause, raw, &raw_count)) {
        return false;
    }

    /* Expand each interval into a LOW-toggle and a HIGH-toggle duration.
     * The ISR toggles the pin each time its countdown expires, so alternating
     * entries drive the LOW → HIGH → LOW → ... sequence automatically. */
    uint8_t low = hal->config.low_ticks;
    uint8_t n = 0U;
    for (size_t i = 0U; i < raw_count; ++i) {
        uint16_t high = (raw[i] > low) ? (uint16_t)(raw[i] - low) : 1U;
        hal->intervals[n++] = low;
        hal->intervals[n++] = high;
    }

    hal->active_index = 0U;
    /* volatile write last: publishes intervals[] to the ISR on M0's in-order store model */
    hal->count = n;
    return true;
}

/* Initialize STM32 TX HAL with queue configuration (NULL config uses defaults).
 * @param hal     pointer to TX HAL instance
 * @param config  TX config (default_pause_ticks) */
void sent_stm32f042_tx_hal_init(sent_stm32f042_tx_hal_t* hal,
                                const sent_stm32f042_tx_config_t* config) {
    SENT_ASSERT(hal != NULL);

    memset(hal, 0, sizeof(*hal));
    if (config != NULL) {
        hal->config = *config;
    } else {
        hal->config.default_pause_ticks = 12U;
        hal->config.low_ticks           = 5U;
    }
    if (hal->config.tx_tick_x10_us == 0U) {
        hal->config.tx_tick_x10_us = 30U;  /* SAE J2716 default: 3.0 us */
    }
}

/* Set the TX tick period.  Stored in config; the sent_app TIM14 driver reads it
 * from hal->config.tx_tick_x10_us when starting a new burst (tim14_kick).  The
 * caller must not change the tick while a TX burst is in flight — set between
 * frames only.  Valid range: 1..65535 (0.1 us units).  Returns true on success. */
static bool stm32_tx_set_tick_x10_us(void* context, uint16_t tick_x10_us) {
    sent_stm32f042_tx_hal_t* hal = (sent_stm32f042_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    if (tick_x10_us == 0U) {
        return false;
    }
    hal->config.tx_tick_x10_us = tick_x10_us;
    return true;
}

uint16_t sent_stm32f042_tx_get_tick_x10_us(const sent_stm32f042_tx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);
    return hal->config.tx_tick_x10_us;
}

/* Wire STM32 TX HAL implementation into the generic TX HAL interface.
 * @param impl     STM32 TX HAL instance
 * @param out_hal  [out] generic TX HAL function-pointer struct to populate */
void sent_stm32f042_make_tx_hal(sent_stm32f042_tx_hal_t* impl, sent_tx_hal_t* out_hal) {
    SENT_ASSERT(out_hal != NULL);

    out_hal->context = impl;
    out_hal->start_tx = stm32_tx_start;
    out_hal->stop_tx = stm32_tx_stop;
    out_hal->submit_frame = stm32_tx_submit;
    out_hal->set_tick_x10_us = stm32_tx_set_tick_x10_us;
}

/* ISR handler: pop the next TX interval from the flat interval array.
 * Does NOT check hal->running so a frame in progress completes fully after stop_tx().
 * New frames are blocked at submission time (stm32_tx_submit checks running).
 * @param hal                TX HAL instance
 * @param out_interval_ticks [out] next interval duration [timer ticks]
 * @return                   true if an interval was available, false if TX idle */
bool sent_stm32f042_tx_pop_next_interval_ticks_from_isr(sent_stm32f042_tx_hal_t* hal,
                                                         uint16_t* out_interval_ticks) {
    uint8_t index = hal->active_index;
    if (index >= hal->count) {
        return false;
    }
    *out_interval_ticks = hal->intervals[index];
    hal->active_index = (uint8_t)(index + 1U);
    return true;
}

/* Return 1 if a frame is currently being transmitted, 0 if idle.
 * @param hal  TX HAL instance
 * @return     number of pending frames */
size_t sent_stm32f042_tx_pending_frames(const sent_stm32f042_tx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);

    size_t pending = 0U;
    if (hal->active_index < hal->count) {
        pending = 1U;
    }
    return pending;
}
