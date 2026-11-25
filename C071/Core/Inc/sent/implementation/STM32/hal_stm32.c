#include "sent/hal_stm32.h"

#include "sent/sent_assert.h"

#include "sent/sent_encoder.h"

#if !defined(SENT_HAL_STM32)
#error "src/hal_stm32.c requires SENT_HAL_STM32 to be enabled."
#endif

#define RX_RING_MASK ((uint8_t)(SENT_STM32_RX_RING_SIZE - 1U))

/* Start STM32 RX HAL: validate config, empty the edge ring.
 * @param context  sent_stm32_rx_hal_t pointer
 * @return         true if config is valid and HAL started */
static bool stm32_rx_start(void* context) {
    sent_stm32_rx_hal_t* hal = (sent_stm32_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL);

    /* Timestamps are raw timer counts, so the clock must be a whole number of MHz. */
    if (hal->config.timer_clock_hz < 1000000U ||
        (hal->config.timer_clock_hz % 1000000U) != 0U) {
        return false;
    }

    hal->running = false;            /* keep the ISR out while the ring is reset */
    hal->head = 0U;
    hal->tail = 0U;
    /* dropped_edges is not cleared: it is monotonic, and the main loop treats
     * any change as a hole in the stream (v1.1 cleared it here, so a restart
     * after an overrun was reported as one more overrun). */
    hal->running = true;
    return true;
}

/* Stop STM32 RX HAL: clear running flag.
 * @param context  sent_stm32_rx_hal_t pointer */
static void stm32_rx_stop(void* context) {
    sent_stm32_rx_hal_t* hal = (sent_stm32_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    hal->running = false;
}

/* Dequeue captured edges from the ring (called from main loop).
 * @param context               sent_stm32_rx_hal_t pointer
 * @param out_timestamps        [out] array of edge timestamps [timer counts]
 * @param inout_timestamp_count [in] max capacity, [out] actual count
 * @return                      true if at least one edge was dequeued */
static bool stm32_rx_poll(void* context,
                          uint32_t* out_timestamps,
                          size_t* inout_timestamp_count) {
    sent_stm32_rx_hal_t* hal = (sent_stm32_rx_hal_t*)context;
    SENT_ASSERT(hal != NULL && out_timestamps != NULL && inout_timestamp_count != NULL);
    if (!hal->running) {
        return false;
    }

    size_t capacity = *inout_timestamp_count;
    size_t n = 0U;
    uint8_t tail = hal->tail;
    uint8_t head = hal->head;        /* single snapshot: ISR may advance it meanwhile */
    while (tail != head && n < capacity) {
        out_timestamps[n++] = hal->edges[tail];
        tail = (uint8_t)((tail + 1U) & RX_RING_MASK);
    }
    hal->tail = tail;
    *inout_timestamp_count = n;
    return n > 0U;
}

/* Initialize STM32 RX HAL with timer configuration (NULL config uses 48 MHz).
 * @param hal     pointer to RX HAL instance
 * @param config  timer config (clock_hz [Hz]) */
void sent_stm32_rx_hal_init(sent_stm32_rx_hal_t* hal,
                                const sent_stm32_rx_config_t* config) {
    SENT_ASSERT(hal != NULL);

    if (config != NULL) {
        hal->config = *config;
    } else {
        hal->config.timer_clock_hz = 48000000U;
    }
    hal->head = 0U;
    hal->tail = 0U;
    hal->dropped_edges = 0U;
    hal->running = false;   /* gates the ISR; start_rx() resets all runtime state */
}

/* Wire STM32 RX HAL implementation into the generic RX HAL interface.
 * @param impl     STM32 RX HAL instance
 * @param out_hal  [out] generic RX HAL function-pointer struct to populate */
void sent_stm32_make_rx_hal(sent_stm32_rx_hal_t* impl, sent_rx_hal_t* out_hal) {
    SENT_ASSERT(out_hal != NULL);

    out_hal->context = impl;
    out_hal->start_rx = stm32_rx_start;
    out_hal->stop_rx = stm32_rx_stop;
    out_hal->poll_timestamps_us = stm32_rx_poll;
    out_hal->set_data_nibbles = NULL;    /* framing is done downstream on raw edges */
    out_hal->set_sync_min_us = NULL;
    out_hal->ticks_per_us = (impl != NULL) ? (impl->config.timer_clock_hz / 1000000U) : 1U;
}

/* Return the number of edges dropped because the ring was full.
 * @param hal  RX HAL instance
 * @return     lost edge count since init (monotonic) */
uint32_t sent_stm32_rx_dropped_edges(const sent_stm32_rx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);
    return hal->dropped_edges;
}

/* Start STM32 TX HAL: reset state.
 * @param context  sent_stm32_tx_hal_t pointer
 * @return         always true */
static bool stm32_tx_start(void* context) {
    sent_stm32_tx_hal_t* hal = (sent_stm32_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL);

    hal->active_index = 0U;
    hal->count = 0U;
    hal->running = true;
    return true;
}

/* Stop STM32 TX HAL: clear running flag.
 * @param context  sent_stm32_tx_hal_t pointer */
static void stm32_tx_stop(void* context) {
    sent_stm32_tx_hal_t* hal = (sent_stm32_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    hal->running = false;
}

/* Encode a SENT frame into the TX interval buffer for ISR-driven transmission.
 * Returns false if the previous frame is still in progress or encoding failed.
 * @param context      sent_stm32_tx_hal_t pointer
 * @param frame        SENT frame to transmit
 * @param config       SENT protocol configuration
 * @param pause_ticks  pause pulse duration [ticks, 0 = use default]
 * @return             true if accepted, false if busy or encoding failed */
static bool stm32_tx_submit(void* context,
                            const sent_frame_t* frame,
                            const sent_config_t* config,
                            uint16_t pause_ticks) {
    sent_stm32_tx_hal_t* hal = (sent_stm32_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL && frame != NULL && config != NULL);
    if (!hal->running) {
        return false;
    }

    if (hal->active_index < hal->count) {
        return false;  /* previous frame still in progress */
    }

    uint16_t effective_pause = pause_ticks == 0U ? hal->config.default_pause_ticks : pause_ticks;
    uint16_t raw[SENT_STM32_TX_MAX_INTERVALS];
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
void sent_stm32_tx_hal_init(sent_stm32_tx_hal_t* hal,
                                const sent_stm32_tx_config_t* config) {
    SENT_ASSERT(hal != NULL);

    if (config != NULL) {
        hal->config = *config;
    } else {
        hal->config.default_pause_ticks = 12U;
        hal->config.low_ticks           = 5U;
        hal->config.tx_tick_x10_us      = 0U;
    }
    if (hal->config.tx_tick_x10_us == 0U) {
        hal->config.tx_tick_x10_us = 30U;  /* SAE J2716 default: 3.0 us */
    }
    hal->running = false;
    hal->count = 0U;          /* ISR pop bails when active_index >= count */
    hal->active_index = 0U;
}

/* Set the TX tick period.  Stored in config; the platform TX driver reads it
 * from hal->config.tx_tick_x10_us when starting a new burst.  Do not change
 * while a TX burst is in flight — set between frames only.
 * Valid range: 1..65535 (0.1 µs units).  Returns true on success. */
static bool stm32_tx_set_tick_x10_us(void* context, uint16_t tick_x10_us) {
    sent_stm32_tx_hal_t* hal = (sent_stm32_tx_hal_t*)context;
    SENT_ASSERT(hal != NULL);
    if (tick_x10_us == 0U) {
        return false;
    }
    hal->config.tx_tick_x10_us = tick_x10_us;
    return true;
}

/* Return the configured TX tick period [0.1-us units].
 * @param hal  TX HAL instance
 * @return     tick period (e.g. 30 = 3.0 µs) */
uint16_t sent_stm32_tx_get_tick_x10_us(const sent_stm32_tx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);
    return hal->config.tx_tick_x10_us;
}

/* Wire STM32 TX HAL implementation into the generic TX HAL interface.
 * @param impl     STM32 TX HAL instance
 * @param out_hal  [out] generic TX HAL function-pointer struct to populate */
void sent_stm32_make_tx_hal(sent_stm32_tx_hal_t* impl, sent_tx_hal_t* out_hal) {
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
bool sent_stm32_tx_pop_next_interval_ticks_from_isr(sent_stm32_tx_hal_t* hal,
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
size_t sent_stm32_tx_pending_frames(const sent_stm32_tx_hal_t* hal) {
    SENT_ASSERT(hal != NULL);

    size_t pending = 0U;
    if (hal->active_index < hal->count) {
        pending = 1U;
    }
    return pending;
}
