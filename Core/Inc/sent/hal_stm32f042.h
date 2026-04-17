#ifndef SENT_HAL_STM32F042_H
#define SENT_HAL_STM32F042_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_STM32F042_RX_MAX_BATCH_SIZE SENT_MAX_TIMESTAMPS
#define SENT_STM32F042_RX_MAX_READY_BATCHES 3U
#define SENT_STM32F042_TX_MAX_INTERVALS SENT_MAX_INTERVALS
#define SENT_STM32F042_TX_MAX_TOGGLES   (SENT_MAX_INTERVALS * 2U) /* 2 toggle edges per interval */

typedef struct {
    uint32_t timer_clock_hz;
    uint16_t timer_autoreload;
    uint8_t capture_batch_size;    /* max SENT_STM32F042_RX_MAX_BATCH_SIZE (13) */
    uint8_t ready_queue_depth;     /* max SENT_STM32F042_RX_MAX_READY_BATCHES (2) */
    /* Sync-detection threshold [µs].  When a captured interval is >= this value
     * and active_count > 0, the current partial batch is discarded and a fresh
     * batch is started with the preceding edge (the sync falling-edge) as
     * timestamp[0].  Set to 0 to disable.
     * For MLX90377 at 3 µs tick: sync = 168 µs, max nibble = 81 µs → use 100 µs.
     * If two consecutive long intervals occur (long pause then sync), the second
     * long interval re-fires and corrects the alignment automatically. */
    uint32_t sync_min_us;
} sent_stm32f042_rx_config_t;

typedef struct {
    uint32_t timestamps_us[SENT_STM32F042_RX_MAX_BATCH_SIZE];
    uint8_t count;
} sent_stm32f042_rx_batch_t;

typedef struct {
    sent_stm32f042_rx_config_t config;
    volatile uint32_t overflow_count;
    volatile uint32_t dropped_batches;
    uint64_t last_counter_ticks;
    uint32_t last_timestamp_us;
    uint32_t ticks_to_us_mul_q12;
    uint32_t ticks_to_us_frac_q12;
    uint32_t ticks_to_us_max_delta;
    uint32_t active_timestamps_us[SENT_STM32F042_RX_MAX_BATCH_SIZE];
    sent_stm32f042_rx_batch_t ready_batches[SENT_STM32F042_RX_MAX_READY_BATCHES];
    volatile uint8_t ready_head;
    volatile uint8_t ready_tail;
    uint8_t active_count;
    bool running;
} sent_stm32f042_rx_hal_t;

typedef struct {
    uint16_t default_pause_ticks;
    uint8_t  low_ticks;   /* LOW-phase duration per interval [ticks]; SAE J2716 min = 5 */
} sent_stm32f042_tx_config_t;

typedef struct {
    sent_stm32f042_tx_config_t config;
    /* Pre-expanded toggle durations: [LOW, HIGH, LOW, HIGH, ...] for each SENT interval.
     * Written by stm32_tx_submit; read by ISR. count published last (M0 store barrier). */
    uint16_t intervals[SENT_STM32F042_TX_MAX_TOGGLES];
    volatile uint8_t count;   /* total toggle entries; 0 = idle */
    uint8_t active_index;     /* ISR-only: no volatile needed */
    bool running;
} sent_stm32f042_tx_hal_t;

void sent_stm32f042_rx_hal_init(sent_stm32f042_rx_hal_t* hal,
                                const sent_stm32f042_rx_config_t* config);
void sent_stm32f042_tx_hal_init(sent_stm32f042_tx_hal_t* hal,
                                const sent_stm32f042_tx_config_t* config);

void sent_stm32f042_make_rx_hal(sent_stm32f042_rx_hal_t* impl, sent_rx_hal_t* out_hal);
void sent_stm32f042_make_tx_hal(sent_stm32f042_tx_hal_t* impl, sent_tx_hal_t* out_hal);

void sent_stm32f042_rx_on_capture_edge_isr(sent_stm32f042_rx_hal_t* hal,
                                            uint16_t captured_counter);
void sent_stm32f042_rx_on_overflow_isr(sent_stm32f042_rx_hal_t* hal);
uint32_t sent_stm32f042_rx_dropped_batches(const sent_stm32f042_rx_hal_t* hal);

bool sent_stm32f042_tx_pop_next_interval_ticks_from_isr(sent_stm32f042_tx_hal_t* hal,
                                                         uint16_t* out_interval_ticks);
size_t sent_stm32f042_tx_pending_frames(const sent_stm32f042_tx_hal_t* hal);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_STM32F042_H */
