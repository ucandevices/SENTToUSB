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
#define SENT_STM32F042_RX_MAX_READY_BATCHES 2U
#define SENT_STM32F042_TX_MAX_INTERVALS SENT_MAX_INTERVALS
#define SENT_STM32F042_TX_QUEUE_DEPTH 4U

typedef struct {
    uint32_t timer_clock_hz;
    uint16_t timer_autoreload;
    uint8_t capture_batch_size;    /* max SENT_STM32F042_RX_MAX_BATCH_SIZE (24) */
    uint8_t ready_queue_depth;     /* max SENT_STM32F042_RX_MAX_READY_BATCHES (4) */
} sent_stm32f042_rx_config_t;

typedef struct {
    uint16_t intervals_ticks[SENT_STM32F042_TX_MAX_INTERVALS];
    uint8_t count;
} sent_stm32f042_tx_frame_intervals_t;

typedef struct {
    uint32_t timestamps_us[SENT_STM32F042_RX_MAX_BATCH_SIZE];
    uint8_t count;
} sent_stm32f042_rx_batch_t;

typedef struct {
    sent_stm32f042_rx_config_t config;
    volatile uint32_t overflow_count;
    volatile uint32_t dropped_batches;
    uint32_t last_counter_ticks;
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
    uint8_t max_pending_frames;    /* max SENT_STM32F042_TX_QUEUE_DEPTH (8) */
} sent_stm32f042_tx_config_t;

typedef struct {
    sent_stm32f042_tx_config_t config;
    sent_stm32f042_tx_frame_intervals_t frame_queue[SENT_STM32F042_TX_QUEUE_DEPTH];
    uint16_t active_intervals[SENT_STM32F042_TX_MAX_INTERVALS];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    uint8_t active_count;      /* ISR-only: no volatile needed */
    uint8_t active_index;      /* ISR-only: no volatile needed */
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
