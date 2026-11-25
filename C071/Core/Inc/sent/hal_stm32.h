/* STM32 HAL backend for SENT RX and TX (STM32F042, STM32C071; one RX and one
 * TX instance per channel).
 *
 * RX: a 32-bit free-running TIMx input-capture ISR calls
 *     sent_stm32_rx_on_capture_edge_isr() on each FALLING edge (J2716
 *     measures falling edge to falling edge).  Raw counter values are queued
 *     in a ring; the main loop drains them via the poll_timestamps_us HAL hook
 *     and performs frame alignment and decoding at full timer resolution.
 *
 * TX: the main loop calls submit_frame which pre-expands the frame into a flat
 *     toggle-interval array.  The platform layer (sent_app.c) then uses DMA to
 *     reload the timer ARR on each update, with PWM mode driving the pin.
 *     sent_stm32_tx_pop_next_interval_ticks_from_isr() is retained for
 *     ISR-driven platforms that do not use DMA. */
#ifndef SENT_HAL_STM32_H
#define SENT_HAL_STM32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_STM32_RX_RING_SIZE         64U                      /* ISR->main edge ring; power of two */
#define SENT_STM32_TX_MAX_INTERVALS     SENT_MAX_INTERVALS        /* tick intervals per frame */
#define SENT_STM32_TX_MAX_TOGGLES       (SENT_MAX_INTERVALS * 2U) /* 2 toggle edges per interval */

/* Compile-time configuration passed to sent_stm32_rx_hal_init(). */
typedef struct {
    uint32_t timer_clock_hz;         /* input-capture timer frequency [Hz]; whole MHz */
} sent_stm32_rx_config_t;

/* Runtime state for one STM32 RX channel.  Single-producer (capture ISR) /
 * single-consumer (main loop) ring of raw capture values. */
typedef struct {
    sent_stm32_rx_config_t config;
    uint32_t edges[SENT_STM32_RX_RING_SIZE];  /* raw timer counts at each falling edge */
    volatile uint8_t head;                     /* ISR writes here */
    volatile uint8_t tail;                     /* main loop reads here */
    volatile uint32_t dropped_edges;           /* edges lost (ring full, over-capture); monotonic */
    volatile bool running;                     /* gates ISR processing */
} sent_stm32_rx_hal_t;

/* Compile-time configuration passed to sent_stm32_tx_hal_init(). */
typedef struct {
    uint16_t default_pause_ticks;    /* inter-frame gap [ticks] used when submit passes 0 */
    uint8_t  low_ticks;              /* LOW-phase duration per interval [ticks]; SAE J2716 min = 5 */
    uint16_t tx_tick_x10_us;         /* TX tick period [0.1 us units]; 30 = 3.0 us (SAE J2716 default) */
} sent_stm32_tx_config_t;

/* Runtime state for one STM32 TX channel. */
typedef struct {
    sent_stm32_tx_config_t config;
    /* Pre-expanded toggle durations: [LOW, HIGH, LOW, HIGH, ...] for each SENT interval.
     * Written by stm32_tx_submit; read by ISR. count published last (M0 store barrier). */
    uint16_t intervals[SENT_STM32_TX_MAX_TOGGLES];
    volatile uint8_t count;          /* total toggle entries; 0 = idle */
    uint8_t active_index;            /* ISR-only: index of the next toggle to pop */
    bool running;
} sent_stm32_tx_hal_t;

/* Initialize RX HAL with the given timer config; NULL uses 48 MHz defaults. */
void sent_stm32_rx_hal_init(sent_stm32_rx_hal_t* hal,
                                const sent_stm32_rx_config_t* config);
/* Initialize TX HAL with the given config; NULL uses SAE J2716 defaults. */
void sent_stm32_tx_hal_init(sent_stm32_tx_hal_t* hal,
                                const sent_stm32_tx_config_t* config);

/* Populate out_hal with STM32 RX function pointers backed by impl. */
void sent_stm32_make_rx_hal(sent_stm32_rx_hal_t* impl, sent_rx_hal_t* out_hal);
/* Populate out_hal with STM32 TX function pointers backed by impl. */
void sent_stm32_make_tx_hal(sent_stm32_tx_hal_t* impl, sent_tx_hal_t* out_hal);

/* ISR handler: store one captured falling-edge counter value.
 *
 * The capture timer must be a free-running 32-bit counter (ARR = 0xFFFFFFFF),
 * so no overflow bookkeeping is needed: consumers difference consecutive
 * values modulo 2^32.  Frame alignment is NOT done here; the raw edge stream
 * is handed to the main loop, which finds frames by structure (sync pulse
 * followed by the configured number of valid nibbles and a matching CRC).
 * Inline because the 4-channel capture ISR calls it for every edge.
 *
 * @param hal               RX HAL instance
 * @param captured_counter  raw 32-bit timer capture register value [timer counts] */
static inline void sent_stm32_rx_on_capture_edge_isr(sent_stm32_rx_hal_t* hal,
                                                     uint32_t captured_counter) {
    if (!hal->running) {
        return;
    }
    uint8_t head = hal->head;
    uint8_t next = (uint8_t)((head + 1U) & (SENT_STM32_RX_RING_SIZE - 1U));
    if (next == hal->tail) {
        hal->dropped_edges++;        /* main loop too slow: edge lost */
        return;
    }
    hal->edges[head] = captured_counter;
    hal->head = next;
}

/* ISR helper: edges the capture hardware lost (over-capture flag).  They
 * leave a hole in the stream exactly like a ring overflow. */
static inline void sent_stm32_rx_note_lost_edges_isr(sent_stm32_rx_hal_t* hal, uint32_t count) {
    if (hal->running) {
        hal->dropped_edges += count;
    }
}

/* Returns the number of edges lost so far.  Monotonic since init: the
 * consumer only compares it with the value it saw last. */
uint32_t sent_stm32_rx_dropped_edges(const sent_stm32_rx_hal_t* hal);
/* Pop the next interval for ISR-driven TX platforms (not used in DMA mode). */
bool sent_stm32_tx_pop_next_interval_ticks_from_isr(sent_stm32_tx_hal_t* hal,
                                                         uint16_t* out_interval_ticks);
/* Returns 1 if a frame is in progress, 0 if idle. */
size_t sent_stm32_tx_pending_frames(const sent_stm32_tx_hal_t* hal);
/* Returns the configured TX tick period [0.1 us units]. */
uint16_t sent_stm32_tx_get_tick_x10_us(const sent_stm32_tx_hal_t* hal);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_STM32_H */
