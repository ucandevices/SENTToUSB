/* SENT operating-mode tracker and runtime statistics store.
 *
 * Tracks whether the SENT interface is idle, receiving, or transmitting, and
 * accumulates diagnostic counters (frame count, CRC errors, etc.) that the
 * application can read without touching HAL internals. */
#ifndef SENT_MODE_MANAGER_H
#define SENT_MODE_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENT_MODE_STOPPED = 0,  /* HAL is idle; no capture or transmission in progress */
    SENT_MODE_RX      = 1,  /* input-capture active; frames being decoded */
    SENT_MODE_TX      = 2,  /* output timer active; frames being transmitted */
} sent_mode_t;

/* Cumulative diagnostic counters; zeroed by sent_mode_manager_init(). */
typedef struct {
    uint32_t frames_decoded;   /* successfully decoded frames since init */
    uint32_t crc_errors;       /* frames rejected by CRC check */
    uint32_t sync_errors;      /* frames rejected due to bad sync pulse length */
    uint32_t dropped_events;   /* timestamp batches lost because the queue was full */
} sent_runtime_stats_t;

typedef struct {
    sent_mode_t mode;
    sent_runtime_stats_t stats;
} sent_mode_manager_t;

/* Initialize manager to STOPPED and zero all statistics counters. */
void sent_mode_manager_init(sent_mode_manager_t* manager);
/* Transition to RX mode (does not start the HAL; caller does that separately). */
void sent_mode_manager_start_rx(sent_mode_manager_t* manager);
/* Transition to TX mode (does not start the HAL; caller does that separately). */
void sent_mode_manager_start_tx(sent_mode_manager_t* manager);
/* Transition to STOPPED (does not stop the HAL; caller does that separately). */
void sent_mode_manager_stop(sent_mode_manager_t* manager);

/* Returns the current mode; STOPPED when manager is NULL. */
static inline sent_mode_t sent_mode_manager_mode(const sent_mode_manager_t* manager) {
    return manager != NULL ? manager->mode : SENT_MODE_STOPPED;
}

static inline bool sent_mode_manager_is_rx(const sent_mode_manager_t* manager) {
    return sent_mode_manager_mode(manager) == SENT_MODE_RX;
}

static inline bool sent_mode_manager_is_tx(const sent_mode_manager_t* manager) {
    return sent_mode_manager_mode(manager) == SENT_MODE_TX;
}

#ifdef __cplusplus
}
#endif

#endif  /* SENT_MODE_MANAGER_H */
