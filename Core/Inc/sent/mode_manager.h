#ifndef SENT_MODE_MANAGER_H
#define SENT_MODE_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENT_MODE_STOPPED = 0,
    SENT_MODE_RX = 1,
    SENT_MODE_TX = 2,
} sent_mode_t;

typedef struct {
    uint32_t frames_decoded;
    uint32_t crc_errors;
    uint32_t sync_errors;
    uint32_t dropped_events;
} sent_runtime_stats_t;

typedef struct {
    sent_mode_t mode;
    sent_runtime_stats_t stats;
} sent_mode_manager_t;

void sent_mode_manager_init(sent_mode_manager_t* manager);
void sent_mode_manager_start_rx(sent_mode_manager_t* manager);
void sent_mode_manager_start_tx(sent_mode_manager_t* manager);
void sent_mode_manager_stop(sent_mode_manager_t* manager);

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
