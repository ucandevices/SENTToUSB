/* Synthetic SENT edge streams for the application host tests. */
#ifndef SIM_SIGNAL_H
#define SIM_SIGNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "sent/sent_protocol.h"

#define SIM_UNITS_PER_US 48U          /* like the 48 MHz capture timer */
#define SIM_MAX_FRAMES   64U
#define SIM_MAX_EDGES    (SIM_MAX_FRAMES * SENT_MAX_TIMESTAMPS + 1U)

typedef struct {
    sent_config_t config;
    uint16_t tick_x10_us;    /* tick; tick * 48 / 10 must be a whole number of counts */
    uint16_t pause_ticks;    /* 0 = no pause pulse */
    uint32_t frames;
    uint32_t seed;
} sim_channel_t;

typedef struct {
    uint64_t t[SIM_MAX_EDGES];             /* virtual 64-bit time [counts] */
    size_t count;
    uint8_t status[SIM_MAX_FRAMES];
    uint8_t data[SIM_MAX_FRAMES][SENT_MAX_DATA_NIBBLES];
} sim_stream_t;

/* Encode sim->frames frames starting at virtual time t0. */
bool sim_generate(const sim_channel_t* sim, uint64_t t0, sim_stream_t* out);

/* Expected 0x510-style payload bytes (data nibbles packed MSB first plus the
 * info byte with the status nibble); returns the DLC. */
uint8_t sim_expected_payload(const sim_stream_t* s, uint32_t frame, uint8_t nibbles,
                             uint8_t* out_bytes);

/* One edge of several channels merged in time order. */
typedef struct {
    uint32_t t;      /* timestamp as the 32-bit timer sees it (wraps) */
    uint8_t ch;
} sim_edge_t;

/* Merge streams by time; returns the number of merged edges. */
size_t sim_merge(const sim_stream_t* streams, size_t n, sim_edge_t* out, size_t max);

#endif
