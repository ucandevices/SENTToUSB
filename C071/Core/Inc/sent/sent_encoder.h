/* SENT frame encoder: builds the interval and timestamp arrays used by the
 * HAL transmit path and by test fixtures that feed synthetic data into the decoder. */
#ifndef SENT_SENT_ENCODER_H
#define SENT_SENT_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Encode frame into an array of interval durations in ticks.
 * Order: [sync | status | data×N | CRC | pause (optional)].
 * pause_ticks: 12–768 to include a pause interval, 0 to omit.
 * out_interval_count is always set (to 0 on failure).
 * Returns false if config is invalid or any nibble value exceeds 0xF. */
bool sent_build_intervals_ticks(const sent_frame_t* frame,
                                const sent_config_t* config,
                                uint16_t pause_ticks,
                                uint16_t* out_intervals_ticks,
                                size_t* out_interval_count);

/* Convert a tick-based interval array to absolute edge timestamps [us].
 * timestamp[0] is always 0; subsequent entries accumulate from there.
 * tick_x10_us: tick period in 0.1-us units (e.g. 30 = 3.0 µs).
 * out_timestamp_count is always set; returns false on overflow or bad args. */
bool sent_intervals_to_timestamps_us(const uint16_t* intervals_ticks,
                                     size_t interval_count,
                                     uint16_t tick_x10_us,
                                     uint32_t* out_timestamps_us,
                                     size_t* out_timestamp_count);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_ENCODER_H */
