#ifndef SENT_SENT_ENCODER_H
#define SENT_SENT_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

bool sent_build_intervals_ticks(const sent_frame_t* frame,
                                const sent_config_t* config,
                                uint16_t pause_ticks,
                                uint16_t* out_intervals_ticks,
                                size_t* out_interval_count);

bool sent_intervals_to_timestamps_us(const uint16_t* intervals_ticks,
                                     size_t interval_count,
                                     uint16_t tick_x10_us,
                                     uint32_t* out_timestamps_us,
                                     size_t* out_timestamp_count);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_ENCODER_H */
