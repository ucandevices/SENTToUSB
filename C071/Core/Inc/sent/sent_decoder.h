/* SENT frame decoder: converts a batch of falling-edge timestamps into a
 * fully decoded sent_frame_t (status nibble, data nibbles, CRC, tick time).
 *
 * The decoder tolerates ±35% per-interval jitter and tries each possible
 * sync position within the timestamp window so that a leading extra edge
 * (e.g. from sync detection) does not prevent decoding. */
#ifndef SENT_SENT_DECODER_H
#define SENT_SENT_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Decode result; worst-case status is reported when no candidate succeeds. */
typedef enum {
    SENT_DECODE_OK          = 0,  /* frame decoded and CRC matched */
    SENT_DECODE_SYNC_ERROR  = 1,  /* no candidate had a valid sync pulse length */
    SENT_DECODE_CRC_ERROR   = 2,  /* at least one sync-valid candidate failed CRC */
    SENT_DECODE_SHAPE_ERROR = 3,  /* sync was valid but nibble intervals were out of range */
} sent_decode_status_t;

/* Decode one SENT frame from an array of absolute falling-edge timestamps.
 * units_per_us is the timestamp resolution in counts per microsecond (>= 1);
 * pass the raw capture-timer counts and its MHz clock to avoid the rounding
 * loss of a microsecond conversion (at a 3 µs tick, 1 µs is a third of a tick).
 * Timestamps may wrap (free-running 32-bit counter).
 * timestamp_count must be at least data_nibbles + 4 (sync + status + data + CRC + 1).
 * out_status is set even when the function returns false (worst-case error code).
 * Returns true and populates out_frame on success.  On SENT_DECODE_CRC_ERROR
 * out_frame holds the nibbles, received CRC and tick time of the last candidate
 * that failed the CRC. */
bool sent_decode_from_timestamps(const sent_config_t* config,
                                 const uint32_t* timestamps,
                                 size_t timestamp_count,
                                 uint32_t units_per_us,
                                 sent_frame_t* out_frame,
                                 sent_decode_status_t* out_status);

/* Cheap pre-check (two multiplies, no division): is an interval of `sync`
 * timestamp units a sync pulse inside config's [min_tick, max_tick] window?
 * This is exactly the sync test sent_decode_from_timestamps() applies to each
 * candidate.  A window of exactly data_nibbles + 4 timestamps has only one
 * candidate (its first interval), so a caller can skip the full decode when
 * this fails: the decoder would return false with SENT_DECODE_SYNC_ERROR. */
bool sent_decode_sync_in_range(const sent_config_t* config,
                               uint32_t sync,
                               uint32_t units_per_us);

/* Same as sent_decode_from_timestamps() with 1 µs timestamp resolution.
 * timestamp_count must be at least data_nibbles + 4 (sync + status + data + CRC + 1).
 * out_status is set even when the function returns false (worst-case error code).
 * Returns true and populates out_frame on success. */
bool sent_decode_from_timestamps_us(const sent_config_t* config,
                                    const uint32_t* timestamps_us,
                                    size_t timestamp_count,
                                    sent_frame_t* out_frame,
                                    sent_decode_status_t* out_status);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_DECODER_H */
