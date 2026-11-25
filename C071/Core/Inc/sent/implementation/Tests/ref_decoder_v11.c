/* TEST-ONLY reference: the v1.1 decoder, kept verbatim (functions renamed) so
 * test_sent_decoder_equiv.c can check that the optimised decoder in
 * sent_decoder.c gives bit-identical results.  Not built into firmware. */
#include "sent/sent_decoder.h"

#include "sent/sent_assert.h"

#include "sent/sent_crc.h"

/* Decoder-only tolerance window for tick-based interval validation. */
#define SENT_TOLERANCE_NUM    35U   /* tick tolerance numerator (35%) */
#define SENT_TOLERANCE_DEN    100U

/* Convert intervals to nibble values and validate their shape.
 * @param intervals        interval array [time units]
 * @param offset           index of the first nibble interval within intervals[]
 * @param required_nibbles number of nibbles to decode
 * @param sync             sync pulse duration [time units]
 * @param tolerance        per-interval tolerance [time units]
 * @param out_nibbles      [out] decoded nibble values [0..15]
 * @return                 true if all nibbles are within spec and within tolerance */
static bool decode_nibble_array(const uint32_t* intervals,
                                 size_t offset,
                                 size_t required_nibbles,
                                 uint32_t sync,
                                 uint32_t tolerance,
                                 uint8_t* out_nibbles) {
    for (size_t j = 0U; j < required_nibbles; ++j) {
        uint32_t interval = intervals[offset + j];
        /* A nibble is at most 27 ticks, the sync is 56: anything as long as the
         * sync cannot be a nibble.  This also keeps interval * 56 inside 32 bits. */
        if (interval >= sync) {
            return false;
        }
        uint32_t scaled = interval * SENT_SYNC_TICKS;
        uint32_t nibble_ticks = (scaled + (sync / 2U)) / sync;

        /*
         * Tolerance check equivalent to:
         *   abs(real_ticks - rounded_ticks) <= 0.35
         * done in integer arithmetic:
         *   abs(scaled - rounded*sync) / sync <= 0.35
         */
        uint32_t expected = nibble_ticks * sync;
        uint32_t diff = (scaled >= expected) ? (scaled - expected) : (expected - scaled);
        if (diff > tolerance) {
            return false;
        }
        if (nibble_ticks < SENT_NIBBLE_MIN_TICKS || nibble_ticks > SENT_NIBBLE_MAX_TICKS) {
            return false;
        }
        out_nibbles[j] = (uint8_t)(nibble_ticks - SENT_NIBBLE_OFFSET);
    }
    return true;
}

/* Validate a pause interval and extract its tick count.
 * @param pause      pause interval duration [time units]
 * @param sync       sync pulse duration [time units]
 * @param tolerance  per-interval tolerance [time units]
 * @param out_ticks  [out] rounded pause duration [ticks]
 * @return           true if pause interval is within spec */
static bool decode_pause_interval(uint32_t pause,
                                   uint32_t sync,
                                   uint32_t tolerance,
                                   uint16_t* out_ticks) {
    /* pause * 56 can exceed 32 bits (768-tick pause, long tick, fine timestamp
     * resolution), so split off whole sync lengths first.  The caller
     * guarantees sync <= UINT32_MAX / 100, hence rem * 56 fits. */
    uint32_t whole = pause / sync;
    if (whole > (SENT_MAX_PAUSE_TICKS / SENT_SYNC_TICKS)) {
        return false;
    }
    uint32_t rem_scaled = (pause % sync) * SENT_SYNC_TICKS;
    uint32_t frac_ticks = (rem_scaled + (sync / 2U)) / sync;
    uint32_t pause_ticks = whole * SENT_SYNC_TICKS + frac_ticks;
    uint32_t expected = frac_ticks * sync;
    uint32_t pause_diff = (rem_scaled >= expected) ? (rem_scaled - expected)
                                                   : (expected - rem_scaled);
    if (pause_diff > tolerance ||
        pause_ticks < SENT_NIBBLE_MIN_TICKS || pause_ticks > SENT_MAX_PAUSE_TICKS) {
        return false;
    }
    *out_ticks = (uint16_t)pause_ticks;
    return true;
}

/* Decode a SENT frame from an array of absolute edge timestamps.
 * @param config           SENT protocol configuration
 * @param timestamps       array of falling-edge timestamps [1/units_per_us µs]
 * @param timestamp_count  number of timestamps (intervals + 1)
 * @param units_per_us     timestamp resolution: counts per microsecond (>= 1)
 * @param out_frame        [out] decoded SENT frame (status, data nibbles, CRC, tick time)
 * @param out_status       [out] decode result: OK, SYNC_ERROR, SHAPE_ERROR, or CRC_ERROR
 * @return                 true if a valid frame was decoded */
bool ref_v11_decode_from_timestamps(const sent_config_t* config,
                                 const uint32_t* timestamps,
                                 size_t timestamp_count,
                                 uint32_t units_per_us,
                                 sent_frame_t* out_frame,
                                 sent_decode_status_t* out_status) {
    if (out_status != NULL) {
        *out_status = SENT_DECODE_SYNC_ERROR;
    }
    SENT_ASSERT(config != NULL && timestamps != NULL && out_frame != NULL);
    if (!sent_validate_config(config) || units_per_us == 0U) {
        return false;
    }

    size_t required_intervals = (size_t)config->data_nibbles + 3U;
    size_t required_nibbles = (size_t)config->data_nibbles + 2U;
    if (timestamp_count < required_intervals + 1U) {
        return false;
    }
    if (timestamp_count > SENT_MAX_TIMESTAMPS) {
        return false;
    }

    uint32_t intervals[SENT_MAX_INTERVALS];
    size_t interval_count = timestamp_count - 1U;
    for (size_t i = 0; i < interval_count; ++i) {
        /* Modular subtraction: a free-running counter may wrap between two
         * edges.  Zero or "negative" steps are rejected. */
        uint32_t delta = timestamps[i + 1U] - timestamps[i];
        if (delta == 0U || delta > 0x7FFFFFFFU) {
            return false;
        }
        intervals[i] = delta;
    }

    /* Valid sync length window [time units x 10]. */
    uint32_t sync_min_x10 = SENT_SYNC_TICKS * (uint32_t)config->min_tick_x10_us * units_per_us;
    uint32_t sync_max_x10 = SENT_SYNC_TICKS * (uint32_t)config->max_tick_x10_us * units_per_us;

    sent_decode_status_t worst_status = SENT_DECODE_SYNC_ERROR;

    for (size_t start = 0U; start + required_intervals <= interval_count; ++start) {
        uint32_t sync = intervals[start];
        if (sync > (UINT32_MAX / 100U)) {
            continue;  /* far longer than any sync; also guards the products below */
        }
        uint32_t sync_x10 = sync * 10U;
        if (sync_x10 < sync_min_x10 || sync_x10 > sync_max_x10) {
            continue;
        }
        uint32_t tolerance = (sync * SENT_TOLERANCE_NUM + (SENT_TOLERANCE_DEN - 1U)) / SENT_TOLERANCE_DEN;

        uint8_t nibbles[SENT_MAX_DATA_NIBBLES + 2U];
        if (!decode_nibble_array(intervals, start + 1U, required_nibbles,
                                 sync, tolerance, nibbles)) {
            if (worst_status < SENT_DECODE_SHAPE_ERROR) {
                worst_status = SENT_DECODE_SHAPE_ERROR;
            }
            continue;
        }

        uint8_t status = nibbles[0];
        uint8_t crc = nibbles[required_nibbles - 1U];
        uint8_t data_count = (uint8_t)config->data_nibbles;

        /* Populate the frame before the CRC verdict so that callers which
         * probe several CRC variants (learn mode) can read the raw nibbles of
         * a CRC_ERROR result. */
        out_frame->status = status;
        out_frame->data_nibbles_count = data_count;
        for (uint8_t i = 0U; i < SENT_MAX_DATA_NIBBLES; ++i) {
            out_frame->data_nibbles[i] = (i < data_count) ? nibbles[1U + i] : 0U;
        }
        out_frame->crc = crc;
        uint32_t tick_div = SENT_SYNC_TICKS * units_per_us;
        out_frame->tick_x10_us = (uint16_t)((sync_x10 + (tick_div / 2U)) / tick_div);
        out_frame->has_pause = false;
        out_frame->pause_ticks = 0U;

        uint8_t computed_crc = sent_crc4_frame(config, status, out_frame->data_nibbles, data_count);
        if (computed_crc != crc) {
            worst_status = SENT_DECODE_CRC_ERROR;
            continue;
        }

        size_t pause_idx = start + required_intervals;
        if (pause_idx < interval_count) {
            uint16_t pause_ticks;
            if (decode_pause_interval(intervals[pause_idx], sync, tolerance, &pause_ticks)) {
                out_frame->has_pause = true;
                out_frame->pause_ticks = pause_ticks;
            }
        }

        if (out_status != NULL) {
            *out_status = SENT_DECODE_OK;
        }
        return true;
    }

    if (out_status != NULL) {
        *out_status = worst_status;
    }
    return false;
}

/* Decode from timestamps with 1 µs resolution (see sent_decode_from_timestamps). */
bool ref_v11_decode_from_timestamps_us(const sent_config_t* config,
                                    const uint32_t* timestamps_us,
                                    size_t timestamp_count,
                                    sent_frame_t* out_frame,
                                    sent_decode_status_t* out_status) {
    return ref_v11_decode_from_timestamps(config, timestamps_us, timestamp_count, 1U,
                                       out_frame, out_status);
}
