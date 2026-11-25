#include "sent/sent_decoder.h"

#include "sent/sent_assert.h"

#include "sent/sent_crc.h"

/* Decoder-only tolerance window for tick-based interval validation. */
#define SENT_TOLERANCE_NUM    35U   /* tick tolerance numerator (35%) */
#define SENT_TOLERANCE_DEN    100U

/* Speed notes (4-channel firmware, Cortex-M0+ without a divide instruction):
 * a software 32-bit division costs far more than a multiply, and the v1.1
 * decoder did one per nibble, one for the tolerance and three for the pause.
 * Now each candidate sync costs ONE division (the reciprocal of the sync
 * length); every nibble and the pause are converted with a multiply by that
 * reciprocal plus an exact remainder correction, so the results are
 * bit-identical to the plain divisions.  The tolerance test is done by
 * multiplication too.  Before the reciprocal is computed, a shift/add-only
 * range check rejects windows whose intervals cannot be nibbles, so noise on
 * a line costs no division at all. */

/* Exact floor(x / d) with remainder, using recip = UINT32_MAX / d (d >= 1).
 * The estimate (x * recip) >> 32 is never above the true quotient and at most
 * 2 below it, so the loop runs at most twice. */
static uint32_t div_by_recip(uint32_t x, uint32_t d, uint32_t recip, uint32_t* out_rem) {
    uint32_t q = (uint32_t)(((uint64_t)x * recip) >> 32U);
    uint32_t r = x - q * d;
    while (r >= d) {
        r -= d;
        q++;
    }
    *out_rem = r;
    return q;
}

/* True when diff exceeds the per-interval tolerance ceil(sync * 35 / 100).
 * tol_x100 = sync * 35.  For an integer diff:
 *   diff > ceil(tol_x100 / 100)  <=>  (diff - 1) * 100 >= tol_x100.
 * diff is at most about sync, and sync <= UINT32_MAX / 100, so no overflow. */
static bool exceeds_tolerance(uint32_t diff, uint32_t tol_x100) {
    return diff != 0U && (diff - 1U) * SENT_TOLERANCE_DEN >= tol_x100;
}

/* Rounded sync-relative tick count of an interval: q = round(value * 56 / sync)
 * as floor((value * 56 + sync / 2) / sync).  out_diff receives
 * |value * 56 - q * sync|, the rounding error in units of 1/56 sync.
 * value * 56 + sync / 2 must fit 32 bits (callers guarantee value < sync and
 * sync <= UINT32_MAX / 100). */
static uint32_t ticks_of(uint32_t value, uint32_t sync, uint32_t recip, uint32_t* out_diff) {
    uint32_t half = sync / 2U;
    uint32_t r;
    uint32_t q = div_by_recip(value * SENT_SYNC_TICKS + half, sync, recip, &r);
    /* value*56 - q*sync = r - half */
    *out_diff = (r >= half) ? (r - half) : (half - r);
    return q;
}

/* Shift/add-only pre-check of the nibble intervals of one candidate.
 * Rejects only intervals the full check would reject as well:
 *   2 * interval >= sync  ->  >= 28 ticks, rounds to more than 27;
 *   5 * interval <  sync  ->  < 11.2 ticks, rounds to less than 12. */
static bool nibbles_plausible(const uint32_t* intervals, size_t count, uint32_t sync) {
    for (size_t j = 0U; j < count; ++j) {
        uint32_t iv = intervals[j];
        if (iv >= sync || (iv << 1U) >= sync || ((iv << 2U) + iv) < sync) {
            return false;
        }
    }
    return true;
}

/* Convert intervals to nibble values and validate their shape.
 * @param intervals        first nibble interval [time units]
 * @param required_nibbles number of nibbles to decode
 * @param sync             sync pulse duration [time units]
 * @param recip            UINT32_MAX / sync
 * @param tol_x100         sync * 35 (tolerance, see exceeds_tolerance())
 * @param out_nibbles      [out] decoded nibble values [0..15]
 * @return                 true if all nibbles are within spec and within tolerance */
static bool decode_nibble_array(const uint32_t* intervals,
                                 size_t required_nibbles,
                                 uint32_t sync,
                                 uint32_t recip,
                                 uint32_t tol_x100,
                                 uint8_t* out_nibbles) {
    for (size_t j = 0U; j < required_nibbles; ++j) {
        /* nibbles_plausible() already ensured interval < sync, which keeps
         * interval * 56 inside 32 bits. */
        uint32_t diff;
        uint32_t nibble_ticks = ticks_of(intervals[j], sync, recip, &diff);

        /* abs(real_ticks - rounded_ticks) <= 0.35, in integer arithmetic. */
        if (exceeds_tolerance(diff, tol_x100)) {
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
 * @param recip      UINT32_MAX / sync
 * @param tol_x100   sync * 35
 * @param out_ticks  [out] rounded pause duration [ticks]
 * @return           true if pause interval is within spec */
static bool decode_pause_interval(uint32_t pause,
                                   uint32_t sync,
                                   uint32_t recip,
                                   uint32_t tol_x100,
                                   uint16_t* out_ticks) {
    /* pause * 56 can exceed 32 bits (768-tick pause, long tick, fine timestamp
     * resolution), so split off whole sync lengths first.  The caller
     * guarantees sync <= UINT32_MAX / 100, hence rem * 56 fits. */
    uint32_t rem;
    uint32_t whole = div_by_recip(pause, sync, recip, &rem);
    if (whole > (SENT_MAX_PAUSE_TICKS / SENT_SYNC_TICKS)) {
        return false;
    }
    uint32_t pause_diff;
    uint32_t frac_ticks = ticks_of(rem, sync, recip, &pause_diff);
    uint32_t pause_ticks = whole * SENT_SYNC_TICKS + frac_ticks;
    if (exceeds_tolerance(pause_diff, tol_x100) ||
        pause_ticks < SENT_NIBBLE_MIN_TICKS || pause_ticks > SENT_MAX_PAUSE_TICKS) {
        return false;
    }
    *out_ticks = (uint16_t)pause_ticks;
    return true;
}

/* Is a sync pulse of this length inside the configured tick window?
 * Same test (and the same overflow guard) as the decoder applies to each
 * candidate; see sent_decoder.h. */
bool sent_decode_sync_in_range(const sent_config_t* config,
                               uint32_t sync,
                               uint32_t units_per_us) {
    SENT_ASSERT(config != NULL);
    if (sync > (UINT32_MAX / 100U)) {
        return false;
    }
    uint32_t sync_x10 = sync * 10U;
    return sync_x10 >= SENT_SYNC_TICKS * (uint32_t)config->min_tick_x10_us * units_per_us &&
           sync_x10 <= SENT_SYNC_TICKS * (uint32_t)config->max_tick_x10_us * units_per_us;
}

/* Decode a SENT frame from an array of absolute edge timestamps.
 * @param config           SENT protocol configuration
 * @param timestamps       array of falling-edge timestamps [1/units_per_us µs]
 * @param timestamp_count  number of timestamps (intervals + 1)
 * @param units_per_us     timestamp resolution: counts per microsecond (>= 1)
 * @param out_frame        [out] decoded SENT frame (status, data nibbles, CRC, tick time)
 * @param out_status       [out] decode result: OK, SYNC_ERROR, SHAPE_ERROR, or CRC_ERROR
 * @return                 true if a valid frame was decoded */
bool sent_decode_from_timestamps(const sent_config_t* config,
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

    sent_decode_status_t worst_status = SENT_DECODE_SYNC_ERROR;

    for (size_t start = 0U; start + required_intervals <= interval_count; ++start) {
        uint32_t sync = intervals[start];
        if (!sent_decode_sync_in_range(config, sync, units_per_us)) {
            continue;
        }
        uint8_t nibbles[SENT_MAX_DATA_NIBBLES + 2U];
        uint32_t recip = 0U;
        uint32_t tol_x100 = sync * SENT_TOLERANCE_NUM;
        bool shape_ok = nibbles_plausible(&intervals[start + 1U], required_nibbles, sync);
        if (shape_ok) {
            recip = UINT32_MAX / sync;   /* the one division for this candidate */
            shape_ok = decode_nibble_array(&intervals[start + 1U], required_nibbles,
                                           sync, recip, tol_x100, nibbles);
        }
        if (!shape_ok) {
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
        out_frame->tick_x10_us = (uint16_t)((sync * 10U + (tick_div / 2U)) / tick_div);
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
            if (decode_pause_interval(intervals[pause_idx], sync, recip, tol_x100, &pause_ticks)) {
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
bool sent_decode_from_timestamps_us(const sent_config_t* config,
                                    const uint32_t* timestamps_us,
                                    size_t timestamp_count,
                                    sent_frame_t* out_frame,
                                    sent_decode_status_t* out_status) {
    return sent_decode_from_timestamps(config, timestamps_us, timestamp_count, 1U,
                                       out_frame, out_status);
}
