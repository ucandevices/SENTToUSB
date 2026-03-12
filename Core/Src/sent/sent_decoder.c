#include "sent/sent_decoder.h"

#include "sent/sent_assert.h"
#include <string.h>

#include "sent/sent_crc.h"

#define SENT_SYNC_TICKS       56U   /* sync pulse duration in ticks */
#define SENT_NIBBLE_MIN_TICKS 12U   /* minimum data nibble ticks (J2716) */
#define SENT_NIBBLE_MAX_TICKS 27U   /* maximum data nibble ticks */
#define SENT_NIBBLE_OFFSET    12U   /* nibble value = ticks - OFFSET */
#define SENT_MAX_PAUSE_TICKS  768U  /* maximum pause pulse ticks */
#define SENT_TOLERANCE_NUM    35U   /* tick tolerance numerator (35%) */
#define SENT_TOLERANCE_DEN    100U

/* Convert intervals to nibble values and validate their shape.
 * @param intervals        interval array
 * @param offset           index of the first nibble interval within intervals[]
 * @param required_nibbles number of nibbles to decode
 * @param sync_us          sync pulse duration [us]
 * @param tolerance_us     per-interval tolerance [us]
 * @param out_nibbles      [out] decoded nibble values [0..15]
 * @return                 true if all nibbles are within spec and within tolerance */
static bool decode_nibble_array(const uint32_t* intervals,
                                 size_t offset,
                                 size_t required_nibbles,
                                 uint32_t sync_us,
                                 uint32_t tolerance_us,
                                 uint8_t* out_nibbles) {
    for (size_t j = 0U; j < required_nibbles; ++j) {
        uint32_t interval_us = intervals[offset + j];
        uint32_t scaled = interval_us * SENT_SYNC_TICKS;
        uint32_t nibble_ticks = (scaled + (sync_us / 2U)) / sync_us;

        /*
         * Tolerance check equivalent to:
         *   abs(real_ticks - rounded_ticks) <= 0.35
         * done in integer arithmetic:
         *   abs(scaled - rounded*sync) / sync <= 0.35
         */
        uint32_t expected = nibble_ticks * sync_us;
        uint32_t diff = (scaled >= expected) ? (scaled - expected) : (expected - scaled);
        if (diff > tolerance_us) {
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
 * @param pause_us      pause interval duration [us]
 * @param sync_us       sync pulse duration [us]
 * @param tolerance_us  per-interval tolerance [us]
 * @param out_ticks     [out] rounded pause duration [ticks]
 * @return              true if pause interval is within spec */
static bool decode_pause_interval(uint32_t pause_us,
                                   uint32_t sync_us,
                                   uint32_t tolerance_us,
                                   uint16_t* out_ticks) {
    uint32_t pause_scaled = pause_us * SENT_SYNC_TICKS;
    uint32_t pause_ticks = (pause_scaled + (sync_us / 2U)) / sync_us;
    uint32_t pause_expected = pause_ticks * sync_us;
    uint32_t pause_diff = (pause_scaled >= pause_expected)
                              ? (pause_scaled - pause_expected)
                              : (pause_expected - pause_scaled);
    if (pause_diff > tolerance_us ||
        pause_ticks < SENT_NIBBLE_MIN_TICKS || pause_ticks > SENT_MAX_PAUSE_TICKS) {
        return false;
    }
    *out_ticks = (uint16_t)pause_ticks;
    return true;
}

/* Decode a SENT frame from an array of absolute edge timestamps.
 * @param config           SENT protocol configuration
 * @param timestamps_us    array of falling-edge timestamps [us]
 * @param timestamp_count  number of timestamps (intervals + 1)
 * @param out_frame        [out] decoded SENT frame (status, data nibbles, CRC, tick time)
 * @param out_status       [out] decode result: OK, SYNC_ERROR, SHAPE_ERROR, or CRC_ERROR
 * @return                 true if a valid frame was decoded */
bool sent_decode_from_timestamps_us(const sent_config_t* config,
                                    const uint32_t* timestamps_us,
                                    size_t timestamp_count,
                                    sent_frame_t* out_frame,
                                    sent_decode_status_t* out_status) {
    if (out_status != NULL) {
        *out_status = SENT_DECODE_SYNC_ERROR;
    }
    SENT_ASSERT(config != NULL && timestamps_us != NULL && out_frame != NULL);
    if (!sent_validate_config(config)) {
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
        if (timestamps_us[i + 1U] <= timestamps_us[i]) {
            return false;
        }
        intervals[i] = timestamps_us[i + 1U] - timestamps_us[i];
    }

    sent_decode_status_t worst_status = SENT_DECODE_SYNC_ERROR;

    for (size_t start = 0U; start + required_intervals <= interval_count; ++start) {
        uint32_t sync_us = intervals[start];
        uint32_t sync_x10 = sync_us * 10U;
        uint32_t tolerance_us = (sync_us * SENT_TOLERANCE_NUM + (SENT_TOLERANCE_DEN - 1U)) / SENT_TOLERANCE_DEN;
        if (sync_x10 < (uint32_t)(SENT_SYNC_TICKS * config->min_tick_x10_us) ||
            sync_x10 > (uint32_t)(SENT_SYNC_TICKS * config->max_tick_x10_us)) {
            continue;
        }

        uint8_t nibbles[SENT_MAX_DATA_NIBBLES + 2U];
        if (!decode_nibble_array(intervals, start + 1U, required_nibbles,
                                 sync_us, tolerance_us, nibbles)) {
            if (worst_status < SENT_DECODE_SHAPE_ERROR) {
                worst_status = SENT_DECODE_SHAPE_ERROR;
            }
            continue;
        }

        uint8_t status = nibbles[0];
        uint8_t crc = nibbles[required_nibbles - 1U];
        uint8_t data_count = (uint8_t)config->data_nibbles;
        uint8_t data[SENT_MAX_DATA_NIBBLES] = {0};
        for (uint8_t i = 0U; i < data_count; ++i) {
            data[i] = nibbles[1U + i];
        }

        uint8_t computed_crc = sent_crc4_j2716(data, data_count, config->crc_mode, status);
        if (computed_crc != crc) {
            worst_status = SENT_DECODE_CRC_ERROR;
            continue;
        }

        memset(out_frame, 0, sizeof(*out_frame));
        out_frame->status = status;
        out_frame->data_nibbles_count = data_count;
        for (uint8_t i = 0U; i < data_count; ++i) {
            out_frame->data_nibbles[i] = data[i];
        }
        out_frame->crc = crc;
        out_frame->tick_x10_us = (uint16_t)((sync_x10 + 28U) / SENT_SYNC_TICKS);

        size_t pause_idx = start + required_intervals;
        if (pause_idx < interval_count) {
            uint16_t pause_ticks;
            if (decode_pause_interval(intervals[pause_idx], sync_us, tolerance_us, &pause_ticks)) {
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
