#include "sent/sent_encoder.h"

#include "sent/sent_assert.h"

#include "sent/sent_crc.h"

/* Encode a SENT frame into an array of interval durations in tick units.
 * @param frame               SENT frame (status, data nibbles, CRC)
 * @param config              SENT protocol configuration
 * @param pause_ticks         pause pulse duration [ticks, 12-768, or 0 for none]
 * @param out_intervals_ticks [out] array of interval durations [ticks]
 * @param out_interval_count  [out] number of intervals written
 * @return                    true on success */
bool sent_build_intervals_ticks(const sent_frame_t* frame,
                                const sent_config_t* config,
                                uint16_t pause_ticks,
                                uint16_t* out_intervals_ticks,
                                size_t* out_interval_count) {
    SENT_ASSERT(out_interval_count != NULL);
    *out_interval_count = 0U;

    SENT_ASSERT(frame != NULL && config != NULL && out_intervals_ticks != NULL);
    if (!sent_validate_config(config)) {
        return false;
    }
    if (frame->data_nibbles_count != config->data_nibbles) {
        return false;
    }
    if (pause_ticks > 0U && (pause_ticks < 12U || pause_ticks > 768U)) {
        return false;
    }

    size_t needed = (size_t)config->data_nibbles + 3U + (pause_ticks > 0U ? 1U : 0U);
    if (needed > SENT_MAX_INTERVALS) {
        return false;
    }

    size_t w = 0U;
    out_intervals_ticks[w++] = 56U;
    out_intervals_ticks[w++] = (uint16_t)(12U + (frame->status & 0x0FU));

    for (uint8_t i = 0U; i < config->data_nibbles; ++i) {
        uint8_t nibble = frame->data_nibbles[i];
        if (nibble > 0x0FU) {
            return false;
        }
        out_intervals_ticks[w++] = (uint16_t)(12U + nibble);
    }

    uint8_t crc = sent_crc4_j2716(frame->data_nibbles,
                                  frame->data_nibbles_count,
                                  config->crc_mode,
                                  frame->status);
    out_intervals_ticks[w++] = (uint16_t)(12U + crc);

    if (pause_ticks > 0U) {
        out_intervals_ticks[w++] = pause_ticks;
    }

    *out_interval_count = w;
    return true;
}

/* Convert tick-based intervals into absolute timestamps in microseconds.
 * @param intervals_ticks    array of interval durations [ticks]
 * @param interval_count     number of intervals
 * @param tick_x10_us        tick duration [0.1 us units, e.g. 30 = 3.0 us]
 * @param out_timestamps_us  [out] array of absolute edge timestamps [us]
 * @param out_timestamp_count [out] number of timestamps written (intervals + 1)
 * @return                   true on success */
bool sent_intervals_to_timestamps_us(const uint16_t* intervals_ticks,
                                     size_t interval_count,
                                     uint16_t tick_x10_us,
                                     uint32_t* out_timestamps_us,
                                     size_t* out_timestamp_count) {
    SENT_ASSERT(out_timestamp_count != NULL);
    *out_timestamp_count = 0U;

    SENT_ASSERT(intervals_ticks != NULL && out_timestamps_us != NULL);
    if (tick_x10_us == 0U) {
        return false;
    }
    if (interval_count == 0U || interval_count > SENT_MAX_INTERVALS) {
        return false;
    }

    uint32_t t_x10 = 0U;
    out_timestamps_us[0] = 0U;
    for (size_t i = 0; i < interval_count; ++i) {
        uint32_t step_x10 = (uint32_t)intervals_ticks[i] * (uint32_t)tick_x10_us;
        if (step_x10 > (UINT32_MAX - t_x10)) {
            return false;
        }
        t_x10 += step_x10;
        out_timestamps_us[i + 1U] = (uint32_t)((t_x10 + 5U) / 10U);
    }

    *out_timestamp_count = interval_count + 1U;
    return true;
}
