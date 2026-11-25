#include "test.h"

#include "sent/sent_crc.h"
#include "sent/sent_decoder.h"
#include "sent/sent_encoder.h"
#include "sent/sent_protocol.h"

static sent_config_t make_config(uint8_t nibbles) {
    sent_config_t c;
    c.data_nibbles = nibbles;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.pause_pulse_enabled = false;
    c.min_tick_x10_us = 25;
    c.max_tick_x10_us = 35;
    c.crc_init_seed = 0x05;
    c.crc_legacy = false;
    return c;
}

/* Encode a frame end-to-end into timestamps so the decoder has a real signal. */
static bool encode_to_timestamps(const sent_config_t* c,
                                 const sent_frame_t* f,
                                 uint16_t pause_ticks,
                                 uint16_t tick_x10,
                                 uint32_t* ts,
                                 size_t* ts_count) {
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t int_count = 0;
    if (!sent_build_intervals_ticks(f, c, pause_ticks, intervals, &int_count)) return false;
    return sent_intervals_to_timestamps_us(intervals, int_count, tick_x10, ts, ts_count);
}

static sent_frame_t make_frame(const sent_config_t* c) {
    sent_frame_t f;
    f.status = 0x00;
    f.data_nibbles_count = c->data_nibbles;
    for (uint8_t i = 0; i < c->data_nibbles; ++i) f.data_nibbles[i] = (uint8_t)((i + 1) & 0xF);
    f.crc = 0;
    f.tick_x10_us = 30;
    f.has_pause = false;
    f.pause_ticks = 0;
    return f;
}

static void test_decode_roundtrip_no_pause(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &ts_count));

    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    TEST_ASSERT_EQ(out.data_nibbles_count, 6);
    for (uint8_t i = 0; i < 6; ++i) TEST_ASSERT_EQ(out.data_nibbles[i], f.data_nibbles[i]);
    TEST_ASSERT_FALSE(out.has_pause);
}

static void test_decode_roundtrip_with_pause(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 50, 30, ts, &ts_count));

    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    TEST_ASSERT_TRUE(out.has_pause);
    TEST_ASSERT_EQ(out.pause_ticks, 50);
}

static void test_decode_invalid_config_fails(void) {
    sent_config_t c = make_config(6);
    c.data_nibbles = 0;
    uint32_t ts[2] = {0, 100};
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 2, &out, &st));
}

static void test_decode_too_few_timestamps(void) {
    sent_config_t c = make_config(6);
    uint32_t ts[3] = {0, 100, 200};
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 3, &out, &st));
}

static void test_decode_too_many_timestamps(void) {
    sent_config_t c = make_config(6);
    uint32_t ts[SENT_MAX_TIMESTAMPS + 1];
    for (size_t i = 0; i < SENT_MAX_TIMESTAMPS + 1; ++i) ts[i] = (uint32_t)(i * 100);
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, SENT_MAX_TIMESTAMPS + 1, &out, &st));
}

static void test_decode_non_increasing_timestamps(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &ts_count));
    /* Corrupt: make a timestamp non-monotonic */
    ts[2] = ts[1];
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
}

static void test_decode_sync_out_of_range(void) {
    /* All sync candidates are outside the configured tick range → SYNC_ERROR. */
    sent_config_t c = make_config(6);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    /* All intervals = 1 us, way too short for any plausible sync. */
    for (size_t i = 1; i < SENT_MAX_TIMESTAMPS; ++i) ts[i] = (uint32_t)i;
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 10, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SYNC_ERROR);
}

static void test_decode_crc_error(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &ts_count));
    /* Mutate the second-to-last interval (CRC nibble) by adding a tick.
     * ts_count = intervals + 1. CRC interval = ts[ts_count-1] - ts[ts_count-2].
     * Add 30us (= 1 tick) to the final timestamp. */
    /* Add exactly 1 tick (3 µs at tick_x10=30) — shifts CRC nibble by one value
     * so it stays within [12,27] ticks but no longer matches computed CRC. */
    ts[ts_count - 1] += 3;
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_CRC_ERROR);
}

static void test_decode_shape_error_invalid_nibble_ticks(void) {
    /* Build a sync that's in range, but the next "nibble" is way outside [12,27]. */
    sent_config_t c = make_config(6);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    /* Intervals (us with tick=3.0us): sync=168 (56 ticks), then nibble = 5 ticks (= 15us, way too short). */
    uint32_t cursor = 0;
    ts[0] = 0;
    ts[1] = (cursor += 168);
    /* Make all subsequent intervals too short to be valid nibbles. */
    for (size_t i = 2; i < 10; ++i) {
        ts[i] = (cursor += 15);
    }
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 10, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);
}

static void test_decode_skips_over_garbage_to_find_frame(void) {
    /* Prepend a garbage interval before a valid frame; decoder should slide past. */
    sent_config_t c = make_config(4);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    size_t ts_count = 0;
    /* Encode a real frame */
    uint32_t real[SENT_MAX_TIMESTAMPS];
    size_t real_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, real, &real_count));

    /* Prepend one bogus interval (non-sync) before the real timestamps */
    ts[0] = 0;
    ts[1] = 50; /* leading garbage interval */
    /* Offset by 1 so ts[2] > ts[1]: real[0]==0 would otherwise equal ts[1]. */
    for (size_t i = 0; i < real_count; ++i) {
        ts[2 + i] = ts[1] + 1 + real[i];
    }
    ts_count = real_count + 2;
    if (ts_count > SENT_MAX_TIMESTAMPS) ts_count = SENT_MAX_TIMESTAMPS;

    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_SYNC_ERROR;
    if (ts_count <= SENT_MAX_TIMESTAMPS) {
        TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
        TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    }
}

static void test_decode_null_status_pointer_ok(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &ts_count));

    sent_frame_t out;
    /* out_status == NULL should be tolerated */
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, NULL));
}

static void test_decode_invalid_pause_drops_pause_flag(void) {
    sent_config_t c = make_config(4);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    /* Build with a valid pause, then corrupt the pause interval to be too short. */
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 50, 30, ts, &ts_count));
    /* Last interval = ts[count-1] - ts[count-2]. Shrink it below MIN_PAUSE. */
    if (ts_count >= 2) {
        ts[ts_count - 1] = ts[ts_count - 2] + 1; /* 1us "pause" — well outside spec */
    }
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    TEST_ASSERT_FALSE(out.has_pause);
}

static void test_decode_tolerance_exceeded_shape_error(void) {
    /* tick=2µs (tick_x10=20): sync=112µs, tolerance=40 (in scaled units).
     * Nibble at 29µs → rounds to 15 ticks, diff=|29*56 - 15*112|=56 > 40.
     * This specifically exercises the diff > tolerance_us branch (line 39). */
    sent_config_t c;
    c.data_nibbles = 4;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.pause_pulse_enabled = false;
    c.min_tick_x10_us = 20;
    c.max_tick_x10_us = 20;
    c.crc_init_seed = 0x05;
    c.crc_legacy = false;

    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    uint32_t t = 0;
    ts[0] = (t = 0);
    ts[1] = (t += 112);  /* sync = 56 ticks × 2µs = 112µs — in valid range */
    for (size_t i = 2; i < 9; ++i) {
        ts[i] = (t += 29);  /* 29µs per nibble: diff=56 > tolerance=40 → fail */
    }

    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 9, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);
}

/* Line 41: nibble_ticks > SENT_NIBBLE_MAX_TICKS (28 > 27) while passing tolerance check.
 * sync=168µs, nibble=84µs → ticks=28 exactly on boundary, diff=0 ≤ tolerance, but >MAX. */
static void test_decode_nibble_ticks_too_large(void) {
    sent_config_t c = make_config(4);
    /* Manually build timestamps: sync=168, then a nibble at 84 (28 ticks > 27). */
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    uint32_t t = 0;
    ts[0] = (t = 0);
    ts[1] = (t += 168);  /* sync = 56 ticks × 3µs = 168µs — in [25*56/10, 35*56/10]*10 = [140,196] ✓ */
    for (size_t i = 2; i < 9; ++i) ts[i] = (t += 84);  /* each nibble = 28 ticks > 27 → SHAPE_ERROR */
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 9, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);
}

/* Lines 64-66: ternary else branch (pause_expected - pause_scaled) when pause_us is
 * slightly below a tick boundary.  sync=168µs, pause_us=149 → 50 ticks but
 * pause_scaled(8344) < pause_expected(8400) → else branch taken, diff=56 ≤ tol=59 → valid. */
static void test_decode_pause_interval_below_expected(void) {
    sent_config_t c = make_config(4);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    /* Encode with pause, tick=30 (3µs/tick), then shrink pause by 1µs so
     * pause_scaled < pause_expected but still within tolerance. */
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 50, 30, ts, &ts_count));
    /* pause interval was 150µs (50 ticks); change it to 149µs. */
    ts[ts_count - 1] = ts[ts_count - 2] + 149;
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    TEST_ASSERT_TRUE(out.has_pause);
    TEST_ASSERT_EQ(out.pause_ticks, 50);
}

/* Line 65: pause_diff > tolerance — pause interval far from any tick boundary.
 * tick_x10=20 (2µs/tick), sync=112µs, tolerance=40, pause_us=41:
 * pause_scaled=2296 < pause_expected=2352, diff=56 > 40 → decode_pause_interval false. */
static void test_decode_pause_diff_above_tolerance(void) {
    sent_config_t c;
    c.data_nibbles = 4;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.pause_pulse_enabled = false;
    c.min_tick_x10_us = 20;
    c.max_tick_x10_us = 20;
    c.crc_init_seed = 0x05;
    c.crc_legacy = false;
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    /* Encode without pause at tick_x10=20 (2µs/tick, sync=112µs). */
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 20, ts, &ts_count));
    /* Append a pause interval of 41µs — diff=56 > tolerance=40 → invalid pause. */
    ts[ts_count] = ts[ts_count - 1] + 41;
    ts_count++;
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    TEST_ASSERT_FALSE(out.has_pause);
}

/* Line 66: pause_ticks > SENT_MAX_PAUSE_TICKS (769 > 768).
 * sync=168µs, pause_us=2307 → pause_ticks=769 > 768 → decode_pause_interval false. */
static void test_decode_pause_ticks_above_max(void) {
    sent_config_t c = make_config(4);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    /* Encode without pause at tick_x10=30 (3µs/tick, sync=168µs). */
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &ts_count));
    /* Append 769-tick pause = 769×3µs = 2307µs — exceeds SENT_MAX_PAUSE_TICKS=768. */
    ts[ts_count] = ts[ts_count - 1] + 2307;
    ts_count++;
    sent_frame_t out;
    sent_decode_status_t st;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, ts_count, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_OK);
    TEST_ASSERT_FALSE(out.has_pause);
}

/* Line 118: sync_x10 > max branch — all intervals are 200µs (> 35*56/10=196µs max),
 * so every start position is skipped via the sync-too-large continue → SYNC_ERROR. */
static void test_decode_sync_above_max(void) {
    sent_config_t c = make_config(4);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    for (size_t i = 0; i < SENT_MAX_TIMESTAMPS; ++i) ts[i] = (uint32_t)(i * 200);
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 9, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SYNC_ERROR);
}

/* Line 125: worst_status >= SEND_DECODE_SHAPE_ERROR already — the update-to-SHAPE_ERROR
 * branch is skipped.  Two sliding windows both have valid syncs (168µs = 56 ticks ✓)
 * but oversized nibbles (168µs → 56 ticks > 27 → SHAPE_ERROR).  Second window hits
 * `if (worst_status < SHAPE_ERROR)` with worst already SHAPE_ERROR → branch is FALSE. */
static void test_decode_second_shape_error_skips_update(void) {
    sent_config_t c = make_config(4);
    /* Timestamps: all intervals = 168µs → every sync is valid but every nibble fails. */
    uint32_t ts[10];
    for (size_t i = 0; i < 10; ++i) ts[i] = (uint32_t)(i * 168);
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    /* 9 timestamps = 8 intervals; start=0 and start=1 both tried, both SHAPE_ERROR. */
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, 9, &out, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);
}

/* Line 169: out_status NULL in the failure path — decoder returns false and must not
 * crash when out_status is NULL. */
static void test_decode_failure_null_out_status(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t ts_count = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &ts_count));
    ts[ts_count - 1] += 3;  /* Corrupt CRC: 1 extra tick, same technique as test_decode_crc_error */
    /* Pass NULL out_status — must not crash and must return false. */
    TEST_ASSERT_FALSE(sent_decode_from_timestamps_us(&c, ts, ts_count, &f, NULL));
}

/* Build timestamps in units of 1/units_per_us µs for an arbitrary tick time
 * [ns], starting at an arbitrary counter value (may wrap). */
static size_t fine_timestamps(const sent_config_t* c, const sent_frame_t* f,
                              uint16_t pause_ticks, uint32_t tick_ns, uint32_t units_per_us,
                              uint32_t start, uint32_t* ts) {
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t n = 0;
    if (!sent_build_intervals_ticks(f, c, pause_ticks, intervals, &n)) return 0;
    uint64_t t_ns = 0;
    ts[0] = start;
    for (size_t i = 0; i < n; ++i) {
        t_ns += (uint64_t)intervals[i] * tick_ns;
        ts[i + 1] = start + (uint32_t)((t_ns * units_per_us + 500U) / 1000U);
    }
    return n + 1;
}

static void test_decode_fine_resolution_off_nominal_tick(void) {
    /* 2.56 µs and 3.1 µs ticks lose frames with 1 µs timestamps; at 48 counts
     * per µs every nibble value must decode. */
    static const uint32_t ticks_ns[] = {2560, 2900, 3100, 3600, 90000};
    for (size_t k = 0; k < sizeof(ticks_ns) / sizeof(ticks_ns[0]); ++k) {
        sent_config_t c = make_config(6);
        c.crc_init_seed = 0x03;
        c.min_tick_x10_us = 20;
        c.max_tick_x10_us = 1130;
        sent_frame_t f = make_frame(&c);
        f.data_nibbles[0] = 0x0; f.data_nibbles[1] = 0xF; f.data_nibbles[2] = 0x7;
        f.data_nibbles[3] = 0x8; f.data_nibbles[4] = 0xF; f.data_nibbles[5] = 0x0;
        f.status = 0xA;
        uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
        size_t n = fine_timestamps(&c, &f, 0, ticks_ns[k], 48, 12345U, ts);
        sent_frame_t out;
        sent_decode_status_t st = SENT_DECODE_SYNC_ERROR;
        TEST_ASSERT_TRUE(sent_decode_from_timestamps(&c, ts, n, 48, &out, &st));
        TEST_ASSERT_EQ(st, SENT_DECODE_OK);
        TEST_ASSERT_EQ(out.status, 0xA);
        for (uint8_t i = 0; i < 6; ++i) TEST_ASSERT_EQ(out.data_nibbles[i], f.data_nibbles[i]);
        TEST_ASSERT_EQ(out.tick_x10_us, (uint16_t)((ticks_ns[k] + 50U) / 100U));
    }
}

static void test_decode_counter_wraparound(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    /* start 2000 counts before the 32-bit counter wraps */
    size_t n = fine_timestamps(&c, &f, 0, 3000, 48, 0xFFFFFFFFU - 2000U, ts);
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_SYNC_ERROR;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps(&c, ts, n, 48, &out, &st));
    TEST_ASSERT_EQ(out.data_nibbles[5], f.data_nibbles[5]);
}

static void test_decode_long_pause_fine_resolution(void) {
    /* 768-tick pause at a 90 µs tick: 3.3 million counts, more than 32 bits
     * once multiplied by 56. */
    sent_config_t c = make_config(6);
    c.min_tick_x10_us = 20;
    c.max_tick_x10_us = 1130;
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    size_t n = fine_timestamps(&c, &f, 768, 90000, 48, 0, ts);
    sent_frame_t out;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps(&c, ts, n, 48, &out, NULL));
    TEST_ASSERT_TRUE(out.has_pause);
    TEST_ASSERT_EQ(out.pause_ticks, 768);
}

static void test_decode_crc_error_reports_nibbles(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    size_t n = fine_timestamps(&c, &f, 0, 3000, 48, 0, ts);
    c.crc_legacy = true;   /* decode with a different CRC than it was built with */
    sent_frame_t out;
    sent_decode_status_t st = SENT_DECODE_OK;
    bool ok = sent_decode_from_timestamps(&c, ts, n, 48, &out, &st);
    if (!ok) {
        TEST_ASSERT_EQ(st, SENT_DECODE_CRC_ERROR);
        TEST_ASSERT_EQ(out.data_nibbles_count, 6);
        for (uint8_t i = 0; i < 6; ++i) TEST_ASSERT_EQ(out.data_nibbles[i], f.data_nibbles[i]);
    }
    TEST_ASSERT_FALSE(sent_decode_from_timestamps(&c, ts, n, 0, &out, &st));   /* zero resolution */
}

static void test_decode_legacy_crc_roundtrip(void) {
    sent_config_t c = make_config(6);
    c.crc_legacy = true;
    sent_frame_t f = make_frame(&c);
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    size_t n = 0;
    TEST_ASSERT_TRUE(encode_to_timestamps(&c, &f, 0, 30, ts, &n));
    sent_frame_t out;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps_us(&c, ts, n, &out, NULL));
    TEST_ASSERT_EQ(out.crc, sent_crc4_j2716_legacy(f.data_nibbles, 6, SENT_CRC_MODE_DATA_ONLY, 0));
}

void run_sent_decoder_tests(void);
void run_sent_decoder_tests(void) {
    TEST_RUN(test_decode_roundtrip_no_pause);
    TEST_RUN(test_decode_roundtrip_with_pause);
    TEST_RUN(test_decode_invalid_config_fails);
    TEST_RUN(test_decode_too_few_timestamps);
    TEST_RUN(test_decode_too_many_timestamps);
    TEST_RUN(test_decode_non_increasing_timestamps);
    TEST_RUN(test_decode_sync_out_of_range);
    TEST_RUN(test_decode_crc_error);
    TEST_RUN(test_decode_shape_error_invalid_nibble_ticks);
    TEST_RUN(test_decode_skips_over_garbage_to_find_frame);
    TEST_RUN(test_decode_null_status_pointer_ok);
    TEST_RUN(test_decode_invalid_pause_drops_pause_flag);
    TEST_RUN(test_decode_tolerance_exceeded_shape_error);
    TEST_RUN(test_decode_nibble_ticks_too_large);
    TEST_RUN(test_decode_pause_interval_below_expected);
    TEST_RUN(test_decode_pause_diff_above_tolerance);
    TEST_RUN(test_decode_pause_ticks_above_max);
    TEST_RUN(test_decode_sync_above_max);
    TEST_RUN(test_decode_second_shape_error_skips_update);
    TEST_RUN(test_decode_failure_null_out_status);
    TEST_RUN(test_decode_fine_resolution_off_nominal_tick);
    TEST_RUN(test_decode_counter_wraparound);
    TEST_RUN(test_decode_long_pause_fine_resolution);
    TEST_RUN(test_decode_crc_error_reports_nibbles);
    TEST_RUN(test_decode_legacy_crc_roundtrip);
}
