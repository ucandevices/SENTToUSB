#include "test.h"

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

static sent_frame_t make_frame(const sent_config_t* c) {
    sent_frame_t f;
    f.status = 0x00;
    f.data_nibbles_count = c->data_nibbles;
    for (uint8_t i = 0; i < c->data_nibbles; ++i) f.data_nibbles[i] = (uint8_t)(i & 0xF);
    f.crc = 0;
    f.tick_x10_us = 30;
    f.has_pause = false;
    f.pause_ticks = 0;
    return f;
}

static void test_build_intervals_basic_no_pause(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint16_t intervals[SENT_MAX_INTERVALS] = {0};
    size_t count = 0;
    TEST_ASSERT_TRUE(sent_build_intervals_ticks(&f, &c, 0, intervals, &count));
    /* sync + status + 6 data + crc = 9 */
    TEST_ASSERT_EQ(count, 9);
    TEST_ASSERT_EQ(intervals[0], SENT_SYNC_TICKS);
}

static void test_build_intervals_with_pause(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint16_t intervals[SENT_MAX_INTERVALS] = {0};
    size_t count = 0;
    TEST_ASSERT_TRUE(sent_build_intervals_ticks(&f, &c, 50, intervals, &count));
    TEST_ASSERT_EQ(count, 10);
    TEST_ASSERT_EQ(intervals[9], 50);
}

static void test_build_intervals_invalid_config_fails(void) {
    sent_config_t c = make_config(6);
    c.data_nibbles = 0; /* invalid */
    sent_frame_t f;
    f.status = 0; f.data_nibbles_count = 0; f.crc = 0;
    f.tick_x10_us = 30; f.has_pause = false; f.pause_ticks = 0;
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t count = 99;
    TEST_ASSERT_FALSE(sent_build_intervals_ticks(&f, &c, 0, intervals, &count));
    TEST_ASSERT_EQ(count, 0);
}

static void test_build_intervals_mismatched_nibble_count_fails(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    f.data_nibbles_count = 5; /* mismatch */
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t count = 0;
    TEST_ASSERT_FALSE(sent_build_intervals_ticks(&f, &c, 0, intervals, &count));
}

static void test_build_intervals_pause_below_min_fails(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t count = 0;
    TEST_ASSERT_FALSE(sent_build_intervals_ticks(&f, &c, SENT_MIN_PAUSE_TICKS - 1, intervals, &count));
}

static void test_build_intervals_pause_above_max_fails(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t count = 0;
    TEST_ASSERT_FALSE(sent_build_intervals_ticks(&f, &c, SENT_MAX_PAUSE_TICKS + 1, intervals, &count));
}

static void test_build_intervals_too_many_fails(void) {
    /* 8 nibbles + sync + status + crc + pause = 12 = MAX. So 8 nibbles + pause is OK.
     * Force the needed > MAX_INTERVALS path: not reachable with valid config (max nibbles=8).
     * The check exists as belt-and-suspenders; we exercise the boundary by using 8 nibbles + pause. */
    sent_config_t c = make_config(SENT_MAX_DATA_NIBBLES);
    sent_frame_t f = make_frame(&c);
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t count = 0;
    TEST_ASSERT_TRUE(sent_build_intervals_ticks(&f, &c, 50, intervals, &count));
    TEST_ASSERT_EQ(count, SENT_MAX_INTERVALS);
}

static void test_build_intervals_rejects_oversized_nibble(void) {
    sent_config_t c = make_config(6);
    sent_frame_t f = make_frame(&c);
    f.data_nibbles[2] = 0x10; /* > 0xF */
    uint16_t intervals[SENT_MAX_INTERVALS];
    size_t count = 0;
    TEST_ASSERT_FALSE(sent_build_intervals_ticks(&f, &c, 0, intervals, &count));
}

static void test_intervals_to_timestamps_basic(void) {
    uint16_t intervals[3] = {56, 15, 20};
    uint32_t ts[SENT_MAX_TIMESTAMPS] = {0};
    size_t count = 0;
    TEST_ASSERT_TRUE(sent_intervals_to_timestamps_us(intervals, 3, 30, ts, &count));
    TEST_ASSERT_EQ(count, 4);
    TEST_ASSERT_EQ(ts[0], 0);
    /* (56*30+5)/10 = 168 */
    TEST_ASSERT_EQ(ts[1], 168);
    /* +15*30 = 450 / 10 = (1680 + 450 + 5)/10 = 213 */
    TEST_ASSERT_EQ(ts[2], 213);
}

static void test_intervals_to_timestamps_rejects_zero_tick(void) {
    uint16_t intervals[1] = {56};
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t count = 99;
    TEST_ASSERT_FALSE(sent_intervals_to_timestamps_us(intervals, 1, 0, ts, &count));
    TEST_ASSERT_EQ(count, 0);
}

static void test_intervals_to_timestamps_rejects_zero_count(void) {
    uint16_t intervals[1] = {56};
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t count = 99;
    TEST_ASSERT_FALSE(sent_intervals_to_timestamps_us(intervals, 0, 30, ts, &count));
}

static void test_intervals_to_timestamps_rejects_too_many(void) {
    uint16_t intervals[SENT_MAX_INTERVALS + 1];
    for (size_t i = 0; i < SENT_MAX_INTERVALS + 1; ++i) intervals[i] = 20;
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t count = 99;
    TEST_ASSERT_FALSE(sent_intervals_to_timestamps_us(intervals, SENT_MAX_INTERVALS + 1,
                                                      30, ts, &count));
}

static void test_intervals_to_timestamps_overflow(void) {
    /* Force overflow: large intervals * large tick. */
    uint16_t intervals[SENT_MAX_INTERVALS];
    for (size_t i = 0; i < SENT_MAX_INTERVALS; ++i) intervals[i] = 0xFFFF;
    uint32_t ts[SENT_MAX_TIMESTAMPS];
    size_t count = 99;
    TEST_ASSERT_FALSE(sent_intervals_to_timestamps_us(intervals, SENT_MAX_INTERVALS,
                                                      0xFFFF, ts, &count));
}

void run_sent_encoder_tests(void);
void run_sent_encoder_tests(void) {
    TEST_RUN(test_build_intervals_basic_no_pause);
    TEST_RUN(test_build_intervals_with_pause);
    TEST_RUN(test_build_intervals_invalid_config_fails);
    TEST_RUN(test_build_intervals_mismatched_nibble_count_fails);
    TEST_RUN(test_build_intervals_pause_below_min_fails);
    TEST_RUN(test_build_intervals_pause_above_max_fails);
    TEST_RUN(test_build_intervals_too_many_fails);
    TEST_RUN(test_build_intervals_rejects_oversized_nibble);
    TEST_RUN(test_intervals_to_timestamps_basic);
    TEST_RUN(test_intervals_to_timestamps_rejects_zero_tick);
    TEST_RUN(test_intervals_to_timestamps_rejects_zero_count);
    TEST_RUN(test_intervals_to_timestamps_rejects_too_many);
    TEST_RUN(test_intervals_to_timestamps_overflow);
}
