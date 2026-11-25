#include "test.h"

#include "sent/sent_protocol.h"

static sent_config_t make_valid_config(void) {
    sent_config_t c;
    c.data_nibbles = 6;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.pause_pulse_enabled = false;
    c.min_tick_x10_us = 25;
    c.max_tick_x10_us = 35;
    c.crc_init_seed = 0x05;
    c.crc_legacy = false;
    return c;
}

static void test_validate_config_accepts_valid(void) {
    sent_config_t c = make_valid_config();
    TEST_ASSERT_TRUE(sent_validate_config(&c));
}

static void test_validate_config_rejects_zero_nibbles(void) {
    sent_config_t c = make_valid_config();
    c.data_nibbles = 0;
    TEST_ASSERT_FALSE(sent_validate_config(&c));
}

static void test_validate_config_rejects_too_many_nibbles(void) {
    sent_config_t c = make_valid_config();
    c.data_nibbles = SENT_MAX_DATA_NIBBLES + 1;
    TEST_ASSERT_FALSE(sent_validate_config(&c));
}

static void test_validate_config_rejects_zero_min_tick(void) {
    sent_config_t c = make_valid_config();
    c.min_tick_x10_us = 0;
    TEST_ASSERT_FALSE(sent_validate_config(&c));
}

static void test_validate_config_rejects_zero_max_tick(void) {
    sent_config_t c = make_valid_config();
    c.max_tick_x10_us = 0;
    TEST_ASSERT_FALSE(sent_validate_config(&c));
}

static void test_validate_config_rejects_min_greater_than_max(void) {
    sent_config_t c = make_valid_config();
    c.min_tick_x10_us = 40;
    c.max_tick_x10_us = 30;
    TEST_ASSERT_FALSE(sent_validate_config(&c));
}

static void test_validate_config_accepts_min_equal_max(void) {
    sent_config_t c = make_valid_config();
    c.min_tick_x10_us = 30;
    c.max_tick_x10_us = 30;
    TEST_ASSERT_TRUE(sent_validate_config(&c));
}

static void test_pack_zero_count_returns_zero(void) {
    uint8_t n[1] = {0xF};
    TEST_ASSERT_EQ(sent_pack_nibbles(n, 0, SENT_NIBBLE_ORDER_MSB_FIRST), 0);
}

static void test_pack_msb_first(void) {
    uint8_t n[] = {0x1, 0x2, 0x3};
    uint32_t packed = sent_pack_nibbles(n, 3, SENT_NIBBLE_ORDER_MSB_FIRST);
    TEST_ASSERT_EQ(packed, 0x123U);
}

static void test_pack_lsb_first(void) {
    uint8_t n[] = {0x1, 0x2, 0x3};
    uint32_t packed = sent_pack_nibbles(n, 3, SENT_NIBBLE_ORDER_LSB_FIRST);
    TEST_ASSERT_EQ(packed, 0x321U);
}

static void test_pack_clamps_to_max(void) {
    uint8_t n[] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9};
    /* Counter > MAX should be silently clamped to MAX (=8). */
    uint32_t packed = sent_pack_nibbles(n, 9, SENT_NIBBLE_ORDER_MSB_FIRST);
    TEST_ASSERT_EQ(packed, 0x12345678U);
}

static void test_pack_masks_high_bits_per_nibble(void) {
    uint8_t n[] = {0xF1, 0xF2}; /* high bits should be ignored */
    uint32_t packed = sent_pack_nibbles(n, 2, SENT_NIBBLE_ORDER_MSB_FIRST);
    TEST_ASSERT_EQ(packed, 0x12U);
}

static void test_unpack_zero_count_fails(void) {
    uint8_t out[1];
    TEST_ASSERT_FALSE(sent_unpack_nibbles(0x123U, 0, SENT_NIBBLE_ORDER_MSB_FIRST, out));
}

static void test_unpack_too_many_fails(void) {
    uint8_t out[SENT_MAX_DATA_NIBBLES + 1];
    TEST_ASSERT_FALSE(sent_unpack_nibbles(0x123U, SENT_MAX_DATA_NIBBLES + 1,
                                          SENT_NIBBLE_ORDER_MSB_FIRST, out));
}

static void test_unpack_msb_first(void) {
    uint8_t out[3] = {0};
    TEST_ASSERT_TRUE(sent_unpack_nibbles(0x123U, 3, SENT_NIBBLE_ORDER_MSB_FIRST, out));
    TEST_ASSERT_EQ(out[0], 0x1);
    TEST_ASSERT_EQ(out[1], 0x2);
    TEST_ASSERT_EQ(out[2], 0x3);
}

static void test_unpack_lsb_first(void) {
    uint8_t out[3] = {0};
    TEST_ASSERT_TRUE(sent_unpack_nibbles(0x123U, 3, SENT_NIBBLE_ORDER_LSB_FIRST, out));
    TEST_ASSERT_EQ(out[0], 0x3);
    TEST_ASSERT_EQ(out[1], 0x2);
    TEST_ASSERT_EQ(out[2], 0x1);
}

static void test_pack_unpack_roundtrip_msb(void) {
    uint8_t in[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};
    uint8_t out[8] = {0};
    uint32_t packed = sent_pack_nibbles(in, 8, SENT_NIBBLE_ORDER_MSB_FIRST);
    TEST_ASSERT_TRUE(sent_unpack_nibbles(packed, 8, SENT_NIBBLE_ORDER_MSB_FIRST, out));
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQ(out[i], in[i]);
}

static void test_pack_unpack_roundtrip_lsb(void) {
    uint8_t in[8] = {0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0x0, 0x1};
    uint8_t out[8] = {0};
    uint32_t packed = sent_pack_nibbles(in, 8, SENT_NIBBLE_ORDER_LSB_FIRST);
    TEST_ASSERT_TRUE(sent_unpack_nibbles(packed, 8, SENT_NIBBLE_ORDER_LSB_FIRST, out));
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQ(out[i], in[i]);
}

void run_sent_protocol_tests(void);
void run_sent_protocol_tests(void) {
    TEST_RUN(test_validate_config_accepts_valid);
    TEST_RUN(test_validate_config_rejects_zero_nibbles);
    TEST_RUN(test_validate_config_rejects_too_many_nibbles);
    TEST_RUN(test_validate_config_rejects_zero_min_tick);
    TEST_RUN(test_validate_config_rejects_zero_max_tick);
    TEST_RUN(test_validate_config_rejects_min_greater_than_max);
    TEST_RUN(test_validate_config_accepts_min_equal_max);
    TEST_RUN(test_pack_zero_count_returns_zero);
    TEST_RUN(test_pack_msb_first);
    TEST_RUN(test_pack_lsb_first);
    TEST_RUN(test_pack_clamps_to_max);
    TEST_RUN(test_pack_masks_high_bits_per_nibble);
    TEST_RUN(test_unpack_zero_count_fails);
    TEST_RUN(test_unpack_too_many_fails);
    TEST_RUN(test_unpack_msb_first);
    TEST_RUN(test_unpack_lsb_first);
    TEST_RUN(test_pack_unpack_roundtrip_msb);
    TEST_RUN(test_pack_unpack_roundtrip_lsb);
}
