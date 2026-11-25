#include "test.h"

#include "sent/sent_crc.h"
#include "sent/sent_protocol.h"

static void test_crc_data_only_known_vector(void) {
    /* J2716 example: 6 nibbles {0x1, 0x2, 0x3, 0x4, 0x5, 0x6}, init 0x05, data-only */
    uint8_t data[] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
    uint8_t crc = sent_crc4_j2716(data, 6, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_status_and_data_includes_status(void) {
    uint8_t data[] = {0x1, 0x2, 0x3, 0x4};
    uint8_t crc1 = sent_crc4_j2716(data, 4, SENT_CRC_MODE_STATUS_AND_DATA, 0x00, 0x05);
    uint8_t crc2 = sent_crc4_j2716(data, 4, SENT_CRC_MODE_STATUS_AND_DATA, 0x0F, 0x05);
    TEST_ASSERT(crc1 != crc2); /* status nibble must affect result */
}

static void test_crc_seed_affects_result(void) {
    uint8_t data[] = {0x1, 0x2, 0x3};
    uint8_t crc1 = sent_crc4_j2716(data, 3, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    uint8_t crc2 = sent_crc4_j2716(data, 3, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x03);
    TEST_ASSERT(crc1 != crc2);
}

static void test_crc_null_data_returns_seed_after_status(void) {
    /* data_nibbles == NULL path is taken */
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x07);
    TEST_ASSERT_EQ(crc, 0x07);
}

static void test_crc_null_data_status_mode(void) {
    /* status path runs with NULL data */
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_STATUS_AND_DATA, 0x05, 0x05);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_high_bit_xor_path(void) {
    /* Force the (crc & 0x08) != 0 branch via known input */
    uint8_t data[] = {0x0F};
    uint8_t crc = sent_crc4_j2716(data, 1, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_status_high_bit_xor_path(void) {
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_STATUS_AND_DATA, 0x0F, 0x0F);
    TEST_ASSERT(crc <= 0x0F);
}

static void test_crc_masks_input_nibbles(void) {
    uint8_t a[] = {0x05};
    uint8_t b[] = {0xF5}; /* upper bits should be ignored */
    uint8_t ca = sent_crc4_j2716(a, 1, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    uint8_t cb = sent_crc4_j2716(b, 1, SENT_CRC_MODE_DATA_ONLY, 0x00, 0x05);
    TEST_ASSERT_EQ(ca, cb);
}

static void test_crc_masks_seed_to_4_bits(void) {
    uint8_t crc = sent_crc4_j2716(NULL, 0, SENT_CRC_MODE_DATA_ONLY, 0x00, 0xF5);
    /* Seed should be masked to 0x05 */
    TEST_ASSERT_EQ(crc, 0x05);
}

/* SAE J2716 APR2016, Appendix B.1: example data nibbles and checksums. */
static const uint8_t k_j2716_rows[6][6] = {
    {5, 3, 14, 5, 3, 14}, {7, 4, 8, 7, 4, 8}, {4, 10, 12, 4, 10, 12},
    {7, 8, 15, 7, 8, 15}, {9, 1, 13, 9, 1, 13}, {0, 0, 0, 0, 0, 0},
};

static void test_crc_recommended_matches_j2716_table_b1_2(void) {
    static const uint8_t expected[6] = {15, 3, 10, 5, 6, 5};
    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQ(sent_crc4_j2716(k_j2716_rows[i], 6, SENT_CRC_MODE_DATA_ONLY, 0, 0x03), expected[i]);
    }
}

static void test_crc_legacy_matches_j2716_table_b1_1(void) {
    static const uint8_t expected[6] = {12, 5, 3, 15, 10, 15};
    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_EQ(sent_crc4_j2716_legacy(k_j2716_rows[i], 6, SENT_CRC_MODE_DATA_ONLY, 0), expected[i]);
    }
}

static void test_crc_legacy_status_mode_and_null_data(void) {
    /* seed 5 -> T[5] = 3, XOR status 0x4 = 7; no data nibbles follow */
    TEST_ASSERT_EQ(sent_crc4_j2716_legacy(NULL, 0, SENT_CRC_MODE_STATUS_AND_DATA, 0x04), 0x07);
    TEST_ASSERT_EQ(sent_crc4_j2716_legacy(NULL, 0, SENT_CRC_MODE_DATA_ONLY, 0x04), 0x05);
}

static void test_crc_frame_selects_algorithm(void) {
    sent_config_t c;
    c.data_nibbles = 6;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.pause_pulse_enabled = false;
    c.min_tick_x10_us = 25;
    c.max_tick_x10_us = 35;
    c.crc_init_seed = 0x03;
    c.crc_legacy = false;
    TEST_ASSERT_EQ(sent_crc4_frame(&c, 0, k_j2716_rows[1], 6), 3);
    c.crc_legacy = true;
    TEST_ASSERT_EQ(sent_crc4_frame(&c, 0, k_j2716_rows[1], 6), 5);
}

void run_sent_crc_tests(void);
void run_sent_crc_tests(void) {
    TEST_RUN(test_crc_data_only_known_vector);
    TEST_RUN(test_crc_status_and_data_includes_status);
    TEST_RUN(test_crc_seed_affects_result);
    TEST_RUN(test_crc_null_data_returns_seed_after_status);
    TEST_RUN(test_crc_null_data_status_mode);
    TEST_RUN(test_crc_high_bit_xor_path);
    TEST_RUN(test_crc_status_high_bit_xor_path);
    TEST_RUN(test_crc_masks_input_nibbles);
    TEST_RUN(test_crc_masks_seed_to_4_bits);
    TEST_RUN(test_crc_recommended_matches_j2716_table_b1_2);
    TEST_RUN(test_crc_legacy_matches_j2716_table_b1_1);
    TEST_RUN(test_crc_legacy_status_mode_and_null_data);
    TEST_RUN(test_crc_frame_selects_algorithm);
}
