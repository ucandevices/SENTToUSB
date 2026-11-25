#include "test.h"

#include "sent/sent_crc.h"
#include "sent/sent_slow_channel.h"

static uint8_t status_from_bits(uint8_t b2, uint8_t b3) {
    return (uint8_t)(((b2 & 1U) << 2U) | ((b3 & 1U) << 3U));
}

static void build_esm12_statuses(uint8_t id, uint16_t data, uint8_t statuses[18]) {
    uint8_t b2[19] = {0};
    uint8_t b3[19] = {0};

    for (uint8_t frame = 1U; frame <= 6U; ++frame) b3[frame] = 1U;
    for (uint8_t frame = 7U; frame <= 18U; ++frame) {
        b2[frame] = (uint8_t)((data >> (18U - frame)) & 1U);
    }
    b3[8] = 0U;
    b3[9] = (uint8_t)((id >> 7U) & 1U);
    b3[10] = (uint8_t)((id >> 6U) & 1U);
    b3[11] = (uint8_t)((id >> 5U) & 1U);
    b3[12] = (uint8_t)((id >> 4U) & 1U);
    b3[13] = 0U;
    b3[14] = (uint8_t)((id >> 3U) & 1U);
    b3[15] = (uint8_t)((id >> 2U) & 1U);
    b3[16] = (uint8_t)((id >> 1U) & 1U);
    b3[17] = (uint8_t)(id & 1U);
    b3[18] = 0U;

    uint32_t crc_message = 0U;
    for (uint8_t frame = 7U; frame <= 18U; ++frame) {
        crc_message = (uint32_t)((crc_message << 1U) | b2[frame]);
        crc_message = (uint32_t)((crc_message << 1U) | b3[frame]);
    }
    uint8_t crc = sent_crc6_esm_j2716(crc_message);
    for (uint8_t frame = 1U; frame <= 6U; ++frame) {
        b2[frame] = (uint8_t)((crc >> (6U - frame)) & 1U);
    }

    for (uint8_t frame = 1U; frame <= 18U; ++frame) {
        statuses[frame - 1U] = status_from_bits(b2[frame], b3[frame]);
    }
}

static void test_esm12_decodes_mlx_default_shape(void) {
    sent_slow_channel_t ctx;
    sent_slow_message_t msg;
    uint8_t statuses[18];
    sent_slow_channel_init(&ctx);
    build_esm12_statuses(0x23U, 0x0A5CU, statuses);

    TEST_ASSERT_FALSE(sent_slow_channel_process_status(&ctx, status_from_bits(0U, 0U), &msg));
    for (uint8_t i = 0U; i < 17U; ++i) {
        TEST_ASSERT_FALSE(sent_slow_channel_process_status(&ctx, statuses[i], &msg));
    }
    TEST_ASSERT_TRUE(sent_slow_channel_process_status(&ctx, statuses[17], &msg));
    TEST_ASSERT_EQ(msg.format, SENT_SLOW_FORMAT_ENHANCED_12_8);
    TEST_ASSERT_EQ(msg.message_id, 0x23U);
    TEST_ASSERT_EQ(msg.data, 0x0A5CU);
}

static void test_esm12_crc_error_is_discarded(void) {
    sent_slow_channel_t ctx;
    sent_slow_message_t msg;
    uint8_t statuses[18];
    sent_slow_channel_init(&ctx);
    build_esm12_statuses(0x01U, 0x0801U, statuses);
    statuses[0] ^= 0x04U;

    TEST_ASSERT_FALSE(sent_slow_channel_process_status(&ctx, status_from_bits(0U, 0U), &msg));
    for (uint8_t i = 0U; i < 18U; ++i) {
        TEST_ASSERT_FALSE(sent_slow_channel_process_status(&ctx, statuses[i], &msg));
    }
}

static void test_short_serial_decodes(void) {
    sent_slow_channel_t ctx;
    sent_slow_message_t msg;
    sent_slow_channel_init(&ctx);

    uint8_t id = 0x6U;
    uint8_t data = 0xA5U;
    uint8_t nibbles[3] = {id, 0x0AU, 0x05U};
    uint8_t crc = sent_crc4_j2716(nibbles, 3U, SENT_CRC_MODE_DATA_ONLY, 0U, 0x03U);
    uint16_t bits = (uint16_t)(((uint16_t)id << 12U) | ((uint16_t)data << 4U) | crc);

    for (uint8_t i = 0U; i < 15U; ++i) {
        uint8_t bit = (uint8_t)((bits >> (15U - i)) & 1U);
        uint8_t start = (i == 0U) ? 1U : 0U;
        TEST_ASSERT_FALSE(sent_slow_channel_process_status(&ctx, status_from_bits(bit, start), &msg));
    }
    TEST_ASSERT_TRUE(sent_slow_channel_process_status(&ctx, status_from_bits((uint8_t)(bits & 1U), 0U), &msg));
    TEST_ASSERT_EQ(msg.format, SENT_SLOW_FORMAT_SHORT);
    TEST_ASSERT_EQ(msg.message_id, id);
    TEST_ASSERT_EQ(msg.data, data);
}

/* Feed one short serial message whose CRC was built by crc_of(). */
static bool feed_short(sent_slow_channel_t* ctx, uint8_t crc, sent_slow_message_t* msg) {
    uint16_t bits = (uint16_t)((0x6U << 12U) | (0xA5U << 4U) | crc);
    bool done = false;
    for (uint8_t i = 0U; i < 16U; ++i) {
        uint8_t bit = (uint8_t)((bits >> (15U - i)) & 1U);
        done = sent_slow_channel_process_status(ctx, status_from_bits(bit, (i == 0U) ? 1U : 0U), msg);
    }
    return done;
}

static void test_short_serial_crc_follows_configuration(void) {
    const uint8_t nibbles[3] = {0x6U, 0xAU, 0x5U};
    uint8_t rec = sent_crc4_j2716(nibbles, 3U, SENT_CRC_MODE_DATA_ONLY, 0U, 0x03U);
    uint8_t var = sent_crc4_j2716(nibbles, 3U, SENT_CRC_MODE_DATA_ONLY, 0U, 0x05U);
    uint8_t leg = sent_crc4_j2716_legacy(nibbles, 3U, SENT_CRC_MODE_DATA_ONLY, 0U);
    sent_slow_channel_t ctx;
    sent_slow_message_t msg;

    sent_slow_channel_init(&ctx);                 /* default: recommended CRC */
    TEST_ASSERT_TRUE(feed_short(&ctx, rec, &msg));
    TEST_ASSERT_FALSE(feed_short(&ctx, var, &msg));

    sent_slow_channel_set_crc(&ctx, 0x05U, false);
    TEST_ASSERT_TRUE(feed_short(&ctx, var, &msg));
    TEST_ASSERT_FALSE(feed_short(&ctx, rec, &msg));

    sent_slow_channel_set_crc(&ctx, 0x05U, true);
    sent_slow_channel_reset(&ctx);                /* reset must keep the CRC choice */
    TEST_ASSERT_TRUE(feed_short(&ctx, leg, &msg));
    TEST_ASSERT_EQ(msg.message_id, 0x6U);
    TEST_ASSERT_EQ(msg.data, 0xA5U);

    sent_slow_channel_set_crc(NULL, 0x03U, false);   /* tolerated */
}

void run_sent_slow_channel_tests(void);
void run_sent_slow_channel_tests(void) {
    TEST_RUN(test_esm12_decodes_mlx_default_shape);
    TEST_RUN(test_esm12_crc_error_is_discarded);
    TEST_RUN(test_short_serial_decodes);
    TEST_RUN(test_short_serial_crc_follows_configuration);
}
