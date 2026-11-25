#include "test.h"

#include "sent/mode_manager.h"

static void test_init_sets_stopped_and_zeros_stats(void) {
    sent_mode_manager_t m;
    m.stats.frames_decoded = 0xDEAD;
    m.stats.crc_errors = 0xBEEF;
    sent_mode_manager_init(&m);
    TEST_ASSERT_EQ(sent_mode_manager_mode(&m), SENT_MODE_STOPPED);
    TEST_ASSERT_EQ(m.stats.frames_decoded, 0);
    TEST_ASSERT_EQ(m.stats.crc_errors, 0);
    TEST_ASSERT_EQ(m.stats.sync_errors, 0);
    TEST_ASSERT_EQ(m.stats.dropped_events, 0);
    TEST_ASSERT_FALSE(sent_mode_manager_is_rx(&m));
    TEST_ASSERT_FALSE(sent_mode_manager_is_tx(&m));
}

static void test_start_rx_transitions(void) {
    sent_mode_manager_t m;
    sent_mode_manager_init(&m);
    sent_mode_manager_start_rx(&m);
    TEST_ASSERT_EQ(sent_mode_manager_mode(&m), SENT_MODE_RX);
    TEST_ASSERT_TRUE(sent_mode_manager_is_rx(&m));
    TEST_ASSERT_FALSE(sent_mode_manager_is_tx(&m));
}

static void test_start_tx_transitions(void) {
    sent_mode_manager_t m;
    sent_mode_manager_init(&m);
    sent_mode_manager_start_tx(&m);
    TEST_ASSERT_EQ(sent_mode_manager_mode(&m), SENT_MODE_TX);
    TEST_ASSERT_FALSE(sent_mode_manager_is_rx(&m));
    TEST_ASSERT_TRUE(sent_mode_manager_is_tx(&m));
}

static void test_stop_returns_to_stopped(void) {
    sent_mode_manager_t m;
    sent_mode_manager_init(&m);
    sent_mode_manager_start_rx(&m);
    sent_mode_manager_stop(&m);
    TEST_ASSERT_EQ(sent_mode_manager_mode(&m), SENT_MODE_STOPPED);
}

static void test_mode_helper_handles_null(void) {
    TEST_ASSERT_EQ(sent_mode_manager_mode(NULL), SENT_MODE_STOPPED);
    TEST_ASSERT_FALSE(sent_mode_manager_is_rx(NULL));
    TEST_ASSERT_FALSE(sent_mode_manager_is_tx(NULL));
}

void run_mode_manager_tests(void);
void run_mode_manager_tests(void) {
    TEST_RUN(test_init_sets_stopped_and_zeros_stats);
    TEST_RUN(test_start_rx_transitions);
    TEST_RUN(test_start_tx_transitions);
    TEST_RUN(test_stop_returns_to_stopped);
    TEST_RUN(test_mode_helper_handles_null);
}
