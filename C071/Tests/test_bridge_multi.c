/* Four sent_bridge_t instances fed with interleaved synthetic edge streams:
 * different tick times, nibble counts, CRC variants and pause pulses per
 * channel, one 32-bit timer view (wraps during the test), shared learn
 * storage.  No hardware, no sent_core: this checks that the bridges are fully
 * independent per instance. */
#include "test.h"

#include "sent_bridge.h"
#include "sim_signal.h"

#define NCH 4U

static sent_bridge_t g_b[NCH];
static sent_bridge_learn_t g_learn;
static sim_stream_t g_streams[NCH];
static sim_edge_t g_edges[NCH * SIM_MAX_EDGES];

/* RX HAL that only reports the timestamp resolution; edges are fed directly. */
static sent_rx_hal_t rx_hal_48mhz(void) {
    sent_rx_hal_t h;
    memset(&h, 0, sizeof(h));
    h.ticks_per_us = SIM_UNITS_PER_US;
    return h;
}

static sent_config_t cfg(uint8_t nibbles, sent_crc_mode_t mode, uint8_t seed, bool legacy) {
    sent_config_t c;
    memset(&c, 0, sizeof(c));
    c.data_nibbles = nibbles;
    c.crc_mode = mode;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.min_tick_x10_us = SENT_BRIDGE_LEARN_MIN_TICK_X10;
    c.max_tick_x10_us = SENT_BRIDGE_LEARN_MAX_TICK_X10;
    c.crc_init_seed = seed;
    c.crc_legacy = legacy;
    return c;
}

static const sim_channel_t k_sim[NCH] = {
    /* config                                                    tick  pause frames seed */
    {{6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false},       30,  12, 60, 11},
    {{4, SENT_CRC_MODE_STATUS_AND_DATA, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 50,   0, 60, 22},
    {{3, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x05, false},      125, 300, 30, 33},
    {{8, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x05, true},       900,  20, 12, 44},
};

static void setup(bool learn_storage) {
    memset(&g_learn, 0, sizeof(g_learn));
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        sent_rx_hal_t rx = rx_hal_48mhz();
        sent_config_t c = cfg(k_sim[ch].config.data_nibbles, k_sim[ch].config.crc_mode,
                              k_sim[ch].config.crc_init_seed, k_sim[ch].config.crc_legacy);
        sent_bridge_init(&g_b[ch], &c, &rx, NULL);
        sent_bridge_set_channel(&g_b[ch], ch);
        if (learn_storage) {
            sent_bridge_set_learn_storage(&g_b[ch], &g_learn);
        }
    }
}

/* Generate all four streams, starting just before the 32-bit wrap. */
static size_t make_edges(void) {
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        if (!sim_generate(&k_sim[ch], 0xFFF00000ULL + 1000ULL * ch, &g_streams[ch])) {
            return 0U;
        }
    }
    return sim_merge(g_streams, NCH, g_edges, sizeof(g_edges) / sizeof(g_edges[0]));
}

static void test_four_bridges_interleaved_decode_all_frames(void) {
    setup(true);
    size_t n = make_edges();
    TEST_ASSERT(n > 1000U);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_TRUE(sent_bridge_start_rx(&g_b[ch]));
    }
    uint32_t got[NCH] = {0};
    for (size_t i = 0U; i < n; ++i) {
        uint8_t ch = g_edges[i].ch;
        sent_can_frame_t f;
        if (sent_bridge_on_edge(&g_b[ch], g_edges[i].t, &f)) {
            TEST_ASSERT_EQ(f.id, 0x510U + 0x40U * ch);
            uint8_t exp[SENT_CAN_MAX_DLC];
            uint8_t dlc = sim_expected_payload(&g_streams[ch], got[ch],
                                               k_sim[ch].config.data_nibbles, exp);
            TEST_ASSERT_EQ(f.dlc, dlc);
            for (uint8_t b = 0U; b < dlc; ++b) {
                TEST_ASSERT_EQ(f.data[b], exp[b]);
            }
            got[ch]++;
        }
    }
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(got[ch], k_sim[ch].frames);
        TEST_ASSERT_EQ(g_b[ch].mode_manager.stats.frames_decoded, k_sim[ch].frames);
        TEST_ASSERT_EQ(g_b[ch].mode_manager.stats.crc_errors, 0);
        TEST_ASSERT_EQ(g_b[ch].mode_manager.stats.sync_errors, 0);
    }
}

/* Random edges on channel 1 must not disturb the other three. */
static void test_noisy_channel_does_not_disturb_others(void) {
    setup(true);
    size_t n = make_edges();
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_TRUE(sent_bridge_start_rx(&g_b[ch]));
    }
    uint32_t got[NCH] = {0};
    uint32_t rng = 1U;
    uint32_t noise_t = 0xFFF00000U;
    for (size_t i = 0U; i < n; ++i) {
        uint8_t ch = g_edges[i].ch;
        uint32_t t = g_edges[i].t;
        if (ch == 1U) {
            rng = rng * 1664525U + 1013904223U;
            noise_t += 50U + (rng >> 20);       /* 1..88 µs apart, random */
            t = noise_t;
        }
        sent_can_frame_t f;
        if (sent_bridge_on_edge(&g_b[ch], t, &f)) {
            got[ch]++;
        }
    }
    TEST_ASSERT_EQ(got[0], k_sim[0].frames);
    TEST_ASSERT_EQ(got[2], k_sim[2].frames);
    TEST_ASSERT_EQ(got[3], k_sim[3].frames);
    TEST_ASSERT_EQ(g_b[0].mode_manager.stats.sync_errors + g_b[2].mode_manager.stats.sync_errors +
                   g_b[3].mode_manager.stats.sync_errors, 0);
}

/* Channel 1 learns while 0, 2, 3 decode; a learn request on channel 3 is
 * refused while channel 1 holds the shared storage, accepted afterwards. */
static void test_shared_learn_one_channel_at_a_time(void) {
    setup(true);
    size_t n = make_edges();
    sent_can_frame_t ctl;
    memset(&ctl, 0, sizeof(ctl));
    ctl.id = SENT_CAN_ID_SENT_CONTROL;
    ctl.dlc = 1U;

    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_TRUE(sent_bridge_start_rx(&g_b[ch]));
    }
    /* Channel 1: forget its format, then learn it. */
    g_b[1].config.data_nibbles = 8U;
    ctl.data[0] = 0x14U;
    TEST_ASSERT_TRUE(sent_bridge_on_can_frame(&g_b[1], &ctl));
    TEST_ASSERT_EQ(sent_bridge_mode(&g_b[1]), SENT_BRIDGE_MODE_LEARN);
    ctl.data[0] = 0x34U;
    TEST_ASSERT_FALSE(sent_bridge_on_can_frame(&g_b[3], &ctl));   /* storage busy */
    TEST_ASSERT_EQ(sent_bridge_mode(&g_b[3]), SENT_BRIDGE_MODE_RX);

    bool learned = false;
    uint32_t got[NCH] = {0};
    for (size_t i = 0U; i < n; ++i) {
        uint8_t ch = g_edges[i].ch;
        sent_can_frame_t f;
        if (sent_bridge_on_edge(&g_b[ch], g_edges[i].t, &f)) {
            if (ch == 1U) {
                TEST_ASSERT_FALSE(learned);
                TEST_ASSERT_EQ(f.id, 0x602U);                 /* learn result 0x601 + 1 */
                TEST_ASSERT_EQ(f.data[0] | (f.data[1] << 8), 50);   /* 5.0 µs */
                TEST_ASSERT_EQ(f.data[2], 4);
                TEST_ASSERT_EQ(f.data[3], 1);                 /* status + data */
                TEST_ASSERT_EQ(f.data[4], 0x03);
                learned = true;
            } else {
                got[ch]++;
            }
        }
    }
    TEST_ASSERT_TRUE(learned);
    TEST_ASSERT_EQ(sent_bridge_mode(&g_b[1]), SENT_BRIDGE_MODE_OFF);
    TEST_ASSERT_EQ(g_b[1].config.data_nibbles, 4);
    TEST_ASSERT_EQ(got[0], k_sim[0].frames);
    TEST_ASSERT_EQ(got[2], k_sim[2].frames);
    TEST_ASSERT_EQ(got[3], k_sim[3].frames);
    /* Storage free again: channel 3 may learn now. */
    TEST_ASSERT_TRUE(sent_bridge_on_can_frame(&g_b[3], &ctl));
    TEST_ASSERT_EQ(sent_bridge_mode(&g_b[3]), SENT_BRIDGE_MODE_LEARN);
    /* Switching the learning channel to TX frees the storage and restores its config. */
    ctl.data[0] = 0x32U;
    TEST_ASSERT_TRUE(sent_bridge_on_can_frame(&g_b[3], &ctl));
    TEST_ASSERT_EQ(sent_bridge_mode(&g_b[3]), SENT_BRIDGE_MODE_TX);
    TEST_ASSERT_EQ(g_b[3].config.data_nibbles, 8);
    TEST_ASSERT_EQ(g_b[3].config.max_tick_x10_us, SENT_BRIDGE_LEARN_MAX_TICK_X10);
    TEST_ASSERT(g_learn.owner == NULL);
}

/* Without learn storage the learn command is refused (no crash). */
static void test_learn_without_storage_refused(void) {
    setup(false);
    sent_can_frame_t ctl;
    memset(&ctl, 0, sizeof(ctl));
    ctl.id = SENT_CAN_ID_SENT_CONTROL;
    ctl.dlc = 1U;
    ctl.data[0] = 0x04U;
    TEST_ASSERT_FALSE(sent_bridge_on_can_frame(&g_b[0], &ctl));
}

/* Each bridge owns exactly its own IDs; channel 0 owns the v1.1 IDs. */
static void test_frame_ownership_per_channel(void) {
    setup(true);
    sent_can_frame_t f;
    memset(&f, 0, sizeof(f));
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        for (uint8_t other = 0U; other < NCH; ++other) {
            bool mine = (ch == other);
            f.dlc = 1U;
            f.id = 0x001U + other;
            TEST_ASSERT_EQ(sent_bridge_owns_frame(&g_b[ch], &f), mine);
            f.id = 0x520U + 0x40U * other;
            TEST_ASSERT_EQ(sent_bridge_owns_frame(&g_b[ch], &f), mine);
            f.id = 0x600U;
            f.data[0] = (uint8_t)((other << 4) | 0x01U);
            TEST_ASSERT_EQ(sent_bridge_owns_frame(&g_b[ch], &f), mine);
        }
        f.id = 0x600U;
        f.dlc = 0U;
        TEST_ASSERT_FALSE(sent_bridge_owns_frame(&g_b[ch], &f));
        f.dlc = 1U;
        f.id = 0x510U + 0x40U * ch;     /* output IDs are not inputs */
        TEST_ASSERT_FALSE(sent_bridge_owns_frame(&g_b[ch], &f));
    }
    /* Control for another channel handed to a bridge anyway: refused, like an
     * unknown command in v1.1. */
    f.id = 0x600U;
    f.data[0] = 0x14U;
    TEST_ASSERT_FALSE(sent_bridge_on_can_frame(&g_b[0], &f));
}

void run_bridge_multi_tests(void);
void run_bridge_multi_tests(void) {
    TEST_RUN(test_four_bridges_interleaved_decode_all_frames);
    TEST_RUN(test_noisy_channel_does_not_disturb_others);
    TEST_RUN(test_shared_learn_one_channel_at_a_time);
    TEST_RUN(test_learn_without_storage_refused);
    TEST_RUN(test_frame_ownership_per_channel);
}
