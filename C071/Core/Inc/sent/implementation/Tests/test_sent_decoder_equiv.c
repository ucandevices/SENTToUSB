/* The optimised decoder (one division per candidate sync, shift/add
 * pre-check, multiply-based tolerance) must give exactly the same results as
 * the v1.1 decoder.  ref_decoder_v11.c is that decoder, kept verbatim.  These
 * tests compare return value, status and every byte of the output frame on
 * a large number of generated windows: valid frames with jitter right at the
 * +-0.35 tick tolerance edge, nibbles just outside 12..27, pauses up to and
 * beyond 768 ticks, garbage, wrap-around, several timestamp resolutions. */
#include "test.h"

#include <stdint.h>

#include "sent/sent_crc.h"
#include "sent/sent_decoder.h"
#include "sent/sent_protocol.h"

bool ref_v11_decode_from_timestamps(const sent_config_t* config,
                                    const uint32_t* timestamps,
                                    size_t timestamp_count,
                                    uint32_t units_per_us,
                                    sent_frame_t* out_frame,
                                    sent_decode_status_t* out_status);

static uint32_t g_rng = 0x12345678U;

static uint32_t rnd(void) {
    /* xorshift32: deterministic, so a failure is reproducible */
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}

static uint32_t rnd_range(uint32_t lo, uint32_t hi) { /* inclusive */
    return lo + rnd() % (hi - lo + 1U);
}

/* Random value in [-span, +span] in 1/1000 units. */
static int32_t rnd_milli(int32_t span) {
    return (int32_t)rnd_range(0U, (uint32_t)(2 * span)) - span;
}

static sent_config_t random_config(void) {
    sent_config_t c;
    c.data_nibbles = (uint8_t)rnd_range(1U, 8U);
    c.crc_mode = (rnd() & 1U) ? SENT_CRC_MODE_STATUS_AND_DATA : SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.pause_pulse_enabled = false;
    switch (rnd() % 4U) {
    case 0: c.min_tick_x10_us = 20U; c.max_tick_x10_us = 1130U; break;  /* bridge default */
    case 1: c.min_tick_x10_us = 25U; c.max_tick_x10_us = 35U; break;
    case 2: c.min_tick_x10_us = (uint16_t)rnd_range(10U, 400U);
            c.max_tick_x10_us = (uint16_t)(c.min_tick_x10_us + rnd_range(0U, 800U)); break;
    default: c.min_tick_x10_us = (uint16_t)rnd_range(1U, 60U);
             c.max_tick_x10_us = (uint16_t)rnd_range(1U, 60U); break;     /* may be invalid */
    }
    c.crc_init_seed = (rnd() & 1U) ? 0x03U : 0x05U;
    c.crc_legacy = (rnd() % 3U) == 0U;
    return c;
}

static uint32_t random_units(void) {
    static const uint32_t k_units[] = {1U, 2U, 8U, 16U, 48U, 64U, 170U};
    return k_units[rnd() % (sizeof(k_units) / sizeof(k_units[0]))];
}

/* Build a window of timestamps for config c.  Mostly frame-shaped, with the
 * per-interval error drawn around the tolerance edge, sometimes a CRC that
 * matches, sometimes leading garbage edges, sometimes a pause interval. */
static size_t build_window(const sent_config_t* c, uint32_t units, uint32_t* ts) {
    /* tick in 1/1000 timestamp units: mostly inside the configured window */
    uint32_t tick_milli_us = rnd_range(1500U, 120000U);
    if (rnd() % 4U != 0U && c->min_tick_x10_us <= c->max_tick_x10_us) {
        tick_milli_us = 100U * rnd_range(c->min_tick_x10_us, c->max_tick_x10_us);
    }
    uint32_t tick_milli_units = tick_milli_us * units;
    size_t n = 0U;
    uint32_t t = rnd();                       /* random start: exercises wrap-around */
    ts[n++] = t;

    size_t lead = (rnd() % 4U == 0U) ? rnd_range(1U, 2U) : 0U;
    size_t frame_iv = (size_t)c->data_nibbles + 3U;
    bool with_pause = (rnd() & 1U) != 0U;
    size_t total_iv = lead + frame_iv + (with_pause ? 1U : 0U);
    if (total_iv > SENT_MAX_INTERVALS) {
        total_iv = SENT_MAX_INTERVALS;
    }

    /* Nibble values with a matching CRC most of the time. */
    uint8_t nib[SENT_MAX_DATA_NIBBLES + 2U];
    for (size_t i = 0U; i < (size_t)c->data_nibbles + 2U; ++i) {
        nib[i] = (uint8_t)(rnd() & 0x0FU);
    }
    if (rnd() % 4U != 0U) {
        nib[c->data_nibbles + 1U] = sent_crc4_frame(c, nib[0], &nib[1], c->data_nibbles);
    }

    for (size_t k = 0U; k < total_iv; ++k) {
        int64_t ticks_milli;
        size_t f = k - lead;                  /* index inside the frame (if k >= lead) */
        uint32_t mode = rnd() % 16U;
        if (k < lead || mode == 0U) {
            ticks_milli = (int64_t)rnd_range(1000U, 900000U);          /* garbage */
        } else if (f == 0U) {
            ticks_milli = 56000 + rnd_milli(300);                      /* sync, +-0.3 ticks */
        } else if (f < frame_iv) {
            int32_t nt = (int32_t)nib[f - 1U] + 12;
            if (mode == 1U) nt = (rnd() & 1U) ? 11 : 28;               /* out of range */
            if (mode == 2U) nt = (rnd() & 1U) ? 10 : 29;
            /* error around the 0.35 tick limit most of the time */
            int32_t err = (mode < 8U) ? (int32_t)(rnd_range(330U, 370U)) * ((rnd() & 1U) ? 1 : -1)
                                      : rnd_milli(500);
            ticks_milli = (int64_t)nt * 1000 + err;
        } else {
            uint32_t p = (rnd() % 3U == 0U) ? rnd_range(760U, 790U) : rnd_range(8U, 800U);
            ticks_milli = (int64_t)p * 1000 + rnd_milli(500);           /* pause */
        }
        if (ticks_milli < 1) {
            ticks_milli = 1;
        }
        uint64_t step = ((uint64_t)ticks_milli * tick_milli_units + 500000U) / 1000000U;
        if (rnd() % 64U == 0U) {
            step = (rnd() & 1U) ? 0U : 0x80000000U;                    /* invalid step */
        }
        t += (uint32_t)step;
        ts[n++] = t;
    }
    return n;
}

/* Run both decoders on the same window; returns false on any difference. */
static bool same_result(const sent_config_t* c, const uint32_t* ts, size_t n, uint32_t units,
                        bool* out_ok, sent_decode_status_t* out_status) {
    sent_frame_t a, b;
    memset(&a, 0xA5, sizeof(a));
    memset(&b, 0xA5, sizeof(b));
    sent_decode_status_t sa = (sent_decode_status_t)99, sb = (sent_decode_status_t)99;
    bool ra = sent_decode_from_timestamps(c, ts, n, units, &a, &sa);
    bool rb = ref_v11_decode_from_timestamps(c, ts, n, units, &b, &sb);
    *out_ok = ra;
    *out_status = sa;
    return ra == rb && sa == sb && memcmp(&a, &b, sizeof(a)) == 0;
}

static void test_decoder_matches_v11_reference_random(void) {
    uint32_t counts[4] = {0U, 0U, 0U, 0U};
    uint32_t pauses = 0U;
    g_rng = 0x12345678U;
    for (uint32_t iter = 0U; iter < 300000U; ++iter) {
        sent_config_t c = random_config();
        uint32_t units = random_units();
        uint32_t ts[SENT_MAX_TIMESTAMPS];
        size_t n = build_window(&c, units, ts);
        bool ok;
        sent_decode_status_t st;
        if (!same_result(&c, ts, n, units, &ok, &st)) {
            printf("       mismatch at iteration %u (nibbles %u, units %u, n %u)\n",
                   (unsigned)iter, (unsigned)c.data_nibbles, (unsigned)units, (unsigned)n);
            TEST_ASSERT(false);
        }
        if ((unsigned)st < 4U) {
            counts[st]++;
        }
        if (ok) {
            sent_frame_t f;
            sent_decode_status_t s2;
            (void)sent_decode_from_timestamps(&c, ts, n, units, &f, &s2);
            if (f.has_pause) {
                pauses++;
            }
        }
    }
    printf("       OK %u, SYNC %u, CRC %u, SHAPE %u, with pause %u\n",
           (unsigned)counts[0], (unsigned)counts[1], (unsigned)counts[2],
           (unsigned)counts[3], (unsigned)pauses);
    /* The generator must actually exercise every outcome. */
    TEST_ASSERT(counts[SENT_DECODE_OK] > 1000U);
    TEST_ASSERT(counts[SENT_DECODE_SYNC_ERROR] > 1000U);
    TEST_ASSERT(counts[SENT_DECODE_CRC_ERROR] > 1000U);
    TEST_ASSERT(counts[SENT_DECODE_SHAPE_ERROR] > 1000U);
    TEST_ASSERT(pauses > 1000U);
}

/* Exhaustive sweep of one nibble interval across the whole plausible range at
 * fine resolution: every rounding and tolerance boundary is hit exactly. */
static void test_decoder_matches_v11_nibble_sweep(void) {
    sent_config_t c;
    memset(&c, 0, sizeof(c));
    c.data_nibbles = 1U;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.min_tick_x10_us = 20U;
    c.max_tick_x10_us = 1130U;
    c.crc_init_seed = 0x03U;
    static const uint32_t k_syncs[] = {56U * 3U * 48U, 56U * 3U * 48U + 1U, 8065U, 167U, 112U, 303744U};
    for (size_t s = 0U; s < sizeof(k_syncs) / sizeof(k_syncs[0]); ++s) {
        uint32_t sync = k_syncs[s];
        uint32_t units = (sync >= 1000U) ? 48U : 1U;
        uint32_t step = (sync > 20000U) ? 7U : 1U;
        for (uint32_t iv = 1U; iv < sync + 2U; iv += step) {
            /* sync, status = iv, data = 12 ticks, CRC = iv, pause = iv * 30 */
            uint32_t ts[6];
            ts[0] = 0xFFFFFF00U;
            ts[1] = ts[0] + sync;
            ts[2] = ts[1] + iv;
            ts[3] = ts[2] + (sync * 12U + 28U) / 56U;
            ts[4] = ts[3] + iv;
            ts[5] = ts[4] + iv * 30U;
            bool ok;
            sent_decode_status_t st;
            if (!same_result(&c, ts, 6U, units, &ok, &st)) {
                printf("       mismatch: sync %u interval %u\n", (unsigned)sync, (unsigned)iv);
                TEST_ASSERT(false);
            }
        }
    }
}

/* Pause sweep: pause from 0 to past 768 ticks, including the whole-sync
 * boundaries where the split into whole and fractional parts changes. */
static void test_decoder_matches_v11_pause_sweep(void) {
    sent_config_t c;
    memset(&c, 0, sizeof(c));
    c.data_nibbles = 1U;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.min_tick_x10_us = 20U;
    c.max_tick_x10_us = 1130U;
    c.crc_init_seed = 0x03U;
    uint32_t sync = 56U * 30U * 48U / 10U + 3U;     /* 3 µs tick at 48 MHz, a bit off */
    uint32_t tick = sync / 56U;
    uint8_t data = 0x7U;
    uint8_t crc = sent_crc4_frame(&c, 0U, &data, 1U);
    for (uint32_t pause = 1U; pause < tick * 800U; pause += 3U) {
        uint32_t ts[6];
        ts[0] = 1000U;
        ts[1] = ts[0] + sync;
        ts[2] = ts[1] + (sync * 12U + 28U) / 56U;
        ts[3] = ts[2] + (sync * (12U + data) + 28U) / 56U;
        ts[4] = ts[3] + (sync * (12U + crc) + 28U) / 56U;
        ts[5] = ts[4] + pause;
        bool ok;
        sent_decode_status_t st;
        if (!same_result(&c, ts, 6U, 48U, &ok, &st)) {
            printf("       mismatch: pause %u\n", (unsigned)pause);
            TEST_ASSERT(false);
        }
        TEST_ASSERT(ok);
    }
}

static void test_sync_in_range_matches_window(void) {
    sent_config_t c;
    memset(&c, 0, sizeof(c));
    c.data_nibbles = 6U;
    c.min_tick_x10_us = 25U;
    c.max_tick_x10_us = 35U;
    /* 48 counts/µs: window = [56*2.5*48, 56*3.5*48] = [6720, 9408] */
    TEST_ASSERT_FALSE(sent_decode_sync_in_range(&c, 6719U, 48U));
    TEST_ASSERT_TRUE(sent_decode_sync_in_range(&c, 6720U, 48U));
    TEST_ASSERT_TRUE(sent_decode_sync_in_range(&c, 9408U, 48U));
    TEST_ASSERT_FALSE(sent_decode_sync_in_range(&c, 9409U, 48U));
    TEST_ASSERT_FALSE(sent_decode_sync_in_range(&c, UINT32_MAX / 100U + 1U, 48U));
}

/* Pre-check boundaries: intervals of 28 ticks or more, and under 11.2 ticks,
 * are SHAPE errors (as before), and 27.34 / 11.66 ticks still decode. */
static void test_precheck_keeps_tolerance_edges(void) {
    sent_config_t c;
    memset(&c, 0, sizeof(c));
    c.data_nibbles = 1U;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.min_tick_x10_us = 20U;
    c.max_tick_x10_us = 1130U;
    c.crc_init_seed = 0x03U;
    const uint32_t sync = 5600U;                    /* 100 units per tick */
    uint8_t data = 0x0U;
    uint8_t crc = sent_crc4_frame(&c, 0xFU, &data, 1U);
    uint32_t ts[5];
    sent_frame_t f;
    sent_decode_status_t st;

    /* status nibble 15 at 27.35 ticks (2735 units), data 0 at 11.65 ticks */
    ts[0] = 0U;
    ts[1] = sync;
    ts[2] = ts[1] + 2735U;
    ts[3] = ts[2] + 1165U;
    ts[4] = ts[3] + 1200U + 100U * crc;
    TEST_ASSERT_TRUE(sent_decode_from_timestamps(&c, ts, 5U, 1U, &f, &st));
    TEST_ASSERT_EQ(f.status, 15);
    TEST_ASSERT_EQ(f.data_nibbles[0], 0);

    ts[2] = ts[1] + 2736U;                          /* 27.36 ticks: over tolerance */
    ts[3] = ts[2] + 1165U;
    ts[4] = ts[3] + 1200U + 100U * crc;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps(&c, ts, 5U, 1U, &f, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);

    ts[2] = ts[1] + 2800U;                          /* 28 ticks: rejected by the pre-check */
    ts[3] = ts[2] + 1200U;
    ts[4] = ts[3] + 1200U + 100U * crc;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps(&c, ts, 5U, 1U, &f, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);

    ts[2] = ts[1] + 1119U;                          /* 11.19 ticks: rejected by the pre-check */
    ts[3] = ts[2] + 1200U;
    ts[4] = ts[3] + 1200U + 100U * crc;
    TEST_ASSERT_FALSE(sent_decode_from_timestamps(&c, ts, 5U, 1U, &f, &st));
    TEST_ASSERT_EQ(st, SENT_DECODE_SHAPE_ERROR);
}

void run_sent_decoder_equiv_tests(void);
void run_sent_decoder_equiv_tests(void) {
    TEST_RUN(test_sync_in_range_matches_window);
    TEST_RUN(test_precheck_keeps_tolerance_edges);
    TEST_RUN(test_decoder_matches_v11_nibble_sweep);
    TEST_RUN(test_decoder_matches_v11_pause_sweep);
    TEST_RUN(test_decoder_matches_v11_reference_random);
}
