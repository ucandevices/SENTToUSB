/* sent_core (application core) host tests: SLCAN protocol, channel
 * addressing, round-robin edge budget, diagnostics, USB overflow counting.
 * Built twice: SENT_NUM_CHANNELS=4 (C071) and =1 (F042, must match v1.1 for
 * channel 0). */
#include "test.h"

#include "sent_core.h"
#include "sim_signal.h"
#include "slcan.h"

#define NCH SENT_NUM_CHANNELS
#define FAKE_RING 64U   /* like SENT_STM32_RX_RING_SIZE */

/* ── Fakes ──────────────────────────────────────────────────────────────── */

typedef struct {
    uint32_t ring[FAKE_RING];
    uint32_t head, tail;     /* free-running indices */
    bool running;
    uint32_t lost;
} fake_rx_t;

typedef struct {
    bool running;
    uint32_t submits;
    sent_frame_t last;
    uint16_t last_pause;
    uint16_t tick;
} fake_tx_t;

static fake_rx_t g_rx[NCH];
static fake_tx_t g_tx[NCH];
static uint32_t g_now_ms;
static bool g_usb_accept;
static char g_out[1 << 16];
static size_t g_out_len;
static uint32_t g_kicks[NCH];

static bool fake_rx_start(void* ctx) {
    fake_rx_t* r = (fake_rx_t*)ctx;
    r->running = true;
    r->head = r->tail = 0U;
    return true;
}
static void fake_rx_stop(void* ctx) { ((fake_rx_t*)ctx)->running = false; }
static bool fake_rx_poll(void* ctx, uint32_t* out, size_t* inout) {
    fake_rx_t* r = (fake_rx_t*)ctx;
    size_t n = 0U;
    if (!r->running) {
        *inout = 0U;
        return false;
    }
    while (r->tail != r->head && n < *inout) {
        out[n++] = r->ring[r->tail % FAKE_RING];
        r->tail++;
    }
    *inout = n;
    return n > 0U;
}
/* "ISR": store one edge, count a loss when the ring is full. */
static void fake_rx_capture(uint8_t ch, uint32_t t) {
    fake_rx_t* r = &g_rx[ch];
    if (!r->running) {
        return;
    }
    if (r->head - r->tail >= FAKE_RING - 1U) {
        r->lost++;
        return;
    }
    r->ring[r->head % FAKE_RING] = t;
    r->head++;
}

static bool fake_tx_start(void* ctx) { ((fake_tx_t*)ctx)->running = true; return true; }
static void fake_tx_stop(void* ctx) { ((fake_tx_t*)ctx)->running = false; }
static bool fake_tx_submit(void* ctx, const sent_frame_t* f, const sent_config_t* c, uint16_t pause) {
    fake_tx_t* t = (fake_tx_t*)ctx;
    (void)c;
    if (!t->running) {
        return false;
    }
    t->submits++;
    t->last = *f;
    t->last_pause = pause;
    return true;
}
static bool fake_tx_tick(void* ctx, uint16_t tick) { ((fake_tx_t*)ctx)->tick = tick; return true; }

static uint32_t p_now(void) { return g_now_ms; }
static bool p_usb(const uint8_t* d, uint16_t len) {
    if (!g_usb_accept) {
        return false;
    }
    if (len > 64U) {
        return false;   /* the core must never hand over more than one packet */
    }
    memcpy(&g_out[g_out_len], d, len);
    g_out_len += len;
    g_out[g_out_len] = '\0';
    return true;
}
static uint32_t p_lost(uint8_t ch) { return g_rx[ch].lost; }

/* Line control fake: last state per channel plus a call counter. */
static struct {
    unsigned calls;
    sent_bridge_mode_t mode;
    bool term;
} g_line[NCH];
static unsigned g_kick_tx_without_line;   /* kicks of a TX channel not yet switched to TX */
static uint32_t g_guard[NCH];
static bool g_supply_ok;
static uint16_t g_supply_mv;
static uint8_t g_supply_flags;
static bool g_supply_rev2d;               /* also report current and source */
static uint16_t g_supply_ma;
static uint8_t g_supply_src;

static void p_kick(uint8_t ch) {
    g_kicks[ch]++;
    if (g_tx[ch].running && g_line[ch].mode != SENT_BRIDGE_MODE_TX) {
        g_kick_tx_without_line++;
    }
}
static void p_line(uint8_t ch, sent_bridge_mode_t mode, bool term) {
    g_line[ch].calls++;
    g_line[ch].mode = mode;
    g_line[ch].term = term;
}
static uint32_t p_guard(uint8_t ch) { return g_guard[ch]; }
static bool p_supply(sent_core_supply_t* s) {
    s->vs_mv = g_supply_mv;
    s->flags = g_supply_flags;
    if (g_supply_rev2d) {
        s->is_ma = g_supply_ma;
        s->source = g_supply_src;
    }
    return g_supply_ok;
}

static sent_config_t default_config(void) {
    sent_config_t c;
    memset(&c, 0, sizeof(c));
    c.data_nibbles = 6U;
    c.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    c.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    c.min_tick_x10_us = SENT_BRIDGE_LEARN_MIN_TICK_X10;
    c.max_tick_x10_us = SENT_BRIDGE_LEARN_MAX_TICK_X10;
    c.crc_init_seed = 0x03U;
    return c;
}

static void setup(void) {
    memset(g_rx, 0, sizeof(g_rx));
    memset(g_tx, 0, sizeof(g_tx));
    memset(g_kicks, 0, sizeof(g_kicks));
    g_now_ms = 1000U;
    g_usb_accept = true;
    g_out_len = 0U;
    g_out[0] = '\0';
    sent_rx_hal_t rx[NCH];
    sent_tx_hal_t tx[NCH];
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        memset(&rx[ch], 0, sizeof(rx[ch]));
        rx[ch].context = &g_rx[ch];
        rx[ch].start_rx = fake_rx_start;
        rx[ch].stop_rx = fake_rx_stop;
        rx[ch].poll_timestamps_us = fake_rx_poll;
        rx[ch].ticks_per_us = SIM_UNITS_PER_US;
        memset(&tx[ch], 0, sizeof(tx[ch]));
        tx[ch].context = &g_tx[ch];
        tx[ch].start_tx = fake_tx_start;
        tx[ch].stop_tx = fake_tx_stop;
        tx[ch].submit_frame = fake_tx_submit;
        tx[ch].set_tick_x10_us = fake_tx_tick;
    }
    memset(g_line, 0, sizeof(g_line));
    memset(g_guard, 0, sizeof(g_guard));
    g_kick_tx_without_line = 0U;
    g_supply_ok = false;
    g_supply_mv = 0U;
    g_supply_flags = 0U;
    g_supply_rev2d = false;
    g_supply_ma = 0U;
    g_supply_src = 0U;
    static const sent_core_platform_t platform = {
        .now_ms = p_now,
        .usb_transmit = p_usb,
        .rx_lost_edges = p_lost,
        .tx_kick = p_kick,
        .line_control = p_line,
        .rx_guard_trips = p_guard,
        .supply_status = p_supply,
        .features = SENT_CORE_FEATURE_TERMINATION | SENT_CORE_FEATURE_SUPPLY_SENSE,
    };
    sent_config_t c = default_config();
    sent_core_init(&platform, &c, rx, tx, 0xBEEFU);
}

static void pump(unsigned passes) {
    for (unsigned i = 0U; i < passes; ++i) {
        sent_core_process();
    }
}

/* Send one SLCAN line, run the main loop, return what came back. */
static const char* cmd(const char* line) {
    g_out_len = 0U;
    g_out[0] = '\0';
    sent_core_on_usb_rx((const uint8_t*)line, (uint32_t)strlen(line));
    sent_core_on_usb_rx((const uint8_t*)"\r", 1U);
    pump(20U);
    return g_out;
}

/* Count output lines that are frames with this ID (and check they parse). */
static unsigned count_frames(const char* out, uint32_t id, sent_can_frame_t* last) {
    unsigned n = 0U;
    char line[64];
    size_t len = 0U;
    for (const char* p = out; *p != '\0'; ++p) {
        if (*p == '\r') {
            line[len] = '\0';
            sent_slcan_command_t c;
            if (len > 0U && line[0] == 't' && sent_slcan_parse_line(line, &c) &&
                c.type == SENT_SLCAN_CMD_FRAME && c.frame.id == id) {
                n++;
                if (last != NULL) {
                    *last = c.frame;
                }
            }
            len = 0U;
        } else if (len < sizeof(line) - 1U) {
            line[len++] = *p;
        }
    }
    return n;
}

/* ── Tests ──────────────────────────────────────────────────────────────── */

static void test_version_and_basic_commands(void) {
    setup();
    TEST_ASSERT(strcmp(cmd("V"), "V0200\r") == 0);
    TEST_ASSERT(strcmp(cmd("v"), "v0200\r") == 0);
    TEST_ASSERT(strcmp(cmd("N"), "NBEEF\r") == 0);
    TEST_ASSERT(strcmp(cmd("F"), "F00\r") == 0);
    TEST_ASSERT(strcmp(cmd("S6"), "\r") == 0);
    TEST_ASSERT(strcmp(cmd("t60"), "\a") == 0);           /* malformed frame */
}

static void test_info_reports_channel_count_and_modes(void) {
    setup();
    sent_can_frame_t f;
    const char* out = cmd("t600106");
    TEST_ASSERT(strncmp(out, "z\r", 2) == 0);
    TEST_ASSERT_EQ(count_frames(out, 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.dlc, 8);
    TEST_ASSERT_EQ(f.data[0], NCH);
    TEST_ASSERT_EQ(f.data[1], 0x02);
    TEST_ASSERT_EQ(f.data[2], 0x00);
    TEST_ASSERT_EQ(f.data[3], 0x00);                      /* all off */
    TEST_ASSERT_EQ(f.data[4], 0xFF);
    TEST_ASSERT_EQ(f.data[5], (NCH == 4U) ? 0xF0 : 0x10); /* all AUTO, all off */
    TEST_ASSERT_EQ(f.data[6], SENT_CORE_FEATURE_TERMINATION | SENT_CORE_FEATURE_SUPPLY_SENSE);
    TEST_ASSERT_EQ(f.data[7], 0x00);
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);             /* RX on every channel */
    out = cmd("t600106");
    TEST_ASSERT_EQ(count_frames(out, 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.data[3], (NCH == 4U) ? 0x55 : 0x01);
    TEST_ASSERT_EQ(f.data[5], (NCH == 4U) ? 0xFF : 0x11); /* AUTO: on in RX */
    TEST_ASSERT(strcmp(cmd("C"), "\r") == 0);
    out = cmd("t600106");
    TEST_ASSERT_EQ(count_frames(out, 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.data[3], 0x00);
}

static void test_channel_addressing_config_and_control(void) {
    setup();
    /* channel 1 config: 4 nibbles, status+data CRC */
    TEST_ASSERT(strcmp(cmd("t0023040103"), "z\r") == 0);
    TEST_ASSERT_EQ(sent_core_bridge(0)->config.data_nibbles, 6);   /* ch0 untouched */
    /* control for channel 1: start RX */
    const char* out = cmd("t600111");
    if (NCH == 4U) {
        TEST_ASSERT_EQ(sent_core_bridge(1)->config.data_nibbles, 4);
        TEST_ASSERT_EQ(sent_core_bridge(1)->config.crc_mode, SENT_CRC_MODE_STATUS_AND_DATA);
        TEST_ASSERT(strcmp(out, "z\r") == 0);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(1)), SENT_BRIDGE_MODE_RX);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(0)), SENT_BRIDGE_MODE_OFF);
        TEST_ASSERT(strcmp(cmd("t600113"), "z\r") == 0);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(1)), SENT_BRIDGE_MODE_OFF);
    } else {
        /* v1.1: 0x002 is an unknown ID (accepted, ignored); 0x11 an unknown command */
        TEST_ASSERT(strcmp(out, "\a") == 0);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(0)), SENT_BRIDGE_MODE_OFF);
    }
    /* channel 0 exactly as v1.1 */
    TEST_ASSERT(strcmp(cmd("t0013040103"), "z\r") == 0);
    TEST_ASSERT_EQ(sent_core_bridge(0)->config.data_nibbles, 4);
    TEST_ASSERT(strcmp(cmd("t600101"), "z\r") == 0);
    TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(0)), SENT_BRIDGE_MODE_RX);
    TEST_ASSERT(strcmp(cmd("t600103"), "z\r") == 0);
    /* control for a channel that does not exist: refused */
    TEST_ASSERT(strcmp(cmd("t600151"), "\a") == 0);
    TEST_ASSERT(strcmp(cmd("t600107"), "\a") == 0);       /* unknown command */
    TEST_ASSERT(strcmp(cmd("t6000"), "z\r") == 0);        /* DLC 0: ignored as in v1.1 */
    TEST_ASSERT(strcmp(cmd("t1232AABB"), "z\r") == 0);    /* unknown ID: ignored */
    /* set TX tick on channel 0 and the last channel */
    TEST_ASSERT(strcmp(cmd("t6003052C01"), "z\r") == 0);  /* 30.0 µs */
    TEST_ASSERT_EQ(g_tx[0].tick, 300);
    if (NCH == 4U) {
        TEST_ASSERT(strcmp(cmd("t6003353200"), "z\r") == 0);
        TEST_ASSERT_EQ(g_tx[3].tick, 50);
    }
}

static void test_tx_frames_routed_per_channel(void) {
    setup();
    TEST_ASSERT(strcmp(cmd("t600102"), "z\r") == 0);      /* ch0 TX */
    TEST_ASSERT(strcmp(cmd("t52050100123456"), "z\r") == 0);
    TEST_ASSERT_EQ(g_tx[0].submits, 1);
    TEST_ASSERT_EQ(g_tx[0].last.status, 1);
    TEST_ASSERT_EQ(g_tx[0].last.data_nibbles[0], 1);
    TEST_ASSERT_EQ(g_tx[0].last.data_nibbles[5], 6);
    TEST_ASSERT_EQ(g_tx[0].last_pause, 12);
    TEST_ASSERT(g_kicks[0] > 0U);
    if (NCH == 4U) {
        TEST_ASSERT(strcmp(cmd("t5E050200654321"), "\a") == 0);   /* ch3 not in TX */
        TEST_ASSERT(strcmp(cmd("t600132"), "z\r") == 0);
        TEST_ASSERT(strcmp(cmd("t5E070200654321E001"), "z\r") == 0);  /* pause 0x1E0 */
        TEST_ASSERT_EQ(g_tx[3].submits, 1);
        TEST_ASSERT_EQ(g_tx[3].last.status, 2);
        TEST_ASSERT_EQ(g_tx[3].last.data_nibbles[0], 6);
        TEST_ASSERT_EQ(g_tx[3].last_pause, 480);
        TEST_ASSERT_EQ(g_tx[1].submits + g_tx[2].submits, 0);
        TEST_ASSERT_EQ(g_tx[0].submits, 1);
        /* TX on 3 and RX on 0 at the same time */
        TEST_ASSERT(strcmp(cmd("t600101"), "z\r") == 0);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(0)), SENT_BRIDGE_MODE_RX);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(3)), SENT_BRIDGE_MODE_TX);
    } else {
        TEST_ASSERT(strcmp(cmd("t56050200654321"), "z\r") == 0);   /* unknown ID in v1.1 */
    }
}

static void test_learn_shared_between_channels(void) {
    setup();
    TEST_ASSERT(strcmp(cmd("t600104"), "z\r") == 0);      /* ch0 learns */
    sent_can_frame_t f;
    const char* out = cmd("t600106");
    TEST_ASSERT_EQ(count_frames(out, 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.data[4], 0);
    TEST_ASSERT_EQ(f.data[3] & 0x3, 3);
    if (NCH == 4U) {
        TEST_ASSERT(strcmp(cmd("t600124"), "\a") == 0);   /* ch2: storage busy */
        TEST_ASSERT(strcmp(cmd("t600103"), "z\r") == 0);  /* ch0 stops: storage free */
        TEST_ASSERT(strcmp(cmd("t600124"), "z\r") == 0);
        TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(2)), SENT_BRIDGE_MODE_LEARN);
        TEST_ASSERT(strcmp(cmd("t600104"), "\a") == 0);
        /* a config frame for ch2 ends its learn */
        TEST_ASSERT(strcmp(cmd("t0031"  "06"), "z\r") == 0);
        TEST_ASSERT_FALSE(sent_bridge_is_learning(sent_core_bridge(2)));
        TEST_ASSERT(strcmp(cmd("t600104"), "z\r") == 0);
    }
}

/* Feed merged streams through the fake capture rings, running the main loop
 * every few edges like the real firmware would. */
static void run_streams(const sim_channel_t* sims, size_t nsims, sim_stream_t* streams,
                        sim_edge_t* edges, size_t max_edges, unsigned edges_per_pass) {
    for (size_t c = 0U; c < nsims; ++c) {
        TEST_ASSERT_TRUE(sim_generate(&sims[c], 0xFFFF0000ULL + 777ULL * c, &streams[c]));
    }
    size_t n = sim_merge(streams, nsims, edges, max_edges);
    for (size_t i = 0U; i < n; ++i) {
        fake_rx_capture(edges[i].ch, edges[i].t);
        if ((i % edges_per_pass) == 0U) {
            sent_core_process();
        }
    }
    pump(200U);
}

static sim_stream_t g_streams[4];
static sim_edge_t g_edges[4 * SIM_MAX_EDGES];

static void test_rx_end_to_end_all_channels(void) {
    setup();
    static const sim_channel_t sims[4] = {
        {{6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 30, 12, 40, 5},
        {{6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 30, 0, 40, 6},
        {{6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 35, 25, 40, 7},
        {{6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 40, 12, 40, 8},
    };
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    g_out_len = 0U;
    run_streams(sims, NCH, g_streams, g_edges, sizeof(g_edges) / sizeof(g_edges[0]), 4U);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(count_frames(g_out, 0x510U + 0x40U * ch, NULL), sims[ch].frames);
        TEST_ASSERT_EQ(sent_core_bridge(ch)->mode_manager.stats.frames_decoded, sims[ch].frames);
        TEST_ASSERT_EQ(g_rx[ch].lost, 0);
    }
    /* diagnostics: one 0x511 + 0x40n per channel once 100 ms have passed */
    g_out_len = 0U;
    g_now_ms += 100U;
    pump(50U);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        sent_can_frame_t d;
        TEST_ASSERT_EQ(count_frames(g_out, 0x511U + 0x40U * ch, &d), 1);
        TEST_ASSERT_EQ(d.data[0] | (d.data[1] << 8), sims[ch].frames);
        TEST_ASSERT_EQ(count_frames(g_out, 0x512U + 0x40U * ch, NULL), 0);  /* no losses */
    }
    /* nothing changed: no more diagnostics */
    g_out_len = 0U;
    g_now_ms += 500U;
    pump(50U);
    TEST_ASSERT_EQ(g_out_len, 0);
}

/* A configured output ID still works per channel. */
static void test_rx_output_id_configurable(void) {
    setup();
    static const sim_channel_t sim = {
        {6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 30, 12, 10, 9};
    /* ch(NCH-1): 6 nibbles, output ID 0x123 */
    char line[32];
    snprintf(line, sizeof(line), "t%03X706000000000123", 0x001U + (NCH - 1U));
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    TEST_ASSERT_TRUE(sim_generate(&sim, 5000U, &g_streams[0]));
    g_out_len = 0U;
    for (size_t i = 0U; i < g_streams[0].count; ++i) {
        fake_rx_capture((uint8_t)(NCH - 1U), (uint32_t)g_streams[0].t[i]);
        sent_core_process();
    }
    pump(100U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x123U, NULL), 10);
}

/* Round robin with an edge budget: a channel flooded with edges does not
 * stop the others from being served in the same pass. */
static void test_edge_budget_per_pass(void) {
    setup();
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        for (uint32_t i = 0U; i < 60U; ++i) {
            fake_rx_capture(ch, 1000U + i * 100U);
        }
    }
    sent_core_process();
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        uint32_t left = g_rx[ch].head - g_rx[ch].tail;
        TEST_ASSERT_EQ(left, (SENT_EDGE_BUDGET >= 60U) ? 0U : 60U - SENT_EDGE_BUDGET);
    }
    pump(10U);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(g_rx[ch].head - g_rx[ch].tail, 0);
    }
}

/* Lost edges reset alignment of that channel only and are reported in the
 * extended diagnostics frame 0x512 + 0x40n. */
static void test_lost_edges_reported_per_channel(void) {
    setup();
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    uint8_t ch = (uint8_t)(NCH - 1U);
    g_rx[ch].lost = 3U;
    g_out_len = 0U;
    g_now_ms += 100U;
    pump(30U);
    TEST_ASSERT_EQ(sent_core_bridge(ch)->mode_manager.stats.dropped_events, 1);
    for (uint8_t other = 0U; other < ch; ++other) {
        TEST_ASSERT_EQ(sent_core_bridge(other)->mode_manager.stats.dropped_events, 0);
    }
    sent_can_frame_t d;
    TEST_ASSERT_EQ(count_frames(g_out, 0x512U + 0x40U * ch, &d), 1);
    TEST_ASSERT_EQ(d.data[0], 1);
    TEST_ASSERT_EQ(d.data[4], 0);
}

/* USB buffer full: frames are dropped whole (never truncated), counted per
 * channel, and the count reaches the host once USB drains. */
static void test_usb_overflow_counted(void) {
    setup();
    static const sim_channel_t sim = {
        {6, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x03, false}, 30, 12, 64, 3};
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    g_usb_accept = false;
    g_out_len = 0U;
    TEST_ASSERT_TRUE(sim_generate(&sim, 100U, &g_streams[0]));
    uint32_t produced = 0U;
    for (int rep = 0; rep < 4; ++rep) {     /* 4 x 64 frames x 16 bytes > 2 KB */
        uint64_t shift = (uint64_t)rep * (g_streams[0].t[g_streams[0].count - 1U] - g_streams[0].t[0] + 1000U);
        for (size_t i = 0U; i < g_streams[0].count; ++i) {
            fake_rx_capture(0U, (uint32_t)(g_streams[0].t[i] + shift));
            sent_core_process();
        }
    }
    produced = sent_core_bridge(0)->mode_manager.stats.frames_decoded;
    TEST_ASSERT(produced >= 200U);
    uint32_t overflows = sent_core_usb_overflows(0);
    TEST_ASSERT(overflows > 0U);
    g_usb_accept = true;
    pump(2000U);            /* drain the ring first ... */
    g_now_ms += 100U;
    pump(200U);             /* ... then the diagnostics have room */
    unsigned delivered = count_frames(g_out, 0x510U, NULL);
    TEST_ASSERT_EQ(delivered + overflows, produced);
    sent_can_frame_t d;
    TEST_ASSERT(count_frames(g_out, 0x512U, &d) >= 1U);
    /* every line in the output is complete and parses */
    size_t lines = 0U;
    for (size_t i = 0U; i < g_out_len; ++i) {
        lines += (g_out[i] == '\r');
    }
    TEST_ASSERT_EQ(g_out[g_out_len - 1U], '\r');
    TEST_ASSERT(lines >= delivered);
}

/* The learn result comes back on 0x601 + n. */
static void test_learn_result_id_per_channel(void) {
    setup();
    uint8_t ch = (uint8_t)(NCH - 1U);
    static const sim_channel_t sim = {
        {5, SENT_CRC_MODE_DATA_ONLY, SENT_NIBBLE_ORDER_MSB_FIRST, false, 20, 1130, 0x05, false}, 45, 12, 30, 21};
    char line[16];
    snprintf(line, sizeof(line), "t6001%X4", ch);
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_TRUE(sim_generate(&sim, 0xFFFFF000ULL, &g_streams[0]));
    g_out_len = 0U;
    for (size_t i = 0U; i < g_streams[0].count; ++i) {
        fake_rx_capture(ch, (uint32_t)g_streams[0].t[i]);
        sent_core_process();
    }
    pump(50U);
    sent_can_frame_t r;
    TEST_ASSERT_EQ(count_frames(g_out, 0x601U + ch, &r), 1);
    TEST_ASSERT_EQ(r.data[0] | (r.data[1] << 8), 45);
    TEST_ASSERT_EQ(r.data[2], 5);
    TEST_ASSERT_EQ(r.data[3], 0);
    TEST_ASSERT_EQ(r.data[4], 0x05);
    TEST_ASSERT_EQ(sent_bridge_mode(sent_core_bridge(ch)), SENT_BRIDGE_MODE_OFF);
}

/* Bytes arriving split across USB packets, CR and LF terminators, and a
 * flood beyond the RX ring: no crash, complete lines still answered. */
static void test_usb_rx_line_assembly(void) {
    setup();
    g_out_len = 0U;
    sent_core_on_usb_rx((const uint8_t*)"V", 1U);
    pump(2U);
    TEST_ASSERT_EQ(g_out_len, 0);
    sent_core_on_usb_rx((const uint8_t*)"\nv\r", 3U);
    pump(10U);
    TEST_ASSERT(strcmp(g_out, "V0200\rv0200\r") == 0);
    uint8_t junk[2000];
    memset(junk, 'x', sizeof(junk));
    sent_core_on_usb_rx(junk, sizeof(junk));
    pump(5U);
    TEST_ASSERT(strcmp(cmd(""), "\r") == 0);   /* the junk line ends: unsupported, acked */
    TEST_ASSERT(strcmp(cmd("V"), "V0200\r") == 0);
}

/* Rev 2b: the platform hears about every mode/termination change, gets the
 * TX state before a TX frame is kicked, and power-up is "all off". */
static void test_line_state_follows_mode(void) {
    setup();
    uint8_t last = (uint8_t)(NCH - 1U);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(g_line[ch].calls, 1);              /* from sent_core_init */
        TEST_ASSERT_EQ(g_line[ch].mode, SENT_BRIDGE_MODE_OFF);
        TEST_ASSERT_FALSE(g_line[ch].term);
    }
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(g_line[ch].calls, 2);
        TEST_ASSERT_EQ(g_line[ch].mode, SENT_BRIDGE_MODE_RX);
        TEST_ASSERT_TRUE(g_line[ch].term);
    }
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);             /* no change, no call */
    TEST_ASSERT_EQ(g_line[0].calls, 2);

    char line[16];
    snprintf(line, sizeof(line), "t6001%X2", last);       /* last channel: TX */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_EQ(g_line[last].mode, SENT_BRIDGE_MODE_TX);
    TEST_ASSERT_FALSE(g_line[last].term);                 /* AUTO: off in TX */
    snprintf(line, sizeof(line), "t%03X50100123456", 0x520U + 0x40U * last);
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT(g_kicks[last] > 0U);
    TEST_ASSERT_EQ(g_kick_tx_without_line, 0);
    if (NCH == 4U) {
        TEST_ASSERT_EQ(g_line[0].mode, SENT_BRIDGE_MODE_RX);   /* others untouched */
        TEST_ASSERT_TRUE(g_line[0].term);
    }

    snprintf(line, sizeof(line), "t6001%X4", last);       /* learn: RX path, on */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_EQ(g_line[last].mode, SENT_BRIDGE_MODE_LEARN);
    TEST_ASSERT_TRUE(g_line[last].term);

    TEST_ASSERT(strcmp(cmd("C"), "\r") == 0);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(g_line[ch].mode, SENT_BRIDGE_MODE_OFF);
        TEST_ASSERT_FALSE(g_line[ch].term);
    }

    /* 'L' = listen-only: RX with AUTO termination off; 'O' turns it on. */
    TEST_ASSERT(strcmp(cmd("L"), "\r") == 0);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(g_line[ch].mode, SENT_BRIDGE_MODE_RX);
        TEST_ASSERT_FALSE(g_line[ch].term);
    }
    snprintf(line, sizeof(line), "t6002%X701", last);     /* forced on still wins */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_TRUE(g_line[last].term);
    snprintf(line, sizeof(line), "t6002%X702", last);     /* back to AUTO: off */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_FALSE(g_line[last].term);
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    for (uint8_t ch = 0U; ch < NCH; ++ch) {
        TEST_ASSERT_EQ(g_line[ch].mode, SENT_BRIDGE_MODE_RX);
        TEST_ASSERT_TRUE(g_line[ch].term);
    }
    TEST_ASSERT(strcmp(cmd("C"), "\r") == 0);
}

/* Control command 0x07: off / on / auto per channel, reported in 0x60F. */
static void test_termination_command(void) {
    setup();
    uint8_t last = (uint8_t)(NCH - 1U);
    char line[16];
    sent_can_frame_t f;

    snprintf(line, sizeof(line), "t6002%X701", last);     /* force on while off */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_EQ(sent_core_termination_setting(last), SENT_BRIDGE_TERM_ON);
    TEST_ASSERT_TRUE(g_line[last].term);
    TEST_ASSERT_EQ(g_line[last].mode, SENT_BRIDGE_MODE_OFF);
    TEST_ASSERT_EQ(count_frames(cmd("t600106"), 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.data[5], (uint8_t)(((NCH == 4U) ? 0xF0U : 0x10U) & ~(0x10U << last)) | (1U << last));

    snprintf(line, sizeof(line), "t6001%X2", last);       /* forced on stays on in TX */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_EQ(g_line[last].mode, SENT_BRIDGE_MODE_TX);
    TEST_ASSERT_TRUE(g_line[last].term);

    snprintf(line, sizeof(line), "t6002%X700", last);     /* force off */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_FALSE(g_line[last].term);
    snprintf(line, sizeof(line), "t6001%X1", last);       /* RX, still off */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_EQ(g_line[last].mode, SENT_BRIDGE_MODE_RX);
    TEST_ASSERT_FALSE(g_line[last].term);
    TEST_ASSERT_EQ(count_frames(cmd("t600106"), 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.data[5] & (0x11U << last), 0);

    snprintf(line, sizeof(line), "t6002%X702", last);     /* back to auto: on in RX */
    TEST_ASSERT(strcmp(cmd(line), "z\r") == 0);
    TEST_ASSERT_TRUE(g_line[last].term);
    TEST_ASSERT_EQ(count_frames(cmd("t600106"), 0x60FU, &f), 1);
    TEST_ASSERT_EQ(f.data[5] & (0x11U << last), 0x11U << last);

    /* refused: bad value, missing value, channel that does not exist */
    snprintf(line, sizeof(line), "t6002%X703", last);
    TEST_ASSERT(strcmp(cmd(line), "\a") == 0);
    snprintf(line, sizeof(line), "t6001%X7", last);
    TEST_ASSERT(strcmp(cmd(line), "\a") == 0);
    snprintf(line, sizeof(line), "t6002%X701", NCH);
    TEST_ASSERT(strcmp(cmd(line), "\a") == 0);
    TEST_ASSERT_EQ(sent_core_termination_setting(last), SENT_BRIDGE_TERM_AUTO);
}

/* Edge-rate guard trips reach the host in 0x512 + 0x40n, data[6..7]. */
static void test_guard_trips_in_loss_frame(void) {
    setup();
    TEST_ASSERT(strcmp(cmd("O"), "\r") == 0);
    uint8_t ch = (uint8_t)(NCH - 1U);
    g_guard[ch] = 2U;
    g_rx[ch].lost = 2U;          /* the platform also notes a lost edge per trip */
    g_out_len = 0U;
    g_now_ms += 100U;
    pump(10U);
    sent_can_frame_t d;
    TEST_ASSERT_EQ(count_frames(g_out, 0x512U + 0x40U * ch, &d), 1);
    TEST_ASSERT_EQ(d.dlc, 8);
    TEST_ASSERT_EQ(d.data[0], 1);    /* one resync event */
    TEST_ASSERT_EQ(d.data[4] | d.data[5], 0);
    TEST_ASSERT_EQ(d.data[6], 2);
    TEST_ASSERT_EQ(d.data[7], 0);
    g_guard[ch] = 0x12345U;          /* saturates at 0xFFFF */
    g_out_len = 0U;
    g_now_ms += 100U;
    pump(10U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x512U + 0x40U * ch, &d), 1);
    TEST_ASSERT_EQ(d.data[6], 0xFF);
    TEST_ASSERT_EQ(d.data[7], 0xFF);
    g_out_len = 0U;
    g_out[0] = '\0';
    g_now_ms += 100U;
    pump(10U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x512U + 0x40U * ch, NULL), 0);   /* unchanged */
}

/* Supply frame 0x60E: every 500 ms while the platform has something to report. */
static void test_supply_frame(void) {
    setup();
    sent_can_frame_t f;
    g_out_len = 0U;
    g_out[0] = '\0';
    g_now_ms += 600U;
    pump(5U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x60EU, NULL), 0);  /* not configured */
    g_supply_ok = true;
    g_supply_mv = 4987U;
    g_supply_flags = SENT_CORE_SUPPLY_EN | SENT_CORE_SUPPLY_FAULT;
    g_out_len = 0U;
    g_out[0] = '\0';
    g_now_ms += 499U;
    pump(5U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x60EU, NULL), 0);  /* < 500 ms since last look */
    g_now_ms += 1U;
    pump(5U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x60EU, &f), 1);
    TEST_ASSERT_EQ(f.dlc, 6);
    TEST_ASSERT_EQ(f.data[0] | (f.data[1] << 8), 4987);
    TEST_ASSERT_EQ(f.data[2], 0x03);
    TEST_ASSERT_EQ(f.data[3] | (f.data[4] << 8), 0xFFFF);  /* current not measured */
    TEST_ASSERT_EQ(f.data[5], 0xFF);                       /* source not reported */
    g_now_ms += 250U;
    pump(5U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x60EU, NULL), 1);
    g_now_ms += 250U;
    pump(5U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x60EU, NULL), 2);
    /* Rev 2d: current and source in data[3..5], data[0..2] unchanged. */
    g_supply_rev2d = true;
    g_supply_mv = 12034U;
    g_supply_flags = 0U;
    g_supply_ma = 0x0123U;
    g_supply_src = SENT_CORE_SUPPLY_SRC_EXT;
    g_out_len = 0U;
    g_out[0] = '\0';
    g_now_ms += 500U;
    pump(5U);
    TEST_ASSERT_EQ(count_frames(g_out, 0x60EU, &f), 1);
    TEST_ASSERT_EQ(f.dlc, 6);
    TEST_ASSERT_EQ(f.data[0] | (f.data[1] << 8), 12034);
    TEST_ASSERT_EQ(f.data[2], 0x00);
    TEST_ASSERT_EQ(f.data[3], 0x23);
    TEST_ASSERT_EQ(f.data[4], 0x01);
    TEST_ASSERT_EQ(f.data[5], 0x01);
}

void run_core_tests(void);
void run_core_tests(void) {
    printf("--- sent_core with SENT_NUM_CHANNELS=%u ---\n", (unsigned)NCH);
    TEST_RUN(test_version_and_basic_commands);
    TEST_RUN(test_info_reports_channel_count_and_modes);
    TEST_RUN(test_channel_addressing_config_and_control);
    TEST_RUN(test_tx_frames_routed_per_channel);
    TEST_RUN(test_learn_shared_between_channels);
    TEST_RUN(test_rx_end_to_end_all_channels);
    TEST_RUN(test_rx_output_id_configurable);
    TEST_RUN(test_edge_budget_per_pass);
    TEST_RUN(test_lost_edges_reported_per_channel);
    TEST_RUN(test_usb_overflow_counted);
    TEST_RUN(test_learn_result_id_per_channel);
    TEST_RUN(test_usb_rx_line_assembly);
    TEST_RUN(test_line_state_follows_mode);
    TEST_RUN(test_termination_command);
    TEST_RUN(test_guard_trips_in_loss_frame);
    TEST_RUN(test_supply_frame);
}
