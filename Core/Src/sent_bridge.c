#include "sent_bridge.h"

#include "sent/sent_assert.h"
#include <string.h>

#include "sent/sent_crc.h"
#include "sent/sent_decoder.h"
#include "sent/sent_encoder.h"
#include "sent/sent_slow_channel.h"

/* rx.skipped value meaning "alignment lost, already accounted for". */
#define RX_SKIPPED_LOST 3U

/* HAL helpers — guard + call, return true if no HAL present (no-op). */

static uint32_t bridge_ticks_per_us(const sent_bridge_t* b) {
    return (b->has_rx_hal && b->rx_hal.ticks_per_us != 0U) ? b->rx_hal.ticks_per_us : 1U;
}

/* Forget the current frame alignment and any partial slow-channel message.
 * The next frames re-align silently (no error is counted for the re-sync). */
static void bridge_reset_rx_stream(sent_bridge_t* b) {
    b->rx.count = 0U;
    b->rx.skipped = RX_SKIPPED_LOST;
    b->rx.skipped_interval = 0U;
    b->rx.ref_sync = 0U;
    sent_slow_channel_reset(&b->slow_channel);
}

static bool bridge_start_rx(sent_bridge_t* b) {
    bridge_reset_rx_stream(b);
    if (!b->has_rx_hal || b->rx_hal.start_rx == NULL) {
        return true;
    }
    return b->rx_hal.start_rx(b->rx_hal.context);
}

static void bridge_stop_rx(sent_bridge_t* b) {
    if (b->has_rx_hal && b->rx_hal.stop_rx != NULL) {
        b->rx_hal.stop_rx(b->rx_hal.context);
    }
}

static bool bridge_start_tx(sent_bridge_t* b) {
    if (!b->has_tx_hal || b->tx_hal.start_tx == NULL) {
        return true;
    }
    return b->tx_hal.start_tx(b->tx_hal.context);
}

static void bridge_stop_tx(sent_bridge_t* b) {
    if (b->has_tx_hal && b->tx_hal.stop_tx != NULL) {
        b->tx_hal.stop_tx(b->tx_hal.context);
    }
}

static void bridge_stop_all(sent_bridge_t* b) {
    bridge_stop_rx(b);
    bridge_stop_tx(b);
}

/* Serialize a decoded sent_frame_t into a compact CAN frame.
 *
 *   data[0 .. B-1]  data nibbles packed in pairs (first nibble in the high half),
 *                   B = ceil(data_nibbles / 2)
 *   data[I]         info byte, I = max(3, B):
 *                     bits 7..4  status and communication nibble
 *                     bit 0      slow-channel message completed with this frame
 *                     bit 1      slow message is Enhanced (else Short)
 *                     bit 2      Enhanced with 16-bit data / 4-bit ID
 *   data[I+1]       slow message ID        (only when bit 0 is set)
 *   data[I+2..I+3]  slow data, little-endian
 *
 * DLC = I + 1, or I + 4 with a slow message.  For up to 6 data nibbles the
 * info byte is byte 3, so the slow-channel fields keep their previous places.
 *
 * @param f       decoded SENT frame
 * @param slow    completed slow-channel message, or NULL
 * @param out_id  CAN ID to use for the output frame
 * @param out     [out] CAN frame to populate */
static void pack_rx_can_frame(const sent_frame_t* f,
                               const sent_slow_message_t* slow,
                               uint16_t out_id,
                               sent_can_frame_t* out) {
    uint8_t n = f->data_nibbles_count;
    uint8_t bytes = (uint8_t)((n + 1U) / 2U);  /* ceil(n/2) */
    uint8_t info = (bytes < 3U) ? 3U : bytes;

    out->id = out_id;
    out->extended = false;
    for (uint8_t i = 0U; i < SENT_CAN_MAX_DLC; ++i) {
        out->data[i] = 0U;
    }
    for (uint8_t i = 0U; i < bytes; ++i) {
        uint8_t hi = (2U * i     < n) ? (f->data_nibbles[2U * i]      & 0x0FU) : 0U;
        uint8_t lo = (2U * i + 1U < n) ? (f->data_nibbles[2U * i + 1U] & 0x0FU) : 0U;
        out->data[i] = (uint8_t)((hi << 4U) | lo);
    }

    out->data[info] = (uint8_t)((f->status & 0x0FU) << 4U);
    out->dlc = (uint8_t)(info + 1U);

    if (slow != NULL && slow->format != SENT_SLOW_FORMAT_NONE) {
        out->data[info] |= SENT_BRIDGE_INFO_SLOW_NEW;
        if (slow->format == SENT_SLOW_FORMAT_ENHANCED_12_8 ||
            slow->format == SENT_SLOW_FORMAT_ENHANCED_16_4) {
            out->data[info] |= SENT_BRIDGE_INFO_SLOW_ENHANCED;
        }
        if (slow->format == SENT_SLOW_FORMAT_ENHANCED_16_4) {
            out->data[info] |= SENT_BRIDGE_INFO_SLOW_16BIT;
        }
        out->data[info + 1U] = slow->message_id;
        out->data[info + 2U] = (uint8_t)(slow->data & 0xFFU);
        out->data[info + 3U] = (uint8_t)((slow->data >> 8U) & 0xFFU);
        out->dlc = (uint8_t)(info + 4U);
    }
}

/* Unpack a CAN TX frame (ID 0x520) into a SENT frame and submit to TX HAL.
 * @param bridge  bridge instance
 * @param frame   CAN frame with packed SENT data (status, data, pause ticks)
 * @return        true if frame was submitted to TX HAL */
static bool handle_tx_can_frame(sent_bridge_t* bridge, const sent_can_frame_t* frame) {
    SENT_ASSERT(bridge != NULL && frame != NULL);
    if (!bridge->config_valid || !sent_mode_manager_is_tx(&bridge->mode_manager) || !bridge->has_tx_hal ||
        bridge->tx_hal.submit_frame == NULL) {
        return false;
    }
    if (frame->dlc < 5U) {
        return false;
    }

    sent_frame_t tx_frame;
    tx_frame.status = (uint8_t)(frame->data[0] & 0x0FU);
    tx_frame.data_nibbles_count = bridge->config.data_nibbles;

    uint32_t packed = ((uint32_t)frame->data[1] << 24U) |
                      ((uint32_t)frame->data[2] << 16U) |
                      ((uint32_t)frame->data[3] << 8U) |
                      (uint32_t)frame->data[4];

    if (!sent_unpack_nibbles(packed,
                             bridge->config.data_nibbles,
                             bridge->config.order,
                             tx_frame.data_nibbles)) {
        return false;
    }

    uint16_t pause_ticks = 12U;
    if (frame->dlc >= 7U) {
        pause_ticks = (uint16_t)frame->data[5] | ((uint16_t)frame->data[6] << 8U);
    }

    return bridge->tx_hal.submit_frame(bridge->tx_hal.context, &tx_frame, &bridge->config, pause_ticks);
}

/* Public command API — each returns true on success. */

bool sent_bridge_start_rx(sent_bridge_t* b) {
    SENT_ASSERT(b != NULL);
    if (!b->config_valid) {
        return false;
    }
    bool ok = bridge_start_rx(b);
    if (ok) {
        bridge_stop_tx(b);           /* stop TX HAL (DMA/timer drains naturally) */
        sent_mode_manager_start_rx(&b->mode_manager);
    }
    return ok;
}

bool sent_bridge_start_tx(sent_bridge_t* b) {
    SENT_ASSERT(b != NULL);
    if (!b->config_valid) {
        return false;
    }
    bool ok = bridge_start_tx(b);
    if (ok) {
        bridge_stop_rx(b);
        sent_mode_manager_start_tx(&b->mode_manager);
    }
    return ok;
}

bool sent_bridge_stop(sent_bridge_t* b) {
    SENT_ASSERT(b != NULL);
    bridge_stop_all(b);
    if (b->learn.active) {
        /* Learning was abandoned: put the pre-learn configuration back. */
        b->learn.active = false;
        b->config = b->learn.saved_config;
        b->config_valid = sent_validate_config(&b->config);
    }
    sent_mode_manager_stop(&b->mode_manager);
    return true;
}

/* Set TX tick period on the TX HAL.
 * tick_x10_us is validated against SENT_BRIDGE_TX_{MIN,MAX}_TICK_X10 so the
 * host stays within the supported range. */
static bool handle_cmd_set_tx_tick(sent_bridge_t* b, uint16_t tick_x10_us) {
    if (tick_x10_us < SENT_BRIDGE_TX_MIN_TICK_X10 ||
        tick_x10_us > SENT_BRIDGE_TX_MAX_TICK_X10) {
        return false;
    }
    if (!b->has_tx_hal || b->tx_hal.set_tick_x10_us == NULL) {
        return false;
    }
    return b->tx_hal.set_tick_x10_us(b->tx_hal.context, tick_x10_us);
}

static bool handle_cmd_learn_tick(sent_bridge_t* b) {
    sent_config_t original = b->learn.active ? b->learn.saved_config : b->config;
    memset(&b->learn, 0, sizeof(b->learn));
    b->learn.saved_config = original;
    b->config = original;
    b->config.min_tick_x10_us = SENT_BRIDGE_LEARN_MIN_TICK_X10;
    b->config.max_tick_x10_us = SENT_BRIDGE_LEARN_MAX_TICK_X10;
    b->config_valid = sent_validate_config(&b->config);
    if (!b->config_valid) {
        b->config = b->learn.saved_config;
        b->config_valid = sent_validate_config(&b->config);
        return false;
    }
    bool ok = bridge_start_rx(b);
    if (ok) {
        bridge_stop_tx(b);
        b->learn.active = true;
        sent_mode_manager_start_rx(&b->mode_manager);
    } else {
        b->config = b->learn.saved_config;
        b->config_valid = sent_validate_config(&b->config);
    }
    return ok;
}

/* Initialize bridge with SENT config and RX/TX HAL backends.
 * @param bridge  bridge instance to initialize
 * @param config  SENT protocol configuration — required; NULL leaves the
 *                bridge in an unconfigured state and operations return errors
 * @param rx_hal  RX HAL interface (NULL if RX not used)
 * @param tx_hal  TX HAL interface (NULL if TX not used) */
void sent_bridge_init(sent_bridge_t* bridge,
                      const sent_config_t* config,
                      const sent_rx_hal_t* rx_hal,
                      const sent_tx_hal_t* tx_hal) {
    SENT_ASSERT(bridge != NULL);

    memset(bridge, 0, sizeof(*bridge));
    if (config != NULL) {
        bridge->config = *config;
        bridge->config_valid = sent_validate_config(&bridge->config);
    }
    bridge->output_can_id = SENT_CAN_ID_SENT_RX_FRAME;
    sent_slow_channel_init(&bridge->slow_channel);
    sent_mode_manager_init(&bridge->mode_manager);
    bridge->rx.skipped = RX_SKIPPED_LOST;

    if (rx_hal != NULL) {
        bridge->rx_hal = *rx_hal;
        bridge->has_rx_hal = true;
    }
    if (tx_hal != NULL) {
        bridge->tx_hal = *tx_hal;
        bridge->has_tx_hal = true;
    }
}

/* Process an incoming CAN frame: config (0x001), control (0x600), or TX (0x520).
 * @param bridge  bridge instance
 * @param frame   parsed CAN frame
 * @return        true if the frame was accepted (or ignored as unknown), false on
 *                a known operation that failed */
bool sent_bridge_on_can_frame(sent_bridge_t* bridge, const sent_can_frame_t* frame) {
    SENT_ASSERT(bridge != NULL && frame != NULL);

    /* Config frame: ID 0x001, each byte one SENT parameter.
     * byte 0: data nibbles (1..8)
     * byte 1: CRC mode (0=data-only, 1=status+data)
     * byte 2: CRC selector: low nibble = start value (0x03 = J2716 recommended,
     *         0x05 = field variant); bit 7 (0x80) = J2716 legacy CRC (FEB2008)
     * byte 3: min tick (units of 0.5 µs; stored as min_tick_x10_us = value*5)
     * byte 4: max tick (units of µs; stored as max_tick_x10_us = value*10)
     * byte 5-6: output RX CAN ID, big-endian (11-bit, 0x001-0x7FF) */
    if (frame->id == SENT_CAN_ID_SENT_CONFIG) {
        if (bridge->learn.active) {
            /* An explicit configuration ends a learn in progress. */
            bridge->learn.active = false;
            bridge->config = bridge->learn.saved_config;
        }
        if (frame->dlc >= 1U && frame->data[0] >= 1U && frame->data[0] <= SENT_MAX_DATA_NIBBLES) {
            bridge->config.data_nibbles = frame->data[0];
        }
        if (frame->dlc >= 2U) {
            bridge->config.crc_mode = (frame->data[1] != 0U)
                                          ? SENT_CRC_MODE_STATUS_AND_DATA
                                          : SENT_CRC_MODE_DATA_ONLY;
        }
        if (frame->dlc >= 3U && frame->data[2] != 0U) {
            bridge->config.crc_legacy = (frame->data[2] & SENT_BRIDGE_CRC_LEGACY_FLAG) != 0U;
            if ((frame->data[2] & 0x0FU) != 0U) {
                bridge->config.crc_init_seed = (uint8_t)(frame->data[2] & 0x0FU);
            }
        }
        if (frame->dlc >= 4U && frame->data[3] != 0U) {
            bridge->config.min_tick_x10_us = (uint16_t)frame->data[3] * 5U;
        }
        if (frame->dlc >= 5U && frame->data[4] != 0U) {
            bridge->config.max_tick_x10_us = (uint16_t)frame->data[4] * 10U;
        }
        if (frame->dlc >= 7U) {
            uint16_t id = ((uint16_t)frame->data[5] << 8U) | (uint16_t)frame->data[6];
            if (id >= 1U && id <= 0x7FFU) {
                bridge->output_can_id = id;
            }
        }
        bridge->config_valid = sent_validate_config(&bridge->config);
        bridge_reset_rx_stream(bridge);   /* frame length may have changed */
        return bridge->config_valid;
    }

    if (frame->id == SENT_CAN_ID_SENT_CONTROL && frame->dlc >= 1U) {
        uint8_t command = frame->data[0];

        if (command == SENT_BRIDGE_CMD_START_RX) {
            return sent_bridge_start_rx(bridge);
        }
        if (command == SENT_BRIDGE_CMD_START_TX) {
            return sent_bridge_start_tx(bridge);
        }
        if (command == SENT_BRIDGE_CMD_STOP) {
            return sent_bridge_stop(bridge);
        }
        if (command == SENT_BRIDGE_CMD_LEARN_TICK) {
            return handle_cmd_learn_tick(bridge);
        }
        if (command == SENT_BRIDGE_CMD_SET_TX_TICK && frame->dlc >= 3U) {
            uint16_t tick = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8U);
            return handle_cmd_set_tx_tick(bridge, tick);
        }
        return false;
    }

    if (frame->id == SENT_CAN_ID_SENT_TX_FRAME) {
        return handle_tx_can_frame(bridge, frame);
    }

    /* Unknown CAN ID — accept silently (host gets a positive ack from caller). */
    return true;
}

/* ── Learn mode ─────────────────────────────────────────────────────────────
 *
 * Works on the raw edge stream, so it needs no prior knowledge of tick time or
 * frame length.  After every edge, each nibble count 1..8 is tried on the
 * window that ENDS at the newest edge.  A window with a plausible sync and
 * valid nibble shapes is then checked against every CRC variant; a variant's
 * counter advances on a match and restarts on a mismatch, so only a candidate
 * that matches frame after frame can reach the required count (a wrong one
 * matches by chance 1 time in 16).
 *
 * Two look-alikes need extra care:
 *  - With constant sensor data a SHORTER candidate can keep matching by
 *    coincidence.  So once any candidate qualifies the search runs for one
 *    more frame and the longest qualifying nibble count wins.
 *  - A pause pulse of 12..27 ticks looks like one more nibble, and "data + CRC
 *    + constant" is itself a valid codeword for some other CRC variant, so a
 *    LONGER candidate can match every frame.  Its "CRC nibble" (really the
 *    pause) never changes although its data does; such candidates are passed
 *    over.
 * A sensor that sends constant data AND a pause pulse of 12..27 ticks cannot
 * be told apart from one with an extra data nibble; configure it manually. */

typedef struct {
    uint8_t seed;
    bool legacy;
    sent_crc_mode_t mode;
} learn_variant_t;

/* In order of preference when several qualify for the same nibble count. */
static const learn_variant_t k_learn_variants[SENT_BRIDGE_LEARN_VARIANTS] = {
    {0x03U, false, SENT_CRC_MODE_DATA_ONLY},        /* J2716 recommended */
    {0x03U, false, SENT_CRC_MODE_STATUS_AND_DATA},
    {0x05U, false, SENT_CRC_MODE_DATA_ONLY},        /* field variant */
    {0x05U, false, SENT_CRC_MODE_STATUS_AND_DATA},
    {0x05U, true,  SENT_CRC_MODE_DATA_ONLY},        /* J2716 legacy (FEB2008) */
    {0x05U, true,  SENT_CRC_MODE_STATUS_AND_DATA},
};

/* Adopt the learned format, stop RX and build the 0x601 result frame:
 *   data[0..1] tick [0.1 µs] little-endian, data[2] data nibbles,
 *   data[3] CRC mode (0 data-only, 1 status+data), data[4] CRC selector
 *   (same encoding as config byte 2). */
static void bridge_learn_commit(sent_bridge_t* bridge,
                                 uint8_t nibbles,
                                 const learn_variant_t* variant,
                                 sent_can_frame_t* out_can_frame) {
    uint16_t tick = bridge->learn.tick_x10[nibbles - 1U];
    uint16_t margin = (uint16_t)((uint32_t)tick * SENT_BRIDGE_LEARN_MARGIN_PCT / 100U);

    bridge->learn.active = false;
    bridge->learn.saved_config.min_tick_x10_us = (tick > margin) ? (uint16_t)(tick - margin) : 1U;
    bridge->learn.saved_config.max_tick_x10_us = (uint16_t)(tick + margin);
    bridge->learn.saved_config.data_nibbles = nibbles;
    bridge->learn.saved_config.crc_mode = variant->mode;
    bridge->learn.saved_config.crc_init_seed = variant->seed;
    bridge->learn.saved_config.crc_legacy = variant->legacy;
    bridge->config = bridge->learn.saved_config;
    bridge->config_valid = sent_validate_config(&bridge->config);

    bridge_stop_rx(bridge);
    sent_mode_manager_stop(&bridge->mode_manager);

    out_can_frame->id = SENT_CAN_ID_SENT_ACK;
    out_can_frame->extended = false;
    out_can_frame->dlc = 5U;
    for (uint8_t i = 0U; i < SENT_CAN_MAX_DLC; ++i) {
        out_can_frame->data[i] = 0U;
    }
    out_can_frame->data[0] = (uint8_t)(tick & 0xFFU);
    out_can_frame->data[1] = (uint8_t)((tick >> 8U) & 0xFFU);
    out_can_frame->data[2] = nibbles;
    out_can_frame->data[3] = (variant->mode == SENT_CRC_MODE_STATUS_AND_DATA) ? 1U : 0U;
    out_can_frame->data[4] = (uint8_t)(variant->seed |
                                       (variant->legacy ? SENT_BRIDGE_CRC_LEGACY_FLAG : 0U));
}

/* Process one edge while learning.
 * @return  true when the format was learned and out_can_frame holds the result */
static bool bridge_learn_edge(sent_bridge_t* bridge,
                               uint32_t timestamp,
                               sent_can_frame_t* out_can_frame) {
    /* Sliding window of the most recent edges. */
    if (bridge->rx.count >= SENT_MAX_TIMESTAMPS) {
        memmove(&bridge->rx.edges[0], &bridge->rx.edges[1],
                (SENT_MAX_TIMESTAMPS - 1U) * sizeof(bridge->rx.edges[0]));
        bridge->rx.count = (uint8_t)(SENT_MAX_TIMESTAMPS - 1U);
    }
    bridge->rx.edges[bridge->rx.count++] = timestamp;

    uint32_t units = bridge_ticks_per_us(bridge);
    bool any_qualified = false;

    for (uint8_t n = 1U; n <= SENT_MAX_DATA_NIBBLES; ++n) {
        uint8_t need = (uint8_t)(n + 4U);
        if (bridge->rx.count < need) {
            break;
        }

        sent_config_t try_cfg = bridge->config;   /* learn tick range */
        try_cfg.data_nibbles = n;
        try_cfg.crc_mode = SENT_CRC_MODE_DATA_ONLY;
        try_cfg.crc_init_seed = 0x03U;
        try_cfg.crc_legacy = false;

        sent_frame_t f;
        sent_decode_status_t status = SENT_DECODE_SYNC_ERROR;
        bool ok = sent_decode_from_timestamps(&try_cfg,
                                              &bridge->rx.edges[bridge->rx.count - need],
                                              need, units, &f, &status);
        if (!ok && status != SENT_DECODE_CRC_ERROR) {
            continue;   /* not a frame-shaped window for this nibble count */
        }

        /* Shapes are valid; f holds the nibbles and the received CRC. */
        bool any_match = false;
        for (uint8_t v = 0U; v < SENT_BRIDGE_LEARN_VARIANTS; ++v) {
            try_cfg.crc_mode = k_learn_variants[v].mode;
            try_cfg.crc_init_seed = k_learn_variants[v].seed;
            try_cfg.crc_legacy = k_learn_variants[v].legacy;
            uint8_t* hits = &bridge->learn.hits[n - 1U][v];
            if (sent_crc4_frame(&try_cfg, f.status, f.data_nibbles, n) == f.crc) {
                if (*hits < 0xFFU) {
                    (*hits)++;
                }
                any_match = true;
            } else {
                *hits = 0U;
            }
            if (*hits >= SENT_BRIDGE_LEARN_REQUIRED_HITS) {
                any_qualified = true;
            }
        }

        if (any_match) {
            /* Remember the tick and whether data / CRC nibble change from
             * frame to frame for this nibble count. */
            uint32_t data = sent_pack_nibbles(f.data_nibbles, n, SENT_NIBBLE_ORDER_MSB_FIRST);
            uint8_t track = bridge->learn.track[n - 1U];
            if ((track & SENT_BRIDGE_LEARN_TRACK_VALID) != 0U) {
                if (data != bridge->learn.last_data[n - 1U]) {
                    track |= SENT_BRIDGE_LEARN_TRACK_DATA_VARIED;
                }
                if ((track & 0x0FU) != f.crc) {
                    track |= SENT_BRIDGE_LEARN_TRACK_CRC_VARIED;
                }
            }
            track = (uint8_t)((track & 0xF0U) | SENT_BRIDGE_LEARN_TRACK_VALID | (f.crc & 0x0FU));
            bridge->learn.track[n - 1U] = track;
            bridge->learn.last_data[n - 1U] = data;
            bridge->learn.tick_x10[n - 1U] = f.tick_x10_us;
        }
    }

    if (bridge->learn.settle == 0U) {
        if (any_qualified) {
            bridge->learn.settle = SENT_BRIDGE_LEARN_SETTLE_EDGES;
        }
        return false;
    }
    if (--bridge->learn.settle != 0U) {
        return false;
    }

    /* Settling time is over: longest qualifying nibble count wins.  First pass
     * skips candidates whose CRC nibble stayed constant while their data
     * changed (a short pause pulse posing as the CRC); second pass takes
     * anything that qualified. */
    for (uint8_t pass = 0U; pass < 2U; ++pass) {
        for (uint8_t n = SENT_MAX_DATA_NIBBLES; n >= 1U; --n) {
            uint8_t track = bridge->learn.track[n - 1U];
            bool suspicious = (track & SENT_BRIDGE_LEARN_TRACK_DATA_VARIED) != 0U &&
                              (track & SENT_BRIDGE_LEARN_TRACK_CRC_VARIED) == 0U;
            if (pass == 0U && suspicious) {
                continue;
            }
            for (uint8_t v = 0U; v < SENT_BRIDGE_LEARN_VARIANTS; ++v) {
                if (bridge->learn.hits[n - 1U][v] >= SENT_BRIDGE_LEARN_REQUIRED_HITS) {
                    bridge_learn_commit(bridge, n, &k_learn_variants[v], out_can_frame);
                    return true;
                }
            }
        }
    }
    return false;   /* candidate fell away while settling; keep searching */
}

/* ── Normal reception ───────────────────────────────────────────────────────
 *
 * The window holds the most recent edges.  As soon as it contains one frame's
 * worth (sync + status + data + CRC = data_nibbles + 4 edges) it is decoded
 * with the oldest edge as the start of the sync pulse:
 *   - success: the frame is consumed; its last edge (end of the CRC nibble)
 *     stays as the first edge of whatever follows (next sync or pause pulse);
 *   - failure: the oldest edge is dropped and the search continues with the
 *     next edge.  This is how a pause pulse is stepped over and how alignment
 *     is recovered after a disturbance, for any tick time. */

/* Classify the single interval found between two good frames, relative to
 * the sync pulse length of the second one. */
typedef enum {
    GAP_PAUSE,   /* legal pause pulse, 12..768 ticks: the frames are consecutive */
    GAP_IDLE,    /* longer than any pause: the line was idle (transmitter stopped) */
    GAP_BAD,     /* shorter than 12 ticks: not a SENT pulse */
} gap_kind_t;

static gap_kind_t classify_gap(uint32_t interval, uint32_t sync) {
    uint64_t scaled = (uint64_t)interval * (2U * SENT_SYNC_TICKS);          /* ticks * 2 * sync */
    if (scaled < (uint64_t)sync * (2U * SENT_MIN_PAUSE_TICKS - 1U)) {       /* < 11.5 ticks */
        return GAP_BAD;
    }
    if (scaled > (uint64_t)sync * (2U * SENT_MAX_PAUSE_TICKS + 1U)) {       /* > 768.5 ticks */
        return GAP_IDLE;
    }
    return GAP_PAUSE;
}

/* Feed one falling-edge timestamp; see sent_bridge.h. */
bool sent_bridge_on_edge(sent_bridge_t* bridge,
                         uint32_t timestamp,
                         sent_can_frame_t* out_can_frame) {
    SENT_ASSERT(bridge != NULL && out_can_frame != NULL);
    if (!bridge->config_valid || !sent_mode_manager_is_rx(&bridge->mode_manager)) {
        return false;
    }

    if (bridge->learn.active) {
        return bridge_learn_edge(bridge, timestamp, out_can_frame);
    }

    uint8_t need = (uint8_t)(bridge->config.data_nibbles + 4U);
    if (bridge->rx.count >= need) {
        bridge->rx.count = 0U;   /* cannot happen; keeps the buffer bounded regardless */
    }
    bridge->rx.edges[bridge->rx.count++] = timestamp;
    if (bridge->rx.count < need) {
        return false;
    }

    sent_frame_t decoded;
    sent_decode_status_t decode_status = SENT_DECODE_SYNC_ERROR;
    if (!sent_decode_from_timestamps(&bridge->config, bridge->rx.edges, need,
                                     bridge_ticks_per_us(bridge), &decoded, &decode_status)) {
        if (decode_status == SENT_DECODE_CRC_ERROR) {
            /* A well-formed frame with a wrong checksum. */
            bridge->mode_manager.stats.crc_errors++;
            sent_slow_channel_reset(&bridge->slow_channel);
            bridge->rx.skipped = RX_SKIPPED_LOST;
        } else if (bridge->rx.skipped < RX_SKIPPED_LOST) {
            /* One discarded interval right after a good frame is normal (pause
             * pulse).  A second one means the frame that should be here is
             * malformed: count it once and consider alignment lost. */
            bridge->rx.skipped_interval = bridge->rx.edges[1] - bridge->rx.edges[0];
            bridge->rx.skipped++;
            if (bridge->rx.skipped == 2U) {
                bridge->mode_manager.stats.sync_errors++;
                sent_slow_channel_reset(&bridge->slow_channel);
                bridge->rx.skipped = RX_SKIPPED_LOST;
            }
        }
        memmove(&bridge->rx.edges[0], &bridge->rx.edges[1],
                (size_t)(need - 1U) * sizeof(bridge->rx.edges[0]));
        bridge->rx.count = (uint8_t)(need - 1U);
        return false;
    }

    uint32_t sync = bridge->rx.edges[1] - bridge->rx.edges[0];

    /* Is this frame the direct successor of the previous good one? */
    bool consecutive = (bridge->rx.skipped == 0U);
    if (bridge->rx.skipped == 1U) {
        gap_kind_t gap = classify_gap(bridge->rx.skipped_interval, sync);
        if (gap == GAP_PAUSE) {
            consecutive = true;
        } else if (gap == GAP_BAD) {
            bridge->mode_manager.stats.sync_errors++;
        } else {
            /* Idle line between frames (e.g. a one-shot generator): not an
             * error, but the frames are not consecutive, so a slow-channel
             * message must not be assembled across it. */
        }
    }
    if (!consecutive) {
        sent_slow_channel_reset(&bridge->slow_channel);
        bridge->rx.ref_sync = 0U;
    }

    /* Consume the frame; keep its last edge as the start of what follows. */
    bridge->rx.edges[0] = bridge->rx.edges[need - 1U];
    bridge->rx.count = 1U;
    bridge->rx.skipped = 0U;

    /* J2716 5.3.3 / 5.3.3.1: successive calibration pulses that differ by more
     * than 1/64 (1.5625 %) make the frame invalid. */
    uint32_t ref = bridge->rx.ref_sync;
    bridge->rx.ref_sync = sync;
    if (ref != 0U) {
        uint32_t diff = (sync >= ref) ? (sync - ref) : (ref - sync);
        if ((uint64_t)diff * 64U > ref) {
            bridge->mode_manager.stats.sync_errors++;
            sent_slow_channel_reset(&bridge->slow_channel);
            return false;
        }
    }

    bridge->mode_manager.stats.frames_decoded++;
    sent_slow_channel_set_crc(&bridge->slow_channel,
                              bridge->config.crc_init_seed, bridge->config.crc_legacy);
    sent_slow_message_t slow_message;
    sent_slow_message_t* slow_ptr = NULL;
    if (sent_slow_channel_process_status(&bridge->slow_channel, decoded.status, &slow_message)) {
        slow_ptr = &slow_message;
    }
    pack_rx_can_frame(&decoded, slow_ptr, bridge->output_can_id, out_can_frame);
    return true;
}

/* Edges were lost upstream; see sent_bridge.h. */
void sent_bridge_on_rx_overrun(sent_bridge_t* bridge) {
    SENT_ASSERT(bridge != NULL);
    bridge->mode_manager.stats.dropped_events++;
    bridge_reset_rx_stream(bridge);
    if (bridge->learn.active) {
        memset(bridge->learn.hits, 0, sizeof(bridge->learn.hits));
        memset(bridge->learn.track, 0, sizeof(bridge->learn.track));
        bridge->learn.settle = 0U;
    }
}
