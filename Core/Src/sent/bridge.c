#include "sent/bridge.h"

#include "sent/sent_assert.h"
#include <string.h>

#include "sent/sent_decoder.h"
#include "sent/sent_encoder.h"

/* Append a response line to the SLCAN response buffer.
 * @param out_responses  [in/out] response collection to append to
 * @param line           null-terminated response string
 * @return               true if appended, false if buffer full */
static bool response_push(sent_bridge_slcan_responses_t* out_responses, const char* line) {
    SENT_ASSERT(out_responses != NULL && line != NULL);
    if (out_responses->count >= SENT_BRIDGE_MAX_RESPONSES) {
        return false;
    }

    size_t idx = out_responses->count;
    size_t len = strlen(line);
    if (len >= sizeof(out_responses->lines[idx])) {
        return false;
    }

    for (size_t i = 0U; i < len; ++i) {
        out_responses->lines[idx][i] = line[i];
    }
    out_responses->lines[idx][len] = '\0';
    out_responses->count++;
    return true;
}

/* Return SLCAN frame ACK ("z\r"/"Z\r") or NACK ("\a") per SLCAN spec.
 * Standard frames (t) use "z\r", extended frames (T) use "Z\r". */
static const char* frame_ack_line(bool ok, bool extended) {
    if (!ok) { return "\a"; }
    return extended ? "Z\r" : "z\r";
}

/* HAL helpers — guard + call, return true if no HAL present (no-op). */

static bool bridge_start_rx(sent_bridge_t* b) {
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
 * Nibbles are packed in pairs: data[i] = nibble[2i]<<4 | nibble[2i+1].
 * DLC = ceil(data_nibbles_count / 2).
 * @param f       decoded SENT frame
 * @param out_id  CAN ID to use for the output frame
 * @param out     [out] CAN frame to populate */
static void pack_rx_can_frame(const sent_frame_t* f,
                               uint16_t out_id,
                               sent_can_frame_t* out) {
    uint8_t n = f->data_nibbles_count;
    uint8_t bytes = (n + 1U) / 2U;  /* ceil(n/2) */

    memset(out, 0, sizeof(*out));
    out->id = out_id;
    out->extended = false;
    out->dlc = bytes;
    for (uint8_t i = 0U; i < bytes; ++i) {
        uint8_t hi = (2U * i     < n) ? (f->data_nibbles[2U * i]      & 0x0FU) : 0U;
        uint8_t lo = (2U * i + 1U < n) ? (f->data_nibbles[2U * i + 1U] & 0x0FU) : 0U;
        out->data[i] = (uint8_t)((hi << 4U) | lo);
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
    memset(&tx_frame, 0, sizeof(tx_frame));
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

/* Per-command handlers — each returns true on success. */

static bool handle_cmd_start_rx(sent_bridge_t* b) {
    if (!b->config_valid) {
        return false;
    }
    bool ok = bridge_start_rx(b);
    if (ok) {
        bridge_stop_tx(b);           /* stop TX HAL (TIM14 drains naturally) */
        sent_mode_manager_start_rx(&b->mode_manager);
    }
    return ok;
}

static bool handle_cmd_start_tx(sent_bridge_t* b) {
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

static bool handle_cmd_stop(sent_bridge_t* b) {
    bridge_stop_all(b);
    b->learn.active = false;
    sent_mode_manager_stop(&b->mode_manager);
    return true;
}

static bool handle_cmd_learn_tick(sent_bridge_t* b) {
    memset(&b->learn, 0, sizeof(b->learn));
    b->learn.saved_config = b->config;
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
 * @param config  SENT protocol configuration (NULL for defaults)
 * @param rx_hal  RX HAL interface (NULL if RX not used)
 * @param tx_hal  TX HAL interface (NULL if TX not used) */
void sent_bridge_init(sent_bridge_t* bridge,
                      const sent_config_t* config,
                      const sent_rx_hal_t* rx_hal,
                      const sent_tx_hal_t* tx_hal) {
    SENT_ASSERT(bridge != NULL);

    memset(bridge, 0, sizeof(*bridge));
    bridge->config = config != NULL ? *config : sent_default_config();
    bridge->config_valid = sent_validate_config(&bridge->config);
    bridge->output_can_id = SENT_CAN_ID_SENT_RX_FRAME;
    sent_mode_manager_init(&bridge->mode_manager);

    if (rx_hal != NULL) {
        bridge->rx_hal = *rx_hal;
        bridge->has_rx_hal = true;
    }
    if (tx_hal != NULL) {
        bridge->tx_hal = *tx_hal;
        bridge->has_tx_hal = true;
    }
}

/* Process an incoming SLCAN command line and produce response lines.
 * @param bridge         bridge instance
 * @param line           null-terminated SLCAN command string
 * @param out_responses  [out] response lines to send back over USB
 * @return               true if command was processed */
bool sent_bridge_on_slcan_line(sent_bridge_t* bridge,
                               const char* line,
                               sent_bridge_slcan_responses_t* out_responses) {
    SENT_ASSERT(bridge != NULL && line != NULL && out_responses != NULL);

    memset(out_responses, 0, sizeof(*out_responses));

    sent_slcan_command_t parsed;
    if (!sent_slcan_parse_line(line, &parsed)) {
        return false;
    }

    /* O / L — open channel: start SENT RX */
    if (parsed.type == SENT_SLCAN_CMD_OPEN || parsed.type == SENT_SLCAN_CMD_LISTEN) {
        bool ok = handle_cmd_start_rx(bridge);
        response_push(out_responses, ok ? "\r" : "\a");
        return true;
    }
    /* C — close channel: stop all activity */
    if (parsed.type == SENT_SLCAN_CMD_CLOSE) {
        handle_cmd_stop(bridge);
        response_push(out_responses, "\r");
        return true;
    }
    /* S/s — set bitrate: irrelevant for USB CDC, always ack */
    if (parsed.type == SENT_SLCAN_CMD_SETBAUD) {
        response_push(out_responses, "\r");
        return true;
    }
    /* V — hardware version */
    if (parsed.type == SENT_SLCAN_CMD_VERSION) {
        response_push(out_responses, "V0101\r");
        return true;
    }
    /* v — firmware version (uCCBViewer calls .substring(1) on this; must not be empty) */
    if (parsed.type == SENT_SLCAN_CMD_FWVERSION) {
        response_push(out_responses, "v0101\r");
        return true;
    }
    /* N — serial number derived from MCU unique ID hash */
    if (parsed.type == SENT_SLCAN_CMD_SERIAL) {
        static const char hex[16] = "0123456789ABCDEF";
        char nbuf[7] = {
            'N',
            hex[(bridge->serial_number >> 12) & 0xFU],
            hex[(bridge->serial_number >>  8) & 0xFU],
            hex[(bridge->serial_number >>  4) & 0xFU],
            hex[(bridge->serial_number >>  0) & 0xFU],
            '\r',
            '\0'
        };
        response_push(out_responses, nbuf);
        return true;
    }
    /* F — read status flags: no errors */
    if (parsed.type == SENT_SLCAN_CMD_STATUS) {
        response_push(out_responses, "F00\r");
        return true;
    }
    /* Unknown but syntactically valid commands: ack gracefully */
    if (parsed.type == SENT_SLCAN_CMD_UNSUPPORTED) {
        response_push(out_responses, "\r");
        return true;
    }
    /* Malformed / empty line: nack */
    if (parsed.type == SENT_SLCAN_CMD_INVALID) {
        response_push(out_responses, "\a");
        return true;
    }

    if (!parsed.has_frame) {
        response_push(out_responses, "\a");
        return true;
    }

    const sent_can_frame_t* frame = &parsed.frame;

    /* Config frame: ID 0x001, each byte one SENT parameter.
     * byte 0: data nibbles (4, 6, or 8)
     * byte 1: CRC mode (0=data-only, 1=status+data)
     * byte 2: CRC init seed (e.g. 0x03=APR2016, 0x05=legacy)
     * byte 3: min tick (units of 0.5 µs; stored as min_tick_x10_us = value*5)
     * byte 4: max tick (units of µs; stored as max_tick_x10_us = value*10)
     * byte 5-6: output RX CAN ID, big-endian (11-bit, 0x001-0x7FF) */
    if (frame->id == SENT_CAN_ID_SENT_CONFIG) {
        if (frame->dlc >= 1U && frame->data[0] >= 1U && frame->data[0] <= SENT_MAX_DATA_NIBBLES) {
            bridge->config.data_nibbles = frame->data[0];
        }
        if (frame->dlc >= 2U) {
            bridge->config.crc_mode = (frame->data[1] != 0U)
                                          ? SENT_CRC_MODE_STATUS_AND_DATA
                                          : SENT_CRC_MODE_DATA_ONLY;
        }
        if (frame->dlc >= 3U && frame->data[2] != 0U) {
            bridge->config.crc_init_seed = frame->data[2];
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
        response_push(out_responses, frame_ack_line(bridge->config_valid, frame->extended));
        return true;
    }

    if (frame->id == SENT_CAN_ID_SENT_CONTROL && frame->dlc >= 1U) {
        uint8_t command = frame->data[0];
        bool ok;

        if (command == SENT_BRIDGE_CMD_START_RX) {
            ok = handle_cmd_start_rx(bridge);
        } else if (command == SENT_BRIDGE_CMD_START_TX) {
            ok = handle_cmd_start_tx(bridge);
        } else if (command == SENT_BRIDGE_CMD_STOP) {
            ok = handle_cmd_stop(bridge);
        } else if (command == SENT_BRIDGE_CMD_LEARN_TICK) {
            ok = handle_cmd_learn_tick(bridge);
        } else {
            ok = false;
        }

        response_push(out_responses, frame_ack_line(ok, frame->extended));
        return true;
    }

    if (frame->id == SENT_CAN_ID_SENT_TX_FRAME) {
        bool ok = handle_tx_can_frame(bridge, frame);
        response_push(out_responses, frame_ack_line(ok, frame->extended));
        return true;
    }

    response_push(out_responses, frame_ack_line(true, frame->extended));
    return true;
}

/* Nibble-count candidates tried during learn mode. */
static const uint8_t k_learn_nibble_counts[SENT_BRIDGE_LEARN_NIBBLE_COUNTS] = {4U, 6U, 8U};

/* Try to learn tick, nibble_count and crc_mode from a timestamp batch.
 * Tries all SENT_BRIDGE_LEARN_NIBBLE_COUNTS × 2 combinations and requires
 * SENT_BRIDGE_LEARN_REQUIRED_HITS consecutive CRC-valid decodes for one combo.
 * Stops RX and populates out_can_frame (ID 0x601, DLC=4) when done.
 * @param bridge           bridge instance (learn.active must be true)
 * @param timestamps_us    timestamp batch
 * @param timestamp_count  batch size
 * @param out_can_frame    [out] 0x601 result frame
 * @return                 true when a combo reached the required hit count */
static bool bridge_try_learn(sent_bridge_t* bridge,
                              const uint32_t* timestamps_us,
                              size_t timestamp_count,
                              sent_can_frame_t* out_can_frame) {
    for (uint8_t ni = 0U; ni < SENT_BRIDGE_LEARN_NIBBLE_COUNTS; ++ni) {
        for (uint8_t cm = 0U; cm < 2U; ++cm) {
            sent_config_t try_cfg = bridge->config;
            try_cfg.data_nibbles = k_learn_nibble_counts[ni];
            try_cfg.crc_mode = (sent_crc_mode_t)cm;
            if (!sent_validate_config(&try_cfg)) {
                continue;
            }

            sent_frame_t decoded;
            sent_decode_status_t status = SENT_DECODE_SYNC_ERROR;
            if (!sent_decode_from_timestamps_us(&try_cfg, timestamps_us, timestamp_count, &decoded, &status)) {
                continue;
            }

            bridge->learn.hits[ni][cm]++;
            if (bridge->learn.hits[ni][cm] < SENT_BRIDGE_LEARN_REQUIRED_HITS) {
                continue;
            }

            /* Enough hits — commit learned parameters. */
            bridge->learn.active = false;

            uint16_t tick = decoded.tick_x10_us;
            uint16_t margin = (uint16_t)((uint32_t)tick * SENT_BRIDGE_LEARN_MARGIN_PCT / 100U);

            bridge->learn.saved_config.min_tick_x10_us = (tick > margin) ? (uint16_t)(tick - margin) : 1U;
            bridge->learn.saved_config.max_tick_x10_us = tick + margin;
            bridge->learn.saved_config.data_nibbles = k_learn_nibble_counts[ni];
            bridge->learn.saved_config.crc_mode = (sent_crc_mode_t)cm;
            bridge->config = bridge->learn.saved_config;
            bridge->config_valid = sent_validate_config(&bridge->config);

            bridge_stop_rx(bridge);
            sent_mode_manager_stop(&bridge->mode_manager);

            memset(out_can_frame, 0, sizeof(*out_can_frame));
            out_can_frame->id = SENT_CAN_ID_SENT_ACK;
            out_can_frame->extended = false;
            out_can_frame->dlc = 4U;
            out_can_frame->data[0] = (uint8_t)(tick & 0xFFU);
            out_can_frame->data[1] = (uint8_t)((tick >> 8U) & 0xFFU);
            out_can_frame->data[2] = k_learn_nibble_counts[ni];
            out_can_frame->data[3] = cm;
            return true;
        }
    }
    return false;
}

/* Decode SENT timestamps into a CAN frame (ID 0x510) for host output.
 * @param bridge           bridge instance
 * @param timestamps_us    array of falling-edge timestamps [us]
 * @param timestamp_count  number of timestamps
 * @param out_can_frame    [out] CAN frame with decoded SENT data
 * @return                 true if a valid frame was decoded */
bool sent_bridge_on_sent_timestamps_us(sent_bridge_t* bridge,
                                       const uint32_t* timestamps_us,
                                       size_t timestamp_count,
                                       sent_can_frame_t* out_can_frame) {
    SENT_ASSERT(bridge != NULL && timestamps_us != NULL && out_can_frame != NULL);
    if (!bridge->config_valid || !sent_mode_manager_is_rx(&bridge->mode_manager)) {
        return false;
    }

    if (bridge->learn.active) {
        return bridge_try_learn(bridge, timestamps_us, timestamp_count, out_can_frame);
    }

    sent_frame_t decoded;
    sent_decode_status_t decode_status = SENT_DECODE_SYNC_ERROR;
    if (!sent_decode_from_timestamps_us(&bridge->config, timestamps_us, timestamp_count, &decoded, &decode_status)) {
        if (decode_status == SENT_DECODE_CRC_ERROR) {
            bridge->mode_manager.stats.crc_errors++;
        } else {
            bridge->mode_manager.stats.sync_errors++;
        }
        return false;
    }

    bridge->mode_manager.stats.frames_decoded++;
    pack_rx_can_frame(&decoded, bridge->output_can_id, out_can_frame);
    return true;
}

/* Poll RX HAL for a timestamp batch, decode it, and return as a CAN frame.
 * @param bridge         bridge instance
 * @param out_can_frame  [out] CAN frame (ID 0x510) with decoded SENT data
 * @return               true if a frame was available and decoded */
bool sent_bridge_poll_rx_hal(sent_bridge_t* bridge, sent_can_frame_t* out_can_frame) {
    SENT_ASSERT(bridge != NULL && out_can_frame != NULL);
    if (!bridge->config_valid || !sent_mode_manager_is_rx(&bridge->mode_manager) ||
        !bridge->has_rx_hal || bridge->rx_hal.poll_timestamps_us == NULL) {
        return false;
    }

    uint32_t timestamps[SENT_MAX_TIMESTAMPS];
    size_t count = SENT_MAX_TIMESTAMPS;
    if (!bridge->rx_hal.poll_timestamps_us(bridge->rx_hal.context, timestamps, &count)) {
        return false;
    }

    return sent_bridge_on_sent_timestamps_us(bridge, timestamps, count, out_can_frame);
}
