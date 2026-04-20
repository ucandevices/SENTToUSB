#ifndef SENT_BRIDGE_H
#define SENT_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/can_frame.h"
#include "sent/hal.h"
#include "sent/mode_manager.h"
#include "sent/sent_protocol.h"
#include "sent/slcan.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_CAN_ID_SENT_CONFIG  0x001U
#define SENT_CAN_ID_SENT_CONTROL 0x600U
#define SENT_CAN_ID_SENT_ACK 0x601U
#define SENT_CAN_ID_SENT_RX_FRAME 0x510U
#define SENT_CAN_ID_SENT_DIAG 0x511U
#define SENT_CAN_ID_SENT_TX_FRAME 0x520U

#define SENT_BRIDGE_MAX_RESPONSES 2U

#define SENT_BRIDGE_CMD_START_RX   0x01U
#define SENT_BRIDGE_CMD_START_TX   0x02U
#define SENT_BRIDGE_CMD_STOP       0x03U
#define SENT_BRIDGE_CMD_LEARN_TICK 0x04U
#define SENT_BRIDGE_CMD_SET_TX_TICK 0x05U  /* data[1..2] = tick_x10_us little-endian */

/* Tick range used during learn mode: covers the full SENT spec (2-90 us tick). */
#define SENT_BRIDGE_LEARN_MIN_TICK_X10 20U
#define SENT_BRIDGE_LEARN_MAX_TICK_X10 900U

/* Margin applied around the learned tick value (±20%, in units of tick_x10). */
#define SENT_BRIDGE_LEARN_MARGIN_PCT 20U

/* Number of CRC-valid hits required per (nibble_count, crc_mode) combo before committing. */
#define SENT_BRIDGE_LEARN_REQUIRED_HITS 3U

/* Number of nibble-count candidates tried during learning: {4, 6, 8}. */
#define SENT_BRIDGE_LEARN_NIBBLE_COUNTS 3U

typedef struct {
    uint8_t count;
    char lines[SENT_BRIDGE_MAX_RESPONSES][SENT_SLCAN_MAX_LINE_LEN + 3U];
} sent_bridge_slcan_responses_t;

typedef struct {
    sent_config_t config;
    bool config_valid;
    sent_mode_manager_t mode_manager;
    sent_rx_hal_t rx_hal;
    sent_tx_hal_t tx_hal;
    bool has_rx_hal;
    bool has_tx_hal;
    uint16_t output_can_id;
    uint16_t serial_number;   /* 16-bit hash of MCU unique ID, set after init */
    struct {
        bool active;            /* true while searching for first valid frame to learn tick */
        sent_config_t saved_config; /* original config restored after learning */
        uint8_t hits[SENT_BRIDGE_LEARN_NIBBLE_COUNTS][2]; /* [nibble_idx][crc_mode] hit counts */
    } learn;
} sent_bridge_t;

void sent_bridge_init(sent_bridge_t* bridge,
                      const sent_config_t* config,
                      const sent_rx_hal_t* rx_hal,
                      const sent_tx_hal_t* tx_hal);

bool sent_bridge_on_slcan_line(sent_bridge_t* bridge,
                               const char* line,
                               sent_bridge_slcan_responses_t* out_responses);

bool sent_bridge_on_sent_timestamps_us(sent_bridge_t* bridge,
                                       const uint32_t* timestamps_us,
                                       size_t timestamp_count,
                                       sent_can_frame_t* out_can_frame);

bool sent_bridge_poll_rx_hal(sent_bridge_t* bridge, sent_can_frame_t* out_can_frame);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_BRIDGE_H */
