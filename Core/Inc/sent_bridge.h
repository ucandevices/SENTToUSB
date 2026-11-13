#ifndef SENT_BRIDGE_H
#define SENT_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"
#include "sent/hal.h"
#include "sent/mode_manager.h"
#include "sent/sent_protocol.h"
#include "sent/sent_slow_channel.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_CAN_ID_SENT_CONFIG  0x001U
#define SENT_CAN_ID_SENT_CONTROL 0x600U
#define SENT_CAN_ID_SENT_ACK 0x601U
#define SENT_CAN_ID_SENT_RX_FRAME 0x510U
#define SENT_CAN_ID_SENT_DIAG 0x511U
#define SENT_CAN_ID_SENT_TX_FRAME 0x520U

#define SENT_BRIDGE_CMD_START_RX   0x01U
#define SENT_BRIDGE_CMD_START_TX   0x02U
#define SENT_BRIDGE_CMD_STOP       0x03U
#define SENT_BRIDGE_CMD_LEARN_TICK 0x04U
#define SENT_BRIDGE_CMD_SET_TX_TICK 0x05U  /* data[1..2] = tick_x10_us little-endian */

/* Config frame (0x001) byte 2 and learn result (0x601) byte 4: CRC selector.
 * Low nibble = start value (0x3 recommended, 0x5 field variant); bit 7 set =
 * SAE J2716 legacy CRC (FEB2008 and older), in which case the nibble is unused. */
#define SENT_BRIDGE_CRC_LEGACY_FLAG 0x80U

/* Bits of the info byte that follows the data bytes in a 0x510 frame. */
#define SENT_BRIDGE_INFO_SLOW_NEW      0x01U  /* a slow-channel message completed with this frame */
#define SENT_BRIDGE_INFO_SLOW_ENHANCED 0x02U  /* enhanced serial message (else short) */
#define SENT_BRIDGE_INFO_SLOW_16BIT    0x04U  /* enhanced format with 16-bit data / 4-bit ID */
/* bits 7..4: status and communication nibble of the frame */

/* Tick range searched in learn mode: the J2716 nominal range (3-90 us) with
 * the +/-25 % a receiver has to tolerate (5.3.3), rounded outwards. */
#define SENT_BRIDGE_LEARN_MIN_TICK_X10 20U
#define SENT_BRIDGE_LEARN_MAX_TICK_X10 1130U

/* Tick range accepted by SET_TX_TICK. */
#define SENT_BRIDGE_TX_MIN_TICK_X10 20U
#define SENT_BRIDGE_TX_MAX_TICK_X10 900U

/* Margin applied around the learned tick value (±20%, in units of tick_x10). */
#define SENT_BRIDGE_LEARN_MARGIN_PCT 20U

/* Consecutive CRC-valid frames a (nibble count, CRC variant) candidate needs. */
#define SENT_BRIDGE_LEARN_REQUIRED_HITS 8U

/* CRC variants probed while learning: seed 0x3 / seed 0x5 / legacy, each
 * with and without the status nibble. */
#define SENT_BRIDGE_LEARN_VARIANTS 6U

/* After the first candidate qualifies, keep collecting this many edges (more
 * than one frame) so that a longer, equally valid candidate can overtake it. */
#define SENT_BRIDGE_LEARN_SETTLE_EDGES 16U

/* learn.track[] layout: low nibble = CRC nibble of the previous matching frame. */
#define SENT_BRIDGE_LEARN_TRACK_VALID       0x80U
#define SENT_BRIDGE_LEARN_TRACK_DATA_VARIED 0x40U
#define SENT_BRIDGE_LEARN_TRACK_CRC_VARIED  0x20U

typedef struct {
    sent_config_t config;
    bool config_valid;

    sent_mode_manager_t mode_manager;

    sent_rx_hal_t rx_hal;
    sent_tx_hal_t tx_hal;
    bool has_rx_hal;
    bool has_tx_hal;

    uint16_t output_can_id;

    sent_slow_channel_t slow_channel;

    /* Frame alignment on the raw falling-edge stream. */
    struct {
        uint32_t edges[SENT_MAX_TIMESTAMPS]; /* most recent edges, oldest first */
        uint8_t count;                       /* valid entries in edges[] */
        uint8_t skipped;                     /* edges discarded since the last good frame (saturates) */
        uint32_t skipped_interval;           /* length of the single discarded interval when skipped == 1 */
        uint32_t ref_sync;                   /* sync length of the previous frame; 0 = no reference */
    } rx;

    struct {
        bool active;                /* true while searching for the frame format */
        sent_config_t saved_config; /* original config restored after learning */
        uint8_t hits[SENT_MAX_DATA_NIBBLES][SENT_BRIDGE_LEARN_VARIANTS]; /* consecutive CRC matches */
        uint16_t tick_x10[SENT_MAX_DATA_NIBBLES]; /* last measured tick per nibble-count candidate */
        uint32_t last_data[SENT_MAX_DATA_NIBBLES]; /* data nibbles of the candidate's previous matching frame */
        uint8_t track[SENT_MAX_DATA_NIBBLES];      /* previous CRC nibble + SENT_BRIDGE_LEARN_TRACK_* flags */
        uint8_t settle;             /* edges left before committing; 0 = no candidate yet */
    } learn;
} sent_bridge_t;

void sent_bridge_init(sent_bridge_t* bridge,
                      const sent_config_t* config,
                      const sent_rx_hal_t* rx_hal,
                      const sent_tx_hal_t* tx_hal);

bool sent_bridge_start_rx(sent_bridge_t* bridge);
bool sent_bridge_start_tx(sent_bridge_t* bridge);
bool sent_bridge_stop(sent_bridge_t* bridge);

bool sent_bridge_on_can_frame(sent_bridge_t* bridge,
                              const sent_can_frame_t* frame);

/* Feed one falling-edge timestamp (unit: 1/rx_hal.ticks_per_us µs, wrapping
 * 32-bit counter).  Returns true when this edge completed something to send
 * to the host: a decoded frame (output CAN ID) or a learn result (0x601). */
bool sent_bridge_on_edge(sent_bridge_t* bridge,
                         uint32_t timestamp,
                         sent_can_frame_t* out_can_frame);

/* Tell the bridge that edges were lost upstream (capture ring overflow):
 * drops alignment and any partly received slow-channel message. */
void sent_bridge_on_rx_overrun(sent_bridge_t* bridge);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_BRIDGE_H */
