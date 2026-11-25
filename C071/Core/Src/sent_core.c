/*
 * sent_core.c — MCU-independent application core: channels, host protocol,
 * USB byte rings, diagnostics.  See sent_core.h.
 *
 * SLCAN command summary:
 *   'O' / 'L'   start RX on every channel (one channel: exactly v1.1).
 *               'L' is listen-only: AUTO termination stays off (passive
 *               monitoring of a live link); 'O' uses the normal AUTO rule.
 *   'C'         stop every channel
 *   'S'/'s'     set baud rate (accepted, ignored)
 *   'V'/'v'     hardware / firmware version (V0200 / v0200)
 *   'N'         serial number
 *   'F'         status flags (always F00)
 *   't<id><dlc><data>'  frame for a channel (config / control / TX), see
 *               sent_bridge.h for the channel addressing.
 *
 * Control 0x600 with command 0x06 (any channel nibble) returns the device
 * information frame 0x60F:
 *   data[0] number of channels
 *   data[1] firmware version major (BCD), data[2] minor (BCD)
 *   data[3] mode of channels 0..3, 2 bits each (channel 0 in bits 1..0):
 *           0 off, 1 RX, 2 TX, 3 learn
 *   data[4] channel that is learning, 0xFF = none
 *   data[5] (rev 2b) bits 3..0: termination currently on, channel n in bit n;
 *           bits 7..4: termination setting is AUTO, channel n in bit 4+n
 *           (a channel whose AUTO bit is clear is forced to the state in
 *           bits 3..0)
 *   data[6] (rev 2b) board features, SENT_CORE_FEATURE_*: bit 0 switchable
 *           termination, bit 1 supply measurement (0x60E), bit 2 push-pull
 *           TX buffer.  0 on the F042 board.
 *   data[7] reserved, 0
 * (v0200 before rev 2b sent 0 in data[5..7], so old hosts are unaffected.)
 *
 * Control 0x600 with command 0x07 (rev 2b): termination of channel n,
 * data[1] = 0 off, 1 on, 2 auto (default: on in RX/learn after 'O', off after
 * 'L', off in TX and when the channel is off).  DLC < 2, a value > 2 or a channel that does not exist
 * is NACKed.  Power-up: every channel AUTO and off, so termination is off.
 *
 * Supply frame 0x60E (rev 2b, only when the platform measures it, i.e. the
 * C071 board while USB is configured), every SENT_CORE_SUPPLY_PERIOD_MS:
 *   data[0..1] sensor supply VS in mV LE, data[2] SENT_CORE_SUPPLY_*
 *   (bit 0 VS_EN on = USB path enabled, bit 1 load-switch fault).
 *   Rev 2d (DLC 3 -> 6, data[0..2] unchanged):
 *   data[3..4] sensor-rail current in mA LE, 0xFFFF = not measured;
 *   data[5] supply source: 0 USB, 1 EXT_IN (or no source on), 0xFF = not
 *   reported.
 */

#include "sent_core.h"

#include <string.h>

#include "can_frame.h"
#include "slcan.h"

/* One USB transfer is limited to one full-speed bulk packet.  The USB driver
 * copies the first packet into packet memory when the transfer is started
 * but reads any further packets from RAM later; with longer transfers the
 * ring could reuse those bytes before they were sent. */
#define USB_TX_MAX_CHUNK 64U

#define DIAG_PERIOD_MS 100U

#define USB_RX_MASK ((uint16_t)(SENT_USB_RX_RING_SIZE - 1U))
#if (SENT_USB_RX_RING_SIZE & (SENT_USB_RX_RING_SIZE - 1U)) != 0U
#error "SENT_USB_RX_RING_SIZE must be a power of two"
#endif

/* SLCAN input line accumulator.  64 bytes covers the longest SLCAN frame:
 * 'T' + 8-char ID + 1-char DLC + 16 data hex + '\0'. */
#define SLCAN_IN_BUF_SIZE 64U

typedef struct {
    uint32_t frames;           /* values last sent in 0x511 + 0x40n */
    uint32_t crc;
    uint32_t sync;
    uint32_t dropped;          /* values last sent in 0x512 + 0x40n */
    uint32_t usb_overflows;
    uint32_t last_ms;
    uint32_t guard;            /* value last sent in 0x512 + 0x40n */
    uint32_t last_lost_edges;  /* platform lost-edge count at the last check */
    uint32_t usb_overflow_count;
} channel_diag_t;

typedef struct {
    uint8_t setting;           /* SENT_BRIDGE_TERM_* */
    bool valid;                /* line_control() called at least once */
    bool term;                 /* last state handed to line_control() */
    sent_bridge_mode_t mode;
} channel_line_t;

static sent_core_platform_t g_platform;
static sent_bridge_t        g_bridge[SENT_NUM_CHANNELS];
static sent_bridge_learn_t  g_learn;          /* shared: one channel learns at a time */
static channel_diag_t       g_diag[SENT_NUM_CHANNELS];
static channel_line_t       g_line[SENT_NUM_CHANNELS];
static uint32_t             g_supply_last_ms;
static uint16_t             g_serial_number;
static uint8_t              g_rr_first;        /* channel served first in this pass */
/* Set by 'L' (SLCAN listen-only), cleared by 'O' and 'C': AUTO termination
 * stays off in RX/learn, so a live ECU-sensor link is monitored without the
 * 10 k + 2.2 nF load.  Forced ON/OFF (command 0x07) still wins. */
static bool                 g_listen_only;

static uint8_t           g_usb_rx[SENT_USB_RX_RING_SIZE];
static volatile uint16_t g_usb_rx_head;        /* written by the USB interrupt */
static volatile uint16_t g_usb_rx_tail;        /* written by the main loop */

static char     g_slcan_in[SLCAN_IN_BUF_SIZE];
static uint16_t g_slcan_in_len;

static uint8_t  g_usb_tx[SENT_USB_TX_BUF_SIZE];
static uint16_t g_usb_tx_head;
static uint16_t g_usb_tx_tail;

/* ── USB TX ring (main loop only) ───────────────────────────────────────── */

/* Append a complete line.  If it does not fit, the whole item is dropped and
 * false returned: the host must never see a truncated SLCAN line. */
static bool usb_tx_push(const char* s, uint16_t len)
{
    uint16_t used = (uint16_t)((g_usb_tx_head + SENT_USB_TX_BUF_SIZE - g_usb_tx_tail) % SENT_USB_TX_BUF_SIZE);
    if (len > (uint16_t)(SENT_USB_TX_BUF_SIZE - 1U - used)) {
        return false;
    }
    for (uint16_t i = 0U; i < len; i++) {
        g_usb_tx[g_usb_tx_head] = (uint8_t)s[i];
        g_usb_tx_head = (uint16_t)((g_usb_tx_head + 1U) % SENT_USB_TX_BUF_SIZE);
    }
    return true;
}

static bool usb_tx_push_frame(const sent_can_frame_t* frame)
{
    char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
    if (!sent_slcan_serialize_frame(frame, line, sizeof(line) - 1U)) {
        return true;   /* not representable; nothing was lost to the buffer */
    }
    size_t len = strlen(line);
    line[len++] = '\r';
    return usb_tx_push(line, (uint16_t)len);
}

/* Queue a frame produced by a channel; count it when the buffer is full. */
static void channel_push_frame(uint8_t ch, const sent_can_frame_t* frame)
{
    if (!usb_tx_push_frame(frame)) {
        g_diag[ch].usb_overflow_count++;
    }
}

/* Hand the oldest contiguous segment (at most one packet) to the USB device. */
static void usb_tx_flush(void)
{
    uint16_t tail = g_usb_tx_tail;
    uint16_t head = g_usb_tx_head;
    if (tail == head || g_platform.usb_transmit == NULL) {
        return;
    }
    uint16_t count = (head > tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(SENT_USB_TX_BUF_SIZE - tail);
    if (count > USB_TX_MAX_CHUNK) {
        count = USB_TX_MAX_CHUNK;
    }
    if (g_platform.usb_transmit(&g_usb_tx[tail], count)) {
        tail = (uint16_t)(tail + count);
        if (tail >= SENT_USB_TX_BUF_SIZE) {
            tail = 0U;
        }
        g_usb_tx_tail = tail;
    }
}

/* ── Host commands ──────────────────────────────────────────────────────── */

static const char* slcan_frame_ack(bool ok, bool extended)
{
    if (!ok) {
        return "\a";
    }
    return extended ? "Z\r" : "z\r";
}

static void push_info_frame(void)
{
    sent_can_frame_t info;
    memset(&info, 0, sizeof(info));
    info.id = SENT_CAN_ID_SENT_INFO;
    info.dlc = 8U;
    info.data[0] = (uint8_t)SENT_NUM_CHANNELS;
    info.data[1] = (uint8_t)(((SENT_FW_VERSION[0] - '0') << 4) | (SENT_FW_VERSION[1] - '0'));
    info.data[2] = (uint8_t)(((SENT_FW_VERSION[2] - '0') << 4) | (SENT_FW_VERSION[3] - '0'));
    info.data[4] = 0xFFU;
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        info.data[3] |= (uint8_t)((uint8_t)sent_bridge_mode(&g_bridge[ch]) << (2U * ch));
        if (sent_bridge_is_learning(&g_bridge[ch])) {
            info.data[4] = ch;
        }
        if (sent_core_termination_on(ch)) {
            info.data[5] |= (uint8_t)(1U << ch);
        }
        if (g_line[ch].setting == SENT_BRIDGE_TERM_AUTO) {
            info.data[5] |= (uint8_t)(0x10U << ch);
        }
    }
    info.data[6] = g_platform.features;
    (void)usb_tx_push_frame(&info);
}

/* ── Line state (TX buffer enable, termination, capture interrupt) ──────── */

static bool termination_wanted(uint8_t ch, sent_bridge_mode_t mode)
{
    switch (g_line[ch].setting) {
    case SENT_BRIDGE_TERM_ON:
        return true;
    case SENT_BRIDGE_TERM_OFF:
        return false;
    default:
        return !g_listen_only &&
               (mode == SENT_BRIDGE_MODE_RX || mode == SENT_BRIDGE_MODE_LEARN);
    }
}

/* Tell the platform about every channel whose mode or termination changed. */
static void apply_line_state(void)
{
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        channel_line_t* l = &g_line[ch];
        sent_bridge_mode_t mode = sent_bridge_mode(&g_bridge[ch]);
        bool term = termination_wanted(ch, mode);
        if (l->valid && l->mode == mode && l->term == term) {
            continue;
        }
        l->valid = true;
        l->mode = mode;
        l->term = term;
        if (g_platform.line_control != NULL) {
            g_platform.line_control(ch, mode, term);
        }
    }
}

/* Control command 0x07: data[1] = SENT_BRIDGE_TERM_*. */
static bool set_termination(const sent_can_frame_t* frame)
{
    uint8_t ch = SENT_BRIDGE_CTRL_CHANNEL(frame->data[0]);
    if (ch >= SENT_NUM_CHANNELS || frame->dlc < 2U || frame->data[1] > SENT_BRIDGE_TERM_AUTO) {
        return false;
    }
    g_line[ch].setting = frame->data[1];
    return true;
}

/* Route a host frame to the channel it addresses.  Frames for no channel are
 * accepted and ignored (v1.1 behaviour for unknown IDs), except control
 * frames for a channel that does not exist, which are refused (v1.1 refused
 * every unknown command). */
static bool route_frame(const sent_can_frame_t* frame)
{
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        if (sent_bridge_owns_frame(&g_bridge[ch], frame)) {
            return sent_bridge_on_can_frame(&g_bridge[ch], frame);
        }
    }
    return !(frame->id == SENT_CAN_ID_SENT_CONTROL && frame->dlc >= 1U);
}

/* Parse and act on one complete SLCAN line (no trailing CR/LF). */
static void dispatch_slcan_line(const char* line)
{
    sent_slcan_command_t parsed;
    if (!sent_slcan_parse_line(line, &parsed)) {
        return;
    }

    const char* resp = NULL;
    char nbuf[7];
    bool send_info = false;

    switch (parsed.type) {
    case SENT_SLCAN_CMD_OPEN:
    case SENT_SLCAN_CMD_LISTEN: {
        bool ok = true;
        g_listen_only = (parsed.type == SENT_SLCAN_CMD_LISTEN);
        for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
            ok = sent_bridge_start_rx(&g_bridge[ch]) && ok;
        }
        resp = ok ? "\r" : "\a";
        break;
    }
    case SENT_SLCAN_CMD_CLOSE:
        g_listen_only = false;
        for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
            (void)sent_bridge_stop(&g_bridge[ch]);
        }
        resp = "\r";
        break;
    case SENT_SLCAN_CMD_SETBAUD:
        resp = "\r";   /* SENT/USB CDC: bitrate is irrelevant, always ack */
        break;
    case SENT_SLCAN_CMD_VERSION:
        resp = "V" SENT_FW_VERSION "\r";
        break;
    case SENT_SLCAN_CMD_FWVERSION:
        /* uCCBViewer calls .substring(1) on this; must not be empty */
        resp = "v" SENT_FW_VERSION "\r";
        break;
    case SENT_SLCAN_CMD_SERIAL: {
        static const char hex[16] = "0123456789ABCDEF";
        nbuf[0] = 'N';
        nbuf[1] = hex[(g_serial_number >> 12) & 0xFU];
        nbuf[2] = hex[(g_serial_number >>  8) & 0xFU];
        nbuf[3] = hex[(g_serial_number >>  4) & 0xFU];
        nbuf[4] = hex[(g_serial_number >>  0) & 0xFU];
        nbuf[5] = '\r';
        nbuf[6] = '\0';
        resp = nbuf;
        break;
    }
    case SENT_SLCAN_CMD_STATUS:
        resp = "F00\r";
        break;
    case SENT_SLCAN_CMD_UNSUPPORTED:
        resp = "\r";
        break;
    case SENT_SLCAN_CMD_INVALID:
        resp = "\a";
        break;
    case SENT_SLCAN_CMD_FRAME:
        if (!parsed.has_frame) {
            resp = "\a";
        } else if (parsed.frame.id == SENT_CAN_ID_SENT_CONTROL && parsed.frame.dlc >= 1U &&
                   SENT_BRIDGE_CTRL_COMMAND(parsed.frame.data[0]) == SENT_BRIDGE_CMD_GET_INFO) {
            resp = slcan_frame_ack(true, parsed.frame.extended);
            send_info = true;
        } else if (parsed.frame.id == SENT_CAN_ID_SENT_CONTROL && parsed.frame.dlc >= 1U &&
                   SENT_BRIDGE_CTRL_COMMAND(parsed.frame.data[0]) == SENT_BRIDGE_CMD_SET_TERMINATION) {
            resp = slcan_frame_ack(set_termination(&parsed.frame), parsed.frame.extended);
        } else {
            resp = slcan_frame_ack(route_frame(&parsed.frame), parsed.frame.extended);
        }
        break;
    }

    /* Mode or termination may have changed: update TX_EN / TERM_EN / capture
     * interrupt first, so a TX frame never starts with the buffer disabled. */
    apply_line_state();

    if (resp != NULL) {
        (void)usb_tx_push(resp, (uint16_t)strlen(resp));
    }
    if (send_info) {
        push_info_frame();
    }

    /* If a channel accepted a TX data frame, start its timer if idle. */
    if (g_platform.tx_kick != NULL) {
        for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
            g_platform.tx_kick(ch);
        }
    }
}

/* Drain the USB RX ring: assemble lines and dispatch each complete one. */
static void process_usb_rx(void)
{
    uint16_t head = g_usb_rx_head;   /* snapshot; the ISR may add more meanwhile */
    uint16_t tail = g_usb_rx_tail;
    while (tail != head) {
        char c = (char)g_usb_rx[tail];
        tail = (uint16_t)((tail + 1U) & USB_RX_MASK);
        g_usb_rx_tail = tail;

        if (c == '\r' || c == '\n') {
            if (g_slcan_in_len > 0U) {
                g_slcan_in[g_slcan_in_len] = '\0';
                dispatch_slcan_line(g_slcan_in);
                g_slcan_in_len = 0U;
            }
        } else if (g_slcan_in_len < (SLCAN_IN_BUF_SIZE - 1U)) {
            g_slcan_in[g_slcan_in_len++] = c;
        } else {
            /* over-long line: keep discarding until the terminator */
        }
    }
}

/* ── Receive path ───────────────────────────────────────────────────────── */

/* Feed at most SENT_EDGE_BUDGET captured edges of one channel to its bridge. */
static void process_channel_edges(uint8_t ch)
{
    sent_bridge_t* b = &g_bridge[ch];
    if (b->rx_hal.poll_timestamps_us == NULL) {
        return;
    }

    uint32_t edges[16];
    uint32_t budget = SENT_EDGE_BUDGET;
    while (budget > 0U) {
        size_t count = (budget < 16U) ? (size_t)budget : 16U;
        if (!b->rx_hal.poll_timestamps_us(b->rx_hal.context, edges, &count)) {
            break;
        }
        budget -= (uint32_t)count;
        for (size_t i = 0U; i < count; i++) {
            sent_can_frame_t out_frame;
            if (sent_bridge_on_edge(b, edges[i], &out_frame)) {
                channel_push_frame(ch, &out_frame);
            }
        }
    }

    /* Edges lost upstream: the stream has a hole, so drop alignment and any
     * partial slow-channel message instead of decoding across it. */
    if (g_platform.rx_lost_edges != NULL) {
        uint32_t lost = g_platform.rx_lost_edges(ch);
        if (lost != g_diag[ch].last_lost_edges) {
            g_diag[ch].last_lost_edges = lost;
            sent_bridge_on_rx_overrun(b);
        }
    }
}

/* ── Diagnostics ────────────────────────────────────────────────────────── */

static void put_le32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
    p[2] = (uint8_t)((v >> 16U) & 0xFFU);
    p[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

/* Per channel, at most every DIAG_PERIOD_MS and only when a counter changed:
 *   0x511 + 0x40n (DLC 8, v1.1 layout): frames LE32, crc errors LE16, sync errors LE16
 *   0x512 + 0x40n (DLC 8): lost-edge events LE32 (each edge-rate guard trip
 *                 counts as one such event too), USB-buffer overflows LE16,
 *                 edge-rate guard trips LE16 (rev 2b; both LE16 saturate at
 *                 0xFFFF.  Before rev 2b data[4..7] was overflows LE32.)
 * The second frame only appears when edges or frames were actually lost, so
 * a v1.1 host sees exactly the v1.1 traffic in normal operation. */
static uint16_t sat16(uint32_t v)
{
    return (v > 0xFFFFU) ? 0xFFFFU : (uint16_t)v;
}

static void emit_diag_if_changed(uint8_t ch)
{
    const sent_runtime_stats_t* st = &g_bridge[ch].mode_manager.stats;
    channel_diag_t* d = &g_diag[ch];
    uint32_t guard = (g_platform.rx_guard_trips != NULL) ? g_platform.rx_guard_trips(ch) : 0U;

    bool main_changed = st->frames_decoded != d->frames || st->crc_errors != d->crc ||
                        st->sync_errors != d->sync;
    bool loss_changed = st->dropped_events != d->dropped || d->usb_overflow_count != d->usb_overflows ||
                        guard != d->guard;
    if (!main_changed && !loss_changed) {
        return;
    }
    uint32_t now = (g_platform.now_ms != NULL) ? g_platform.now_ms() : 0U;
    if ((now - d->last_ms) < DIAG_PERIOD_MS) {
        return;
    }
    d->last_ms = now;

    sent_can_frame_t diag;
    diag.extended = false;
    diag.dlc = 8U;
    if (main_changed) {
        diag.id = SENT_CAN_ID_DIAG(ch);
        put_le32(&diag.data[0], st->frames_decoded);
        diag.data[4] = (uint8_t)(st->crc_errors & 0xFFU);
        diag.data[5] = (uint8_t)((st->crc_errors >> 8U) & 0xFFU);
        diag.data[6] = (uint8_t)(st->sync_errors & 0xFFU);
        diag.data[7] = (uint8_t)((st->sync_errors >> 8U) & 0xFFU);
        channel_push_frame(ch, &diag);
        d->frames = st->frames_decoded;
        d->crc = st->crc_errors;
        d->sync = st->sync_errors;
    }
    if (loss_changed) {
        diag.id = SENT_CAN_ID_DIAG2(ch);
        put_le32(&diag.data[0], st->dropped_events);
        uint16_t ovf = sat16(d->usb_overflow_count);
        uint16_t trips = sat16(guard);
        diag.data[4] = (uint8_t)(ovf & 0xFFU);
        diag.data[5] = (uint8_t)(ovf >> 8U);
        diag.data[6] = (uint8_t)(trips & 0xFFU);
        diag.data[7] = (uint8_t)(trips >> 8U);
        channel_push_frame(ch, &diag);
        d->dropped = st->dropped_events;
        d->usb_overflows = d->usb_overflow_count;
        d->guard = guard;
    }
}

/* Sensor-supply frame 0x60E every SENT_CORE_SUPPLY_PERIOD_MS (platforms that
 * measure the supply only). */
static void emit_supply_status(void)
{
    if (g_platform.supply_status == NULL || g_platform.now_ms == NULL) {
        return;
    }
    uint32_t now = g_platform.now_ms();
    if ((now - g_supply_last_ms) < SENT_CORE_SUPPLY_PERIOD_MS) {
        return;
    }
    g_supply_last_ms = now;
    sent_core_supply_t s = {
        .vs_mv = 0U,
        .flags = 0U,
        .is_ma = SENT_CORE_SUPPLY_IS_NA,
        .source = SENT_CORE_SUPPLY_SRC_NA,
    };
    if (!g_platform.supply_status(&s)) {
        return;
    }
    sent_can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = SENT_CAN_ID_SENT_SUPPLY;
    f.dlc = 6U;
    f.data[0] = (uint8_t)(s.vs_mv & 0xFFU);
    f.data[1] = (uint8_t)(s.vs_mv >> 8U);
    f.data[2] = s.flags;
    f.data[3] = (uint8_t)(s.is_ma & 0xFFU);
    f.data[4] = (uint8_t)(s.is_ma >> 8U);
    f.data[5] = s.source;
    (void)usb_tx_push_frame(&f);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void sent_core_init(const sent_core_platform_t* platform,
                    const sent_config_t* default_config,
                    const sent_rx_hal_t rx_hal[SENT_NUM_CHANNELS],
                    const sent_tx_hal_t tx_hal[SENT_NUM_CHANNELS],
                    uint16_t serial_number)
{
    memset(&g_platform, 0, sizeof(g_platform));
    if (platform != NULL) {
        g_platform = *platform;
    }
    memset(&g_learn, 0, sizeof(g_learn));
    memset(g_diag, 0, sizeof(g_diag));
    memset(g_line, 0, sizeof(g_line));
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        g_line[ch].setting = SENT_BRIDGE_TERM_AUTO;
        const sent_rx_hal_t* rx = (rx_hal != NULL && rx_hal[ch].context != NULL) ? &rx_hal[ch] : NULL;
        const sent_tx_hal_t* tx = (tx_hal != NULL && tx_hal[ch].context != NULL) ? &tx_hal[ch] : NULL;
        sent_bridge_init(&g_bridge[ch], default_config, rx, tx);
        sent_bridge_set_channel(&g_bridge[ch], ch);
        sent_bridge_set_learn_storage(&g_bridge[ch], &g_learn);
        if (g_platform.rx_lost_edges != NULL) {
            g_diag[ch].last_lost_edges = g_platform.rx_lost_edges(ch);
        }
        if (g_platform.rx_guard_trips != NULL) {
            g_diag[ch].guard = g_platform.rx_guard_trips(ch);
        }
    }
    g_supply_last_ms = (g_platform.now_ms != NULL) ? g_platform.now_ms() : 0U;
    g_serial_number = serial_number;
    g_rr_first = 0U;
    g_listen_only = false;
    g_slcan_in_len = 0U;
    g_usb_rx_head = 0U;
    g_usb_rx_tail = 0U;
    g_usb_tx_head = 0U;
    g_usb_tx_tail = 0U;
    apply_line_state();   /* every channel: OFF, termination off */
}

/* USB receive callback (interrupt context).  Bytes that do not fit are
 * dropped, which makes the affected line fail to parse and be NACKed. */
void sent_core_on_usb_rx(const uint8_t* data, uint32_t len)
{
    uint16_t head = g_usb_rx_head;
    for (uint32_t i = 0U; i < len; i++) {
        uint16_t next = (uint16_t)((head + 1U) & USB_RX_MASK);
        if (next == g_usb_rx_tail) {
            break;   /* ring full */
        }
        g_usb_rx[head] = data[i];
        head = next;
    }
    g_usb_rx_head = head;
}

/* Main-loop pump.  Channels take turns (the first one served rotates), each
 * limited to SENT_EDGE_BUDGET edges per pass. */
void sent_core_process(void)
{
    process_usb_rx();

    for (uint8_t k = 0U; k < SENT_NUM_CHANNELS; k++) {
        uint8_t ch = (uint8_t)((g_rr_first + k) % SENT_NUM_CHANNELS);
        process_channel_edges(ch);
    }
    g_rr_first = (uint8_t)((g_rr_first + 1U) % SENT_NUM_CHANNELS);

    /* A learn can end on an edge (learn -> RX). */
    apply_line_state();

    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        emit_diag_if_changed(ch);
    }
    emit_supply_status();
    usb_tx_flush();
}

sent_bridge_t* sent_core_bridge(uint8_t channel)
{
    return (channel < SENT_NUM_CHANNELS) ? &g_bridge[channel] : NULL;
}

uint32_t sent_core_usb_overflows(uint8_t channel)
{
    return (channel < SENT_NUM_CHANNELS) ? g_diag[channel].usb_overflow_count : 0U;
}

uint8_t sent_core_termination_setting(uint8_t channel)
{
    return (channel < SENT_NUM_CHANNELS) ? g_line[channel].setting : SENT_BRIDGE_TERM_AUTO;
}

bool sent_core_termination_on(uint8_t channel)
{
    return (channel < SENT_NUM_CHANNELS) ? g_line[channel].term : false;
}
