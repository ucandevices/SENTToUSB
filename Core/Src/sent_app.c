/*
 * sent_app.c — Full SENT bridge adapter for STM32F042 USB-to-SENT converter.
 *
 * Wires the SENT library (bridge + STM32F042 HAL) to:
 *   - USB CDC RX  (SLCAN commands from host)
 *   - USB CDC TX  (SLCAN responses / decoded SENT frames to host)
 *   - TIM2 CH3    (SENT RX falling-edge capture ISR)
 *   - TIM2 OVF    (timer overflow ISR for extended timestamps)
 *   - TIM14       (SENT TX interval scheduler ISR)
 */

#include "sent_app.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include "sent/bridge.h"
#include "sent/can_frame.h"
#include "sent/hal_stm32f042.h"
#include "sent/slcan.h"
#include "sent/sent_decoder.h"
#include "sent/sent_protocol.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* TIM14 handle is defined in main.c */
extern TIM_HandleTypeDef htim14;

/* ── Static state ──────────────────────────────────────────────────────────── */

static sent_bridge_t           g_bridge;
static sent_stm32f042_rx_hal_t g_rx_hal;
static sent_stm32f042_tx_hal_t g_tx_hal;

/* ISR fire counters for loopback diagnostics */
volatile uint32_t g_tim14_isr_count = 0U;
volatile uint32_t g_rx_edge_count   = 0U;

/* SLCAN input line accumulator (filled from USB RX callback) */
#define SLCAN_IN_BUF_SIZE 64U
static char     g_slcan_in[SLCAN_IN_BUF_SIZE];
static uint16_t g_slcan_in_len;

/* USB TX ring buffer */
#define USB_TX_BUF_SIZE 512U
static uint8_t  g_usb_tx[USB_TX_BUF_SIZE];
static uint16_t g_usb_tx_head;
static uint16_t g_usb_tx_tail;

/* ── USB TX ring helpers ────────────────────────────────────────────────────── */

static void usb_tx_push(const char *s, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++) {
        uint16_t next = (uint16_t)((g_usb_tx_head + 1U) % USB_TX_BUF_SIZE);
        if (next == g_usb_tx_tail) { break; }   /* drop on overflow */
        g_usb_tx[g_usb_tx_head] = (uint8_t)s[i];
        g_usb_tx_head = next;
    }
}

static void usb_tx_flush(void)
{
    uint16_t tail = g_usb_tx_tail;
    uint16_t head = g_usb_tx_head;
    if (tail == head) { return; }

    uint16_t count = (head > tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(USB_TX_BUF_SIZE - tail);
    if (CDC_Transmit_FS(&g_usb_tx[tail], count) == USBD_OK) {
        tail = (uint16_t)(tail + count);
        if (tail >= USB_TX_BUF_SIZE) { tail = 0U; }
        g_usb_tx_tail = tail;
    }
}

/* ── RX loopback helpers ────────────────────────────────────────────────────── */

/* No-op stop_rx: keeps the stm32f042 RX HAL running even when the bridge
 * switches to TX mode, so the loopback signal is always captured by TIM2. */
static void rx_stop_noop(void *ctx) { (void)ctx; }

/* Parse and push MLX90377 angle/magnitude from a decoded 0x510 SENT CAN frame.
 *
 * MLX90377 6-nibble SENT fast-channel layout (SAE J2716, MSB-first):
 *   Nibbles[0..2] → 12-bit angle    (0 = 0°, 4095 ≈ 360°)
 *   Nibbles[3..5] → 12-bit magnitude (raw field strength)
 *
 * CAN frame data layout produced by pack_rx_can_frame (6 nibbles, MSB-first):
 *   data[0] = status nibble  (bits[3:2]=rolling counter, bit[1]=error)
 *   data[1] = 0x00           (upper byte of 24-bit packed value is empty)
 *   data[2] = n0<<4 | n1     (angle bits 11..4)
 *   data[3] = n2<<4 | n3     (angle bits 3..0  | magnitude bits 11..8)
 *   data[4] = n4<<4 | n5     (magnitude bits 7..0)
 *   data[5] = CRC nibble
 *
 * Output line: "MLX A:AAA M:MMM S:R E:E\r\n"
 *   A = 12-bit angle in hex (0x000-0xFFF)
 *   M = 12-bit magnitude in hex (0x000-0xFFF)
 *   S = 2-bit rolling counter (0-3)
 *   E = error bit (0=OK, 1=error) */
static void push_mlx90377_line(const sent_can_frame_t *cf)
{
    if (cf->id != SENT_CAN_ID_SENT_RX_FRAME || cf->dlc < 5U) {
        return;
    }
    uint16_t angle = ((uint16_t)cf->data[2] << 4) | (cf->data[3] >> 4);
    uint16_t mag   = ((uint16_t)(cf->data[3] & 0x0FU) << 8) | cf->data[4];
    uint8_t  stat  = cf->data[0] & 0x0FU;
    uint8_t  roll  = (uint8_t)((stat >> 2) & 0x03U);
    uint8_t  err   = (uint8_t)((stat >> 1) & 0x01U);

    static const char hex[] = "0123456789ABCDEF";
    char line[26];
    uint8_t i = 0U;
    line[i++] = 'M'; line[i++] = 'L'; line[i++] = 'X';
    line[i++] = ' '; line[i++] = 'A'; line[i++] = ':';
    line[i++] = hex[(angle >> 8U) & 0xFU];
    line[i++] = hex[(angle >> 4U) & 0xFU];
    line[i++] = hex[ angle        & 0xFU];
    line[i++] = ' '; line[i++] = 'M'; line[i++] = ':';
    line[i++] = hex[(mag >> 8U) & 0xFU];
    line[i++] = hex[(mag >> 4U) & 0xFU];
    line[i++] = hex[ mag        & 0xFU];
    line[i++] = ' '; line[i++] = 'S'; line[i++] = ':';
    line[i++] = (char)('0' + roll);
    line[i++] = ' '; line[i++] = 'E'; line[i++] = ':';
    line[i++] = (char)('0' + err);
    line[i++] = '\r'; line[i++] = '\n';
    usb_tx_push(line, (uint16_t)i);
}

/* Pack a decoded sent_frame_t into a CAN 0x510 frame and push to USB TX ring.
 * Format: data[0]=status, data[1..4]=nibbles packed big-endian MSB-first,
 *         data[5]=crc, data[6..7]=tick_x10_us little-endian. */
static void push_rx_frame(const sent_frame_t *f)
{
    sent_can_frame_t cf;
    memset(&cf, 0, sizeof(cf));
    cf.id       = SENT_CAN_ID_SENT_RX_FRAME;
    cf.extended = false;
    cf.dlc      = 8U;
    cf.data[0]  = f->status & 0x0FU;
    uint32_t packed = sent_pack_nibbles(f->data_nibbles,
                                        f->data_nibbles_count,
                                        SENT_NIBBLE_ORDER_MSB_FIRST);
    cf.data[1]  = (uint8_t)((packed >> 24U) & 0xFFU);
    cf.data[2]  = (uint8_t)((packed >> 16U) & 0xFFU);
    cf.data[3]  = (uint8_t)((packed >>  8U) & 0xFFU);
    cf.data[4]  = (uint8_t)( packed         & 0xFFU);
    cf.data[5]  = f->crc & 0x0FU;
    cf.data[6]  = (uint8_t)( f->tick_x10_us        & 0xFFU);
    cf.data[7]  = (uint8_t)((f->tick_x10_us >> 8U) & 0xFFU);

    char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
    if (sent_slcan_serialize_frame(&cf, line, sizeof(line))) {
        uint16_t len = (uint16_t)strlen(line);
        usb_tx_push(line, len);
        usb_tx_push("\r", 1U);
    }
}

/* ── TIM14 control ──────────────────────────────────────────────────────────── */

static void tim14_kick(void)
{
    /* Start TIM14 only if it is not already running */
    if ((htim14.Instance->CR1 & TIM_CR1_CEN) == 0U) {
        __HAL_TIM_SET_COUNTER(&htim14, 0U);
        __HAL_TIM_SET_AUTORELOAD(&htim14, 0U);  /* fire after 1 tick (3 µs) */
        HAL_TIM_Base_Start_IT(&htim14);
    }
}

/* ── Bridge SLCAN dispatch helper ───────────────────────────────────────────── */

static void dispatch_slcan_line(const char *line)
{
    sent_bridge_slcan_responses_t resp;
    (void)sent_bridge_on_slcan_line(&g_bridge, line, &resp);

    for (uint8_t i = 0U; i < resp.count; i++) {
        uint16_t len = (uint16_t)strlen(resp.lines[i]);
        /* Bridge response strings already include \r or \a terminator */
        usb_tx_push(resp.lines[i], len);
    }

    /* Kick TIM14 if TX HAL has frames waiting */
    if (sent_stm32f042_tx_pending_frames(&g_tx_hal) > 0U) {
        tim14_kick();
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void SentApp_Init(void)
{
    /* Default config: 3 µs tick, 6 nibbles, data-only CRC, pause enabled.
     * The host can override via the learn command (CAN 0x600 data[0]=0x04). */
    sent_config_t cfg = sent_default_config();
    /* Q12 tick→µs conversion underestimates by ~0.4% (mul=85 vs exact 85.83).
     * 56-tick sync measures as 167 µs instead of 168 µs.
     * Lower min_tick to 25 (2.5 µs) so 167 µs / 56 = 2.98 µs still passes. */
    cfg.min_tick_x10_us = 25U;
    /* MLX90377 uses DATA_ONLY CRC (SAE J2716 APR2016 recommended). */
    cfg.crc_mode = SENT_CRC_MODE_DATA_ONLY;

    /* RX HAL: 48 MHz timer clock, 16-bit autoreload (matches TIM2 init).
     * capture_batch_size = data_nibbles + 4: sync + status + N_data + CRC edges,
     *   plus the edge that starts the following pause/sync interval = 10 for MLX90377.
     * sync_min_us = 100 µs: the ISR resets the batch when it sees a long interval
     *   (sync = 168 µs >> max nibble = 81 µs at 3 µs tick).  This guarantees frame
     *   alignment regardless of which edge capture starts on at power-up. */
    sent_stm32f042_rx_config_t rx_cfg = {
        .timer_clock_hz    = 48000000U,
        .timer_autoreload  = 0xFFFFU,
        .capture_batch_size = (uint8_t)(cfg.data_nibbles + 4U),
        .ready_queue_depth  = SENT_STM32F042_RX_MAX_READY_BATCHES,
        .sync_min_us        = 100U,
    };
    sent_stm32f042_rx_hal_init(&g_rx_hal, &rx_cfg);
    sent_rx_hal_t rx_hal;
    sent_stm32f042_make_rx_hal(&g_rx_hal, &rx_hal);
    rx_hal.stop_rx = rx_stop_noop;  /* keep RX HAL alive during TX for loopback */

    /* TX HAL: pause 12 ticks, max 3 frames queued (must be < SENT_STM32F042_TX_QUEUE_DEPTH=4) */
    sent_stm32f042_tx_config_t tx_cfg = { .default_pause_ticks = 12U, .max_pending_frames = 3U };
    sent_stm32f042_tx_hal_init(&g_tx_hal, &tx_cfg);
    sent_tx_hal_t tx_hal;
    sent_stm32f042_make_tx_hal(&g_tx_hal, &tx_hal);

    /* Bridge */
    sent_bridge_init(&g_bridge, &cfg, &rx_hal, &tx_hal);

    /* Buffers */
    memset(g_slcan_in, 0, sizeof(g_slcan_in));
    g_slcan_in_len = 0U;
    g_usb_tx_head  = 0U;
    g_usb_tx_tail  = 0U;
}

/* Called from TIM2 CH3 capture ISR (HAL_TIM_IC_CaptureCallback in main.c) */
void SentApp_OnSentRxCaptureEdge(uint16_t captured_counter)
{
    g_rx_edge_count++;
    sent_stm32f042_rx_on_capture_edge_isr(&g_rx_hal, captured_counter);
}

/* Called from TIM2 overflow ISR (HAL_TIM_PeriodElapsedCallback in main.c) */
void SentApp_OnSentRxTimerOverflow(void)
{
    sent_stm32f042_rx_on_overflow_isr(&g_rx_hal);
}

/* Called from TIM14 ISR (SentApp_OnTim14UpdateIrq in main.c) */
uint8_t SentApp_PopNextTxIntervalTicks(uint16_t *out_ticks)
{
    return sent_stm32f042_tx_pop_next_interval_ticks_from_isr(&g_tx_hal, out_ticks) ? 1U : 0U;
}

/* Called from TIM14 ISR to check if more frames are pending */
uint8_t SentApp_HasPendingTxIntervals(void)
{
    return (sent_stm32f042_tx_pending_frames(&g_tx_hal) > 0U) ? 1U : 0U;
}

/* Called from USB CDC receive callback (usbd_cdc_if.c) */
void SentApp_OnUsbRx(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++) {
        char c = (char)data[i];

        if (c == '\r' || c == '\n') {
            if (g_slcan_in_len > 0U) {
                g_slcan_in[g_slcan_in_len] = '\0';
                dispatch_slcan_line(g_slcan_in);
                g_slcan_in_len = 0U;
            }
        } else {
            if (g_slcan_in_len < (SLCAN_IN_BUF_SIZE - 1U)) {
                g_slcan_in[g_slcan_in_len++] = c;
            }
        }
    }
}

/* Main-loop pump: drain decoded RX frames, flush USB TX */
void SentApp_Process(void)
{
    if (sent_mode_manager_is_rx(&g_bridge.mode_manager)) {
        /* Normal RX path: poll timestamps directly so we can dump raw data on failure */
        uint32_t timestamps[SENT_MAX_TIMESTAMPS];
        size_t count = SENT_MAX_TIMESTAMPS;
        while (g_bridge.rx_hal.poll_timestamps_us != NULL &&
               g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, timestamps, &count)) {
            uint32_t crc_before = g_bridge.mode_manager.stats.crc_errors;
            sent_can_frame_t out_frame;
            bool ok = sent_bridge_on_sent_timestamps_us(&g_bridge, timestamps, count, &out_frame);
            if (ok) {
                /* MLX90377: decode angle (nibbles 0-2) and magnitude (nibbles 3-5) */
                push_mlx90377_line(&out_frame);
                /* SLCAN frame for protocol-level tools */
                char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
                if (sent_slcan_serialize_frame(&out_frame, line, sizeof(line))) {
                    uint16_t len = (uint16_t)strlen(line);
                    usb_tx_push(line, len);
                    usb_tx_push("\r\n", 2U);
                }
            } else if (g_bridge.mode_manager.stats.crc_errors == crc_before + 1U &&
                       crc_before < 3U) {
                /* Dump raw intervals for first 3 CRC errors to see multiple frames */
                static const char hex[] = "0123456789ABCDEF";
                char buf[4];
                usb_tx_push("RAW ", 4U);
                for (size_t i = 1U; i < count; i++) {
                    uint32_t iv = timestamps[i] - timestamps[i - 1U];
                    buf[0] = hex[(iv >> 12U) & 0xFU];
                    buf[1] = hex[(iv >>  8U) & 0xFU];
                    buf[2] = hex[(iv >>  4U) & 0xFU];
                    buf[3] = hex[iv & 0xFU];
                    usb_tx_push(buf, 4U);
                    usb_tx_push(" ", 1U);
                }
                usb_tx_push("\r\n", 2U);
            }
            count = SENT_MAX_TIMESTAMPS;
        }

        /* Diagnostic heartbeat: ~1/s, shows edge count + active batch depth */
        static uint32_t rx_dbg_counter = 0U;
        if (++rx_dbg_counter >= 100000U) {
            rx_dbg_counter = 0U;
            static const char hex[] = "0123456789ABCDEF";
            uint8_t rxe  = (uint8_t)(g_rx_edge_count              & 0xFFU);
            uint8_t crc  = (uint8_t)(g_bridge.mode_manager.stats.crc_errors  & 0xFFU);
            uint8_t syn  = (uint8_t)(g_bridge.mode_manager.stats.sync_errors & 0xFFU);
            uint8_t frm  = (uint8_t)(g_bridge.mode_manager.stats.frames_decoded & 0xFFU);
            /* Format: "DBG E:EE F:FF C:CC S:SS\r\n"
             * F=frames decoded, C=crc errors, S=sync errors */
            char dbg[32];
            uint8_t i = 0U;
            dbg[i++]='D'; dbg[i++]='B'; dbg[i++]='G';
            dbg[i++]=' '; dbg[i++]='E'; dbg[i++]=':';
            dbg[i++]=hex[(rxe>>4)&0xF]; dbg[i++]=hex[rxe&0xF];
            dbg[i++]=' '; dbg[i++]='F'; dbg[i++]=':';
            dbg[i++]=hex[(frm>>4)&0xF]; dbg[i++]=hex[frm&0xF];
            dbg[i++]=' '; dbg[i++]='C'; dbg[i++]=':';
            dbg[i++]=hex[(crc>>4)&0xF]; dbg[i++]=hex[crc&0xF];
            dbg[i++]=' '; dbg[i++]='S'; dbg[i++]=':';
            dbg[i++]=hex[(syn>>4)&0xF]; dbg[i++]=hex[syn&0xF];
            dbg[i++]='\r'; dbg[i++]='\n';
            usb_tx_push(dbg, i);
        }
    } else {
        /* TX mode (or stopped): RX HAL still running due to rx_stop_noop.
         * Decode loopback timestamps directly, bypassing bridge mode check. */

        /* Debug: once per ~1s send RX HAL state so we can see progress */
        static uint32_t dbg_counter = 0U;
        if (++dbg_counter >= 100000U) {
            dbg_counter = 0U;
            char dbg[32];
            uint8_t ac      = g_rx_hal.active_count;
            uint8_t t14     = (uint8_t)(g_tim14_isr_count & 0xFFU);
            uint8_t rxe     = (uint8_t)(g_rx_edge_count   & 0xFFU);
            uint8_t tim_run = (uint8_t)((htim14.Instance->CR1 & 1U) != 0U);
            uint8_t dropped = (uint8_t)(g_rx_hal.dropped_batches & 0xFFU);
            /* Format: "tFFF5" + ac + tim14_isr_lo + rx_edge_lo + tim_run + dropped */
            dbg[0] = 't'; dbg[1] = 'F'; dbg[2] = 'F'; dbg[3] = 'F'; dbg[4] = '5';
            const char hex[] = "0123456789ABCDEF";
            dbg[5]  = hex[(ac >> 4) & 0xF]; dbg[6]  = hex[ac & 0xF];
            dbg[7]  = hex[(t14 >> 4) & 0xF]; dbg[8]  = hex[t14 & 0xF];
            dbg[9]  = hex[(rxe >> 4) & 0xF]; dbg[10] = hex[rxe & 0xF];
            dbg[11] = hex[(tim_run >> 4) & 0xF]; dbg[12] = hex[tim_run & 0xF];
            dbg[13] = hex[(dropped >> 4) & 0xF]; dbg[14] = hex[dropped & 0xF];
            dbg[15] = '\r'; dbg[16] = '\0';
            usb_tx_push(dbg, 16U);
        }

        uint32_t timestamps[SENT_MAX_TIMESTAMPS];
        size_t count = SENT_MAX_TIMESTAMPS;
        while (g_rx_hal.running &&
               g_bridge.rx_hal.poll_timestamps_us != NULL &&
               g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, timestamps, &count)) {
            sent_frame_t decoded;
            sent_decode_status_t status;
            if (sent_decode_from_timestamps_us(&g_bridge.config, timestamps, count,
                                               &decoded, &status)) {
                push_rx_frame(&decoded);
            } else {
                /* Send decode-failure debug: "tFFE5" + status byte */
                char df[10];
                const char hex[] = "0123456789ABCDEF";
                df[0]='t'; df[1]='F'; df[2]='F'; df[3]='E'; df[4]='1';
                df[5]=hex[((uint8_t)status >> 4) & 0xF];
                df[6]=hex[(uint8_t)status & 0xF];
                df[7]='\r'; df[8]='\0';
                usb_tx_push(df, 8U);
            }
            count = SENT_MAX_TIMESTAMPS;
        }
    }

    usb_tx_flush();
}

/* Called by main.c at startup to auto-start RX */
void SentApp_StartRxMode(void)
{
    sent_can_frame_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id       = SENT_CAN_ID_SENT_CONTROL;
    ctrl.extended = false;
    ctrl.dlc      = 1U;
    ctrl.data[0]  = SENT_BRIDGE_CMD_START_RX;

    char line[SENT_SLCAN_MAX_LINE_LEN + 1U];
    if (sent_slcan_serialize_frame(&ctrl, line, sizeof(line))) {
        dispatch_slcan_line(line);
    }
}

void SentApp_StartTxMode(void)
{
    sent_can_frame_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id       = SENT_CAN_ID_SENT_CONTROL;
    ctrl.extended = false;
    ctrl.dlc      = 1U;
    ctrl.data[0]  = SENT_BRIDGE_CMD_START_TX;

    char line[SENT_SLCAN_MAX_LINE_LEN + 1U];
    if (sent_slcan_serialize_frame(&ctrl, line, sizeof(line))) {
        dispatch_slcan_line(line);
    }
}

/* Send one loopback test frame: status=1, nibbles=[1,2,3,4,5,6] */
void SentApp_TestTxFrame(void)
{
    /* CAN 0x520: data[0]=status, data[1..4]=packed nibbles MSB-first */
    sent_can_frame_t tx_cmd;
    memset(&tx_cmd, 0, sizeof(tx_cmd));
    tx_cmd.id       = SENT_CAN_ID_SENT_TX_FRAME;
    tx_cmd.extended = false;
    tx_cmd.dlc      = 5U;
    /* status=1, nibbles [1,2,3,4,5,6] packed MSB-first: 0x00123456 */
    tx_cmd.data[0]  = 0x01U;
    tx_cmd.data[1]  = 0x00U;
    tx_cmd.data[2]  = 0x12U;
    tx_cmd.data[3]  = 0x34U;
    tx_cmd.data[4]  = 0x56U;

    char line[SENT_SLCAN_MAX_LINE_LEN + 1U];
    if (sent_slcan_serialize_frame(&tx_cmd, line, sizeof(line))) {
        dispatch_slcan_line(line);
    }
}
