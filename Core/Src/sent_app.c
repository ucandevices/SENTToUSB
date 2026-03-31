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
#define USB_TX_BUF_SIZE 448U
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

    /* TX HAL: pause 12 ticks, max 3 frames queued (must be < SENT_STM32F042_TX_QUEUE_DEPTH=4) */
    sent_stm32f042_tx_config_t tx_cfg = { .default_pause_ticks = 12U, .max_pending_frames = 3U };
    sent_stm32f042_tx_hal_init(&g_tx_hal, &tx_cfg);
    sent_tx_hal_t tx_hal;
    sent_stm32f042_make_tx_hal(&g_tx_hal, &tx_hal);

    /* Bridge */
    sent_bridge_init(&g_bridge, &cfg, &rx_hal, &tx_hal);

    /* Serial number: XOR-fold the STM32F042 96-bit unique device ID (3×32-bit
     * words at 0x1FFFF7AC) into a 16-bit value used in the SLCAN N response. */
    const uint32_t uid0 = *(volatile const uint32_t*)0x1FFFF7ACU;
    const uint32_t uid1 = *(volatile const uint32_t*)0x1FFFF7B0U;
    const uint32_t uid2 = *(volatile const uint32_t*)0x1FFFF7B4U;
    const uint32_t h32  = uid0 ^ uid1 ^ uid2;
    g_bridge.serial_number = (uint16_t)(h32 ^ (h32 >> 16U));

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
        uint32_t timestamps[SENT_MAX_TIMESTAMPS];
        size_t count = SENT_MAX_TIMESTAMPS;
        while (g_bridge.rx_hal.poll_timestamps_us != NULL &&
               g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, timestamps, &count)) {
            sent_can_frame_t out_frame;
            if (sent_bridge_on_sent_timestamps_us(&g_bridge, timestamps, count, &out_frame)) {
                char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
                if (sent_slcan_serialize_frame(&out_frame, line, sizeof(line))) {
                    usb_tx_push(line, (uint16_t)strlen(line));
                    usb_tx_push("\r", 1U);
                }
            }
            count = SENT_MAX_TIMESTAMPS;
        }
    } else {
        /* Not in RX mode: drain the RX HAL queue to prevent overflow but discard data. */
        uint32_t timestamps[SENT_MAX_TIMESTAMPS];
        size_t count = SENT_MAX_TIMESTAMPS;
        while (g_bridge.rx_hal.poll_timestamps_us != NULL &&
               g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, timestamps, &count)) {
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
