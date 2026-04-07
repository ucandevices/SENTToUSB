/*
 * sent_app.c — SENT bridge application layer for STM32F042 USB-to-SENT converter.
 *
 * This file wires three hardware resources to the SENT library:
 *
 *   USB CDC RX  → SLCAN command parser → bridge → TX HAL / RX mode control
 *   TIM2 CH3    → input-capture ISR   → RX HAL  (SENT signal from sensor on PA2)
 *   TIM14       → interval scheduler  → TX HAL  (SENT signal to DUT on PA4)
 *
 * SLCAN command summary (subset implemented by the bridge):
 *   'O'         Open channel — start SENT RX (enable frame forwarding to host)
 *   'C'         Close channel — stop RX
 *   'S'/'s'     Set baud rate (accepted, ignored — SENT uses fixed tick period)
 *   'V'/'v'     Hardware / firmware version
 *   'N'         Serial number
 *   'F'         Status flags
 *   't<id><dlc><data>\r'   Send CAN frame (11-bit ID) — used for SENT TX and control
 *
 * Control frames use CAN ID 0x600 (SENT_CAN_ID_SENT_CONTROL):
 *   data[0] = 0x01  Start RX mode
 *   data[0] = 0x02  Start TX mode
 *   data[0] = 0x04  Learn tick period from next sync pulse
 *
 * SENT TX frames use CAN ID 0x100 (SENT_CAN_ID_SENT_DATA):
 *   data[0]     = status nibble
 *   data[1..N]  = data nibbles (6 for MLX90377)
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

/* SLCAN input line accumulator (filled byte-by-byte from USB RX callback).
 * 64 bytes covers the longest SLCAN frame: 't' + 3-char ID + 1-char DLC + 16 data hex + '\0'. */
#define SLCAN_IN_BUF_SIZE 64U
static char     g_slcan_in[SLCAN_IN_BUF_SIZE];
static uint16_t g_slcan_in_len;

/* USB TX ring buffer — responses queued here, flushed from the main loop.
 * Decouples USB transmit (which may be busy) from ISR/callback context. */
#define USB_TX_BUF_SIZE 448U
static uint8_t  g_usb_tx[USB_TX_BUF_SIZE];
static uint16_t g_usb_tx_head;
static uint16_t g_usb_tx_tail;

/* ── TX ISR state ───────────────────────────────────────────────────────────
 *
 * TIM14 prescaler = 71 → 48 MHz / 72 = 666.67 kHz.
 * ARR = 1 → timer fires every 2 counts → period = 2 × 72 / 48 MHz = 3 µs = 1 SENT tick.
 *
 * Each SENT interval is a two-phase pulse on PA4 (active-LOW):
 *   LOW  phase: PA4 driven LOW for SENT_TX_LOW_TICKS ticks (15 µs).
 *   HIGH phase: PA4 driven HIGH for the remaining (interval − low_ticks) ticks.
 *
 * State machine driven by a simple tick-countdown in the ISR.  No dynamic ARR
 * changes: the timer always fires every 1 tick and the ISR just decrements
 * g_tx_ticks_left.  This eliminates the race between ARR writes and the running
 * counter that caused the HIGH phase to appear too short on a logic analyser.
 *
 * SAE J2716 requires the falling edge (LOW) to last at least 5 ticks (15 µs). */
#define SENT_TX_LOW_TICKS  5U   /* 5 × 3 µs = 15 µs active-LOW pulse */

/* phase: 0 = idle / load next interval, 1 = LOW phase, 2 = HIGH phase */
static volatile uint8_t  g_tx_phase      = 0U;
static volatile uint16_t g_tx_ticks_left = 0U;  /* ticks remaining in the current phase */
static volatile uint16_t g_tx_high_ticks = 0U;  /* HIGH-phase length for the current interval */

/* ── USB TX ring helpers ────────────────────────────────────────────────────── */

/* Append bytes to the ring buffer; silently drops on overflow */
static void usb_tx_push(const char *s, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++) {
        uint16_t next = (uint16_t)((g_usb_tx_head + 1U) % USB_TX_BUF_SIZE);
        if (next == g_usb_tx_tail) { break; }   /* buffer full — drop */
        g_usb_tx[g_usb_tx_head] = (uint8_t)s[i];
        g_usb_tx_head = next;
    }
}

/* Attempt to send the oldest contiguous segment to the USB CDC layer.
 * Called from the main loop only — CDC_Transmit_FS must not be called from ISR. */
static void usb_tx_flush(void)
{
    uint16_t tail = g_usb_tx_tail;
    uint16_t head = g_usb_tx_head;
    if (tail == head) { return; }   /* nothing to send */

    /* Send a contiguous chunk: from tail to end-of-buffer or head, whichever comes first.
     * The caller must re-call on the next iteration to handle the wrap-around segment. */
    uint16_t count = (head > tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(USB_TX_BUF_SIZE - tail);
    if (CDC_Transmit_FS(&g_usb_tx[tail], count) == USBD_OK) {
        tail = (uint16_t)(tail + count);
        if (tail >= USB_TX_BUF_SIZE) { tail = 0U; }
        g_usb_tx_tail = tail;
    }
}

/* ── TIM14 control ──────────────────────────────────────────────────────────── */

/* Kick TIM14 to start transmitting queued frames.
 * No-op if the timer is already running (TIM_CR1_CEN set).
 * Direct register access bypasses HAL's TIM state machine, which stays BUSY
 * after HAL_TIM_Base_Start_IT and blocks re-entry via the HAL API.
 *
 * ARR = 0: timer fires every 1 SENT tick (3 µs).  The ISR uses a software
 * countdown (g_tx_ticks_left) instead of dynamic ARR changes, which avoids
 * races between ARR writes and the running counter. */
static void tim14_kick(void)
{
    if ((htim14.Instance->CR1 & TIM_CR1_CEN) == 0U) {
        g_tx_phase      = 0U;   /* first ISR will load the first interval */
        g_tx_ticks_left = 0U;
        htim14.Instance->CNT    = 0U;
        htim14.Instance->ARR    = 1U;   /* period = 2 counts × 72 / 48 MHz = 3 µs = 1 SENT tick */
        htim14.Instance->SR     = 0U;   /* clear stale UIF so first IRQ is clean */
        htim14.Instance->DIER  |= TIM_IT_UPDATE;
        htim14.Instance->CR1   |= TIM_CR1_CEN;
    }
}

/* ── SLCAN dispatch ─────────────────────────────────────────────────────────── */

/* Parse and act on one complete SLCAN line (no trailing CR/LF).
 * Responses are queued in the USB TX ring buffer for flushing from the main loop. */
static void dispatch_slcan_line(const char *line)
{
    sent_bridge_slcan_responses_t resp;
    (void)sent_bridge_on_slcan_line(&g_bridge, line, &resp);

    /* Forward all response strings produced by the bridge */
    for (uint8_t i = 0U; i < resp.count; i++) {
        uint16_t len = (uint16_t)strlen(resp.lines[i]);
        usb_tx_push(resp.lines[i], len);   /* bridge strings already include \r terminator */
    }

    /* If the bridge accepted a TX data frame, start TIM14 if it is idle */
    if (sent_stm32f042_tx_pending_frames(&g_tx_hal) > 0U) {
        tim14_kick();
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void SentApp_Init(void)
{
    /* ── SENT configuration ── */
    /* Default: 3 µs/tick, 6 data nibbles, DATA_ONLY CRC (SAE J2716 APR2016),
     * pause interval enabled.  The host can override tick period with the
     * LEARN command (CAN 0x600, data[0]=0x04) after connecting the sensor. */
    sent_config_t cfg = sent_default_config();

    /* Q12 tick→µs conversion: mul=85 gives 2.996 µs instead of exact 3.000 µs.
     * A 56-tick sync measures as 167 µs rather than 168 µs.
     * Lower min_tick to 25 (×0.1 µs = 2.5 µs) so 167/56 = 2.98 µs still passes. */
    cfg.min_tick_x10_us = 25U;

    /* MLX90377 uses DATA_ONLY CRC (recommended by SAE J2716 APR2016) */
    cfg.crc_mode = SENT_CRC_MODE_DATA_ONLY;

    /* ── RX HAL ──
     * TIM2 runs at 48 MHz, 16-bit free-running.  PA2 = TIM2_CH3 (AF2), RISING edge.
     * capture_batch_size = nibbles + 4 edges: sync + status + N_data + CRC + leading edge
     *   of the next interval.  For MLX90377 (6 nibbles): 6+4 = 10 edges per frame.
     * sync_min_us = 100 µs: the HAL resets batch alignment when it sees an interval
     *   longer than this.  Max data nibble at 3 µs/tick = 15+12 = 81 µs < 100 µs,
     *   and sync = 56 ticks = 168 µs >> 100 µs, so this unambiguously marks frame start. */
    sent_stm32f042_rx_config_t rx_cfg = {
        .timer_clock_hz     = 48000000U,
        .timer_autoreload   = 0xFFFFU,
        .capture_batch_size = (uint8_t)(cfg.data_nibbles + 4U),
        .ready_queue_depth  = SENT_STM32F042_RX_MAX_READY_BATCHES,
        .sync_min_us        = 100U,
    };
    sent_stm32f042_rx_hal_init(&g_rx_hal, &rx_cfg);
    sent_rx_hal_t rx_hal;
    sent_stm32f042_make_rx_hal(&g_rx_hal, &rx_hal);

    /* ── TX HAL ──
     * pause_ticks = 12: pause interval appended after each frame (12 × 3 µs = 36 µs).
     * max_pending_frames must be < SENT_STM32F042_TX_QUEUE_DEPTH (4) to leave headroom. */
    sent_stm32f042_tx_config_t tx_cfg = { .default_pause_ticks = 12U, .max_pending_frames = 3U };
    sent_stm32f042_tx_hal_init(&g_tx_hal, &tx_cfg);
    sent_tx_hal_t tx_hal;
    sent_stm32f042_make_tx_hal(&g_tx_hal, &tx_hal);

    /* ── Bridge ── */
    sent_bridge_init(&g_bridge, &cfg, &rx_hal, &tx_hal);

    /* Derive a 16-bit serial number from the STM32F042 96-bit unique device ID
     * (three 32-bit words at 0x1FFFF7AC–0x1FFFF7B4) by XOR-folding.
     * Used in the SLCAN 'N' response so the host can identify each dongle. */
    const uint32_t uid0 = *(volatile const uint32_t*)0x1FFFF7ACU;
    const uint32_t uid1 = *(volatile const uint32_t*)0x1FFFF7B0U;
    const uint32_t uid2 = *(volatile const uint32_t*)0x1FFFF7B4U;
    const uint32_t h32  = uid0 ^ uid1 ^ uid2;
    g_bridge.serial_number = (uint16_t)(h32 ^ (h32 >> 16U));

    /* Clear buffers */
    memset(g_slcan_in, 0, sizeof(g_slcan_in));
    g_slcan_in_len = 0U;
    g_usb_tx_head  = 0U;
    g_usb_tx_tail  = 0U;
}

/* Called from TIM2 CH3 capture ISR (HAL_TIM_IC_CaptureCallback in main.c).
 * captured_counter is the raw 16-bit TIM2 value at the moment of the RISING edge.
 * The RX HAL accumulates these and detects frame boundaries via the sync interval. */
void SentApp_OnSentRxCaptureEdge(uint16_t captured_counter)
{
    sent_stm32f042_rx_on_capture_edge_isr(&g_rx_hal, captured_counter);
}

/* Called from TIM2 overflow ISR (HAL_TIM_PeriodElapsedCallback in main.c).
 * Notifies the RX HAL so it can extend 16-bit timestamps across counter rollovers. */
void SentApp_OnSentRxTimerOverflow(void)
{
    sent_stm32f042_rx_on_overflow_isr(&g_rx_hal);
}

/* Called from TIM14 update ISR (TIM14_IRQHandler in stm32f0xx_it.c).
 *
 * Fixed-period tick pump: TIM14 fires every 1 SENT tick (3 µs, ARR=0).
 * The ISR drives a 3-state machine via a software countdown:
 *
 *   g_tx_phase = 0  (idle / load)
 *     Pop the next interval from the TX HAL queue.
 *     If the queue is empty: idle PA4 HIGH, stop TIM14.
 *     Otherwise: pull PA4 LOW, load SENT_TX_LOW_TICKS into countdown, → phase 1.
 *
 *   g_tx_phase = 1  (LOW phase)
 *     Decrement countdown.  When it reaches 0: release PA4 HIGH, load HIGH-phase
 *     tick count into countdown, → phase 2.
 *
 *   g_tx_phase = 2  (HIGH phase)
 *     Decrement countdown.  When it reaches 0: → phase 0 (load next interval).
 *
 * Using a fixed ARR (no dynamic ARR changes) eliminates the race between an ARR
 * write and the running counter that caused the HIGH phase to appear too short. */
void SentApp_OnTim14UpdateIrq(void)
{
    /* Fast path: just count down the current phase */
    if (g_tx_ticks_left > 0U) {
        g_tx_ticks_left--;
        return;
    }

    /* Countdown expired — advance the state machine */
    if (g_tx_phase == 1U) {
        /* LOW phase complete: switch PA4 HIGH and start the HIGH phase countdown */
    	SENT_TX_GPIO_Port->BRR  = SENT_TX_Pin;
        g_tx_ticks_left = (uint16_t)(g_tx_high_ticks - 1U);
        g_tx_phase = 2U;

    } else {
        /* HIGH phase complete (or first ISR after kick): load the next interval */
        uint16_t ticks;
        if (!sent_stm32f042_tx_pop_next_interval_ticks_from_isr(&g_tx_hal, &ticks)) {
            /* Queue exhausted — frame done.  Idle PA4 HIGH and stop TIM14. */
            SENT_TX_GPIO_Port->BRR  = SENT_TX_Pin;
            htim14.Instance->CR1  &= ~TIM_CR1_CEN;
            htim14.Instance->DIER &= ~TIM_IT_UPDATE;
            return;
        }
        /* SAE J2716 minimum interval is 12 ticks > SENT_TX_LOW_TICKS (5), so the
         * subtraction never underflows with valid frames; clamp defensively. */
        uint16_t high = (ticks > SENT_TX_LOW_TICKS)
                      ? (uint16_t)(ticks - SENT_TX_LOW_TICKS)
                      : 1U;
        g_tx_high_ticks = high;
        SENT_TX_GPIO_Port->BSRR = SENT_TX_Pin;       /*TX is reverted by Transistor PA4 LOW — start of interval */
        g_tx_ticks_left = SENT_TX_LOW_TICKS - 1U;     /* count remaining LOW ticks */
        g_tx_phase = 1U;
    }
}

/* Called from USB CDC receive callback (USB interrupt context).
 * Accumulates bytes into a line buffer and dispatches complete SLCAN lines.
 * HAL_Delay() and other blocking calls must NOT be made here. */
void SentApp_OnUsbRx(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++) {
        char c = (char)data[i];

        if (c == '\r' || c == '\n') {
            /* CR or LF terminates a SLCAN command line */
            if (g_slcan_in_len > 0U) {
                g_slcan_in[g_slcan_in_len] = '\0';
                dispatch_slcan_line(g_slcan_in);
                g_slcan_in_len = 0U;
            }
        } else {
            /* Accumulate; silently discard if buffer is full (shouldn't happen with valid SLCAN) */
            if (g_slcan_in_len < (SLCAN_IN_BUF_SIZE - 1U)) {
                g_slcan_in[g_slcan_in_len++] = c;
            }
        }
    }
}

/* Main-loop pump — must be called as frequently as possible.
 *
 * RX path: polls the RX HAL for completed frame batches (assembled in ISR context),
 *   decodes each batch into a SENT frame, serialises as a SLCAN 't' line, and queues
 *   it in the USB TX ring buffer.
 *
 * TX path: managed entirely by TIM14 ISR + dispatch_slcan_line(); nothing to do here.
 *
 * USB flush: attempts to send the next contiguous ring-buffer segment to the host. */
void SentApp_Process(void)
{
    if (sent_mode_manager_is_rx(&g_bridge.mode_manager)) {
        /* Drain all completed RX batches from the ISR-filled queue */
        uint32_t timestamps[SENT_MAX_TIMESTAMPS];
        size_t count = SENT_MAX_TIMESTAMPS;
        while (g_bridge.rx_hal.poll_timestamps_us != NULL &&
               g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, timestamps, &count)) {
            sent_can_frame_t out_frame;
            if (sent_bridge_on_sent_timestamps_us(&g_bridge, timestamps, count, &out_frame)) {
                /* Encode as SLCAN: 't' + 3-nibble ID + 1-nibble DLC + data hex + '\r' */
                char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
                if (sent_slcan_serialize_frame(&out_frame, line, sizeof(line))) {
                    usb_tx_push(line, (uint16_t)strlen(line));
                    usb_tx_push("\r", 1U);
                }
            }
            count = SENT_MAX_TIMESTAMPS;   /* reset for next poll call */
        }
    } else {
        /* Not in RX mode: still drain the HAL queue to prevent overflow when/if
         * the sensor keeps sending (e.g. after 'C' closes the channel). */
        uint32_t timestamps[SENT_MAX_TIMESTAMPS];
        size_t count = SENT_MAX_TIMESTAMPS;
        while (g_bridge.rx_hal.poll_timestamps_us != NULL &&
               g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, timestamps, &count)) {
            count = SENT_MAX_TIMESTAMPS;
        }
    }

    usb_tx_flush();
}
