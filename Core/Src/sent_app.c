/*
 * sent_app.c — SENT bridge application layer for STM32F042 USB-to-SENT converter.
 *
 * This file wires three hardware resources to the SENT library:
 *
 *   USB CDC RX  → SLCAN command parser → bridge → TX HAL / RX mode control
 *   TIM2 CH3    → input-capture ISR   → RX HAL  (SENT signal from sensor on PA2)
 *   TIM3 CH3    → DMA-driven PWM1 output → TX HAL  (SENT signal to DUT on PB0)
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
 *   data[0] = 0x05  Set TX tick period (data[1..2] = tick_x10_us, little-endian)
 *
 * SENT TX uses TIM3_CH3 (PB0, AF1) in PWM Mode 1 with DMA1_Channel3 reloading ARR.
 * DMA reloads TIM3→ARR from a pre-computed cycle buffer on each timer update,
 * eliminating the per-interval ISR of the previous TIM14 software approach.
 *
 * SENT TX frames use CAN ID 0x100 (SENT_CAN_ID_SENT_DATA):
 *   data[0]     = status nibble
 *   data[1..N]  = data nibbles (6 for MLX90377)
 */

#include "sent_app.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include "sent_bridge.h"
#include "can_frame.h"
#include "sent/hal_stm32f042.h"
#include "slcan.h"
#include "sent/sent_protocol.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* TIM3 and DMA handles are defined in main.c */
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_up;

/* ── Static state ──────────────────────────────────────────────────────────── */

static sent_bridge_t           g_bridge;
static sent_stm32f042_rx_hal_t g_rx_hal;
static sent_stm32f042_tx_hal_t g_tx_hal;

/* 16-bit serial number derived from MCU UID, returned over SLCAN 'N' command. */
static uint16_t g_serial_number;

/* SLCAN input line accumulator (filled byte-by-byte from USB RX callback).
 * 64 bytes covers the longest SLCAN frame: 't' + 3-char ID + 1-char DLC + 16 data hex + '\0'. */
#define SLCAN_IN_BUF_SIZE 64U
static char     g_slcan_in[SLCAN_IN_BUF_SIZE];
static uint16_t g_slcan_in_len;

/* USB TX ring buffer — responses queued here, flushed from the main loop.
 * Decouples USB transmit (which may be busy) from ISR/callback context. */
#define USB_TX_BUF_SIZE 384U
static uint8_t  g_usb_tx[USB_TX_BUF_SIZE];
static uint16_t g_usb_tx_head;
static uint16_t g_usb_tx_tail;

/* ── Diagnostic counter snapshot (0x511 emission) ───────────────────────────
 * Last-emitted stats values.  A 0x511 diag frame is pushed to the host whenever
 * any of these change, so the viewer's CRC/sync error counters stay in sync
 * with the firmware without needing a host-initiated poll. */
static uint32_t g_last_frames_decoded = 0U;
static uint32_t g_last_crc_errors     = 0U;
static uint32_t g_last_sync_errors    = 0U;

/* ── TX DMA state ───────────────────────────────────────────────────────────
 *
 * The TX HAL pre-expands each SENT frame into alternating [LOW_ticks, HIGH_ticks]
 * pairs in g_tx_hal.intervals[].  tim3_dma_start() folds each pair into a single
 * total-interval ARR value (LOW+HIGH ticks → cycles−1) IN PLACE, writing the
 * result back into intervals[0..n_raw-1].  This avoids a separate DMA buffer.
 *
 * TIM3_CH3 (PB0, AF1) uses PWM Mode 1 with a fixed CCR3 = low_ticks × cycles_per_tick.
 * PWM Mode 1: OCxREF=1 when CNT < CCR3 (LOW pulse), OCxREF=0 otherwise (HIGH phase).
 * Through the gate transistor: OCxREF=1 → PB0 HIGH → transistor ON → SENT bus LOW.
 *
 * The sync interval (period 0) is special: OC3 is started in FORCED_ACTIVE so the
 * LOW pulse appears immediately on the bus, then a busy-wait + FORCED_INACTIVE
 * ends it, and PWM1 is queued to take effect at the first UEV.  See
 * tim3_dma_start() for the full rationale (AN4776 §1.4.2 OCxM preload behavior).
 *
 * DMA1_Channel3 reloads TIM3→ARR from intervals[1..n_raw-1] on each update event.
 * When DMA completes, the TC callback arms TIM_IT_UPDATE for one final cleanup event.
 * SentApp_OnTim3UpdateIrq() stops the counter and returns OC3 to FORCED_INACTIVE
 * (PB0 LOW → transistor OFF → SENT bus HIGH = idle). */

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

/* ── TIM3 DMA TX control ────────────────────────────────────────────────────── */

/* TIM3 clock = APB1 × 1 = 48 MHz (same as SystemCoreClock). */
#define TIM3_CLOCK_HZ 48000000U

/* DMA transfer-complete callback: all ARR values except the last have been loaded.
 * Disable the DMA request and arm the TIM3 update interrupt so the final interval
 * fires SentApp_OnTim3UpdateIrq() for cleanup.
 *
 * Note on the end-of-frame "glitch" rising edge: at end-of-pause UEV, the
 * cleanup ISR's CCMR2=FORCED_INACTIVE write lags the UEV by ~1-3 µs, during
 * which PWM1 sees CNT=0 < CCR3 and briefly drives OCxREF=1.  That brief LOW
 * becomes a small extra rising edge ~6 µs after pause's true rise.  Tempting
 * to suppress it by writing FORCED_INACTIVE here, but on STM32F0 OCxM is
 * NOT preload-controllable (AN4776 §1.4.2 talks about families where it is) —
 * the write would take effect immediately and truncate pause's LOW pulse,
 * collapsing the RX-measured CRC interval to ~1 µs and breaking decoding.
 * Leave the glitch in place; the RX HAL's sync detection at the next frame's
 * status edge re-aligns the batch and skips it. */
static void tim3_tx_dma_complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    TIM3->DIER &= ~TIM_DMA_UPDATE;
    TIM3->SR    = 0U;               /* clear any pending UIF before enabling IT */
    TIM3->DIER |= TIM_IT_UPDATE;
}

/* Start DMA-driven TX for the frame pre-loaded into g_tx_hal.
 * No-op if the timer is already running.
 *
 * Sequence:
 *   1. Fold each [LOW_ticks, HIGH_ticks] pair into a single total ARR value
 *      (cycles−1) written back to intervals[0..n_raw-1] in-place.
 *   2. URS+UG flush: pre-load CCR3, ARR=intervals[0] (sync), and CCMR2 with
 *      OC3 = FORCED_ACTIVE.  URS=1 keeps UG from generating a UEV/DMA request.
 *   3. CR1 |= CEN — counter starts.  OC3 is FORCED_ACTIVE so OCxREF=1 → PB0
 *      HIGH → transistor ON → bus LOW.  This is the sync's active-LOW pulse.
 *   4. Spin until CNT >= CCR3 (≈ 15 µs), then write OC3 = FORCED_INACTIVE
 *      (immediate effect: OCxREF=0, bus HIGH) to end the LOW pulse.  Then
 *      write OC3 = PWM1 — per AN4776 §1.4.2 OCxM is preload-like, so this
 *      mid-period write is held until the next UEV.
 *   5. Arm DMA to transfer intervals[1..n_raw-1] → TIM3→ARR on update events,
 *      then enable TIM_DMA_UPDATE.
 *   6. At CNT=ARR (end of sync, ≈ 168 µs) the OC unit reloads with PWM1 and
 *      DMA loads intervals[1] into ARR atomically — status interval starts
 *      with a clean PWM1 LOW pulse, and the rest of the frame runs DMA-only.
 *
 * After the last DMA transfer, tim3_tx_dma_complete() enables TIM_IT_UPDATE.
 * SentApp_OnTim3UpdateIrq() stops the timer and returns OC3 to FORCED_INACTIVE
 * (PB0 LOW → transistor OFF → SENT bus HIGH = idle).
 *
 * Why FORCED_ACTIVE for the sync's LOW pulse instead of just letting PWM1
 * handle it: empirically, transitioning OC3 from FORCED_INACTIVE → PWM1 with
 * the timer stopped does NOT make the OC comparator re-evaluate OCxREF on the
 * first counter tick — OCxREF stays at the FORCED_INACTIVE value (=0) for the
 * entire first period, suppressing the sync's LOW pulse.  Forcing OCxREF=1 via
 * FORCED_ACTIVE bypasses that. */
static void tim3_dma_start(void)
{
    if (TIM3->CR1 & TIM_CR1_CEN) { return; }   /* already running */

    uint8_t n = g_tx_hal.count;
    if (n == 0U || (n & 1U)) { return; }        /* must be even (LOW+HIGH pairs) */

    uint8_t n_raw = n >> 1U;                     /* number of complete SENT intervals */

    uint16_t tick_x10 = sent_stm32f042_tx_get_tick_x10_us(&g_tx_hal);
    if (tick_x10 == 0U) { tick_x10 = 30U; }
    uint32_t cycles_per_tick = ((TIM3_CLOCK_HZ / 1000000U) * (uint32_t)tick_x10) / 10U;
    if (cycles_per_tick < 2U) { cycles_per_tick = 2U; }

    uint8_t low_ticks = g_tx_hal.config.low_ticks;
    if (low_ticks == 0U) { low_ticks = 5U; }
    uint32_t ccr3 = (uint32_t)low_ticks * cycles_per_tick;
    if (ccr3 > 65535U) { ccr3 = 65535U; }

    /* Fold [LOW_ticks, HIGH_ticks, ...] pairs into total ARR values in-place.
     * intervals[i] = (LOW[i] + HIGH[i]) × cycles_per_tick − 1. */
    for (uint8_t i = 0U; i < n_raw; i++) {
        uint32_t total = (uint32_t)g_tx_hal.intervals[2U * i] +
                         (uint32_t)g_tx_hal.intervals[2U * i + 1U];
        uint32_t c = total * cycles_per_tick;
        if (c < 2U)     { c = 2U; }
        if (c > 65536U) { c = 65536U; }
        g_tx_hal.intervals[i] = (uint16_t)(c - 1U);
    }

    /* Flush stale TIM3 state from the previous frame.  URS=1 makes the upcoming
     * UG a preload-only refresh (no UEV → no DMA request).  Start in
     * FORCED_ACTIVE so OCxREF=1 the moment CEN goes 1 — this guarantees the
     * sync's 15 µs LOW pulse on the bus regardless of whatever transition
     * weirdness was suppressing it under PWM1. */
    TIM3->CR1   |= TIM_CR1_URS;
    TIM3->CCR3   = (uint16_t)ccr3;
    TIM3->ARR    = g_tx_hal.intervals[0];
    TIM3->CCMR2  = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_FORCED_ACTIVE;
    TIM3->EGR    = TIM_EGR_UG;
    TIM3->SR     = 0U;
    TIM3->CR1   &= ~TIM_CR1_URS;
    TIM3->CNT    = 0U;

    /* Start the counter while OC3 is FORCED_ACTIVE — bus drops LOW immediately
     * (start of the sync's 15 µs active-LOW pulse). */
    TIM3->CR1 |= TIM_CR1_CEN;

    /* Spin until CNT crosses CCR3 (= 15 µs LOW pulse), then close the LOW pulse
     * by writing OC3 = FORCED_INACTIVE — this is immediate (bus rises HIGH).
     * Then queue OC3 = PWM1; per AN4776 §1.4.2 OCxM is preload-like, so a
     * mid-period write to PWM1 is NOT picked up by the OC comparator until the
     * next UEV.  That's exactly what we want: at CNT=ARR (end of sync) the OC
     * unit reloads with PWM1 and the status interval's LOW pulse starts cleanly,
     * with the DMA-loaded intervals[1] becoming the new ARR at the same UEV. */
    while ((uint16_t)TIM3->CNT < (uint16_t)ccr3) { /* ~15 µs */ }
    TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_FORCED_INACTIVE;
    TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_PWM1;

    if (n_raw > 1U) {
        TIM3->DIER |= TIM_DMA_UPDATE;
        HAL_DMA_Start_IT(&hdma_tim3_up,
                          (uint32_t)&g_tx_hal.intervals[1],
                          (uint32_t)&TIM3->ARR,
                          (uint32_t)(n_raw - 1U));
    } else {
        TIM3->DIER |= TIM_IT_UPDATE;
    }
}

/* ── SLCAN dispatch ─────────────────────────────────────────────────────────── */

/* SLCAN frame ACK: lower-case "z\r" for standard, upper "Z\r" for extended,
 * BEL ("\a") for nack. Per SLCAN spec. */
static const char* slcan_frame_ack(bool ok, bool extended)
{
    if (!ok) { return "\a"; }
    return extended ? "Z\r" : "z\r";
}

/* Parse and act on one complete SLCAN line (no trailing CR/LF).
 * Responses are queued in the USB TX ring buffer for flushing from the main loop. */
static void dispatch_slcan_line(const char *line)
{
    sent_slcan_command_t parsed;
    if (!sent_slcan_parse_line(line, &parsed)) {
        return;
    }

    const char* resp = NULL;
    char nbuf[7];

    switch (parsed.type) {
    case SENT_SLCAN_CMD_OPEN:
    case SENT_SLCAN_CMD_LISTEN:
        resp = sent_bridge_start_rx(&g_bridge) ? "\r" : "\a";
        break;
    case SENT_SLCAN_CMD_CLOSE:
        (void)sent_bridge_stop(&g_bridge);
        resp = "\r";
        break;
    case SENT_SLCAN_CMD_SETBAUD:
        /* SENT/USB CDC: bitrate is irrelevant, always ack */
        resp = "\r";
        break;
    case SENT_SLCAN_CMD_VERSION:
        resp = "V0101\r";
        break;
    case SENT_SLCAN_CMD_FWVERSION:
        /* uCCBViewer calls .substring(1) on this; must not be empty */
        resp = "v0101\r";
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
        if (parsed.has_frame) {
            bool ok = sent_bridge_on_can_frame(&g_bridge, &parsed.frame);
            resp = slcan_frame_ack(ok, parsed.frame.extended);
        } else {
            resp = "\a";
        }
        break;
    }

    if (resp != NULL) {
        usb_tx_push(resp, (uint16_t)strlen(resp));
    }

    /* If the bridge accepted a TX data frame, start DMA TX if idle */
    if (sent_stm32f042_tx_pending_frames(&g_tx_hal) > 0U) {
        tim3_dma_start();
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void SentApp_Init(void)
{
    /* ── SENT configuration ──
     * 3 µs/tick (default), 6 data nibbles, MSB-first, DATA_ONLY CRC with
     * APR2016 seed (0x03).  The host can override tick period with the
     * LEARN command (CAN 0x600, data[0]=0x04) after connecting the sensor.
     *
     * min_tick_x10_us = 25 (2.5 µs): the Q12 tick→µs conversion uses
     * mul=85 which yields 2.996 µs instead of exact 3.000 µs, so a 56-tick
     * sync measures 167 µs rather than 168 µs.  167/56 = 2.98 µs still
     * passes with min_tick=25. */
    sent_config_t cfg;
    cfg.data_nibbles         = 6U;
    cfg.crc_mode             = SENT_CRC_MODE_DATA_ONLY;
    cfg.order                = SENT_NIBBLE_ORDER_MSB_FIRST;
    cfg.pause_pulse_enabled  = false;
    cfg.min_tick_x10_us      = 25U;
    cfg.max_tick_x10_us      = 900U;
    cfg.crc_init_seed        = 0x03U;

    /* ── RX HAL ──
     * TIM2 runs at 48 MHz, 16-bit free-running.  PA2 = TIM2_CH3 (AF2), RISING edge.
     * capture_batch_size = nibbles + 4 edges: sync-end + status-end + N_data-ends +
     *   CRC-end + pause-end.  For MLX90377 (6 nibbles): 6+4 = 10 edges per frame.
     *
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
     * pause_ticks = 12: pause interval appended after each frame (12 × tick).
     * low_ticks   = 5 : SAE J2716 minimum active-LOW pulse (5 ticks).
     * tx_tick_x10_us = 30: default 3.0 µs/tick.  Host may override at runtime
     *                      via control frame 0x600 data[0]=0x05 (SET_TX_TICK). */
    sent_stm32f042_tx_config_t tx_cfg = {
        .default_pause_ticks = 12U,
        .low_ticks           = 5U,
        .tx_tick_x10_us      = 30U,
    };
    sent_stm32f042_tx_hal_init(&g_tx_hal, &tx_cfg);
    sent_tx_hal_t tx_hal;
    sent_stm32f042_make_tx_hal(&g_tx_hal, &tx_hal);

    /* ── Bridge ── */
    sent_bridge_init(&g_bridge, &cfg, &rx_hal, &tx_hal);

    /* Register DMA TC callback — called by HAL_DMA_IRQHandler after all ARR values
     * have been transferred, to arm the final TIM3 update interrupt for cleanup. */
    hdma_tim3_up.XferCpltCallback = tim3_tx_dma_complete;

    /* Derive a 16-bit serial number from the STM32F042 96-bit unique device ID
     * (three 32-bit words at 0x1FFFF7AC–0x1FFFF7B4) by XOR-folding.
     * Used in the SLCAN 'N' response so the host can identify each dongle. */
    const uint32_t uid0 = *(volatile const uint32_t*)0x1FFFF7ACU;
    const uint32_t uid1 = *(volatile const uint32_t*)0x1FFFF7B0U;
    const uint32_t uid2 = *(volatile const uint32_t*)0x1FFFF7B4U;
    const uint32_t h32  = uid0 ^ uid1 ^ uid2;
    g_serial_number = (uint16_t)(h32 ^ (h32 >> 16U));

    /* Reset buffer cursors — buffers themselves are BSS (zero-initialised). */
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

/* Called from TIM3_IRQHandler (stm32f0xx_it.c) after DMA TC enables TIM_IT_UPDATE.
 * This fires exactly once per transmitted frame — after the last interval completes.
 * Stops the counter and returns TIM3_CH3 to forced-inactive so PB0 stays LOW (idle). */
void SentApp_OnTim3UpdateIrq(void)
{
    TIM3->CR1  &= ~TIM_CR1_CEN;
    TIM3->DIER &= ~TIM_IT_UPDATE;
    /* Forced-inactive: OCxREF = 0 → PB0 LOW → gate transistor OFF → SENT bus HIGH (idle). */
    TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_FORCED_INACTIVE;
    /* Release the TX HAL so the next frame can be submitted */
    g_tx_hal.count        = 0U;
    g_tx_hal.active_index = 0U;
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
 * TX path: managed entirely by TIM3 DMA + dispatch_slcan_line(); nothing to do here.
 *
 * USB flush: attempts to send the next contiguous ring-buffer segment to the host. */
/* Emit a diagnostic CAN frame (ID 0x511, DLC=8) when any counter changes.
 * Layout: data[0..3] = frames_decoded LE32, data[4..5] = crc_errors LE16,
 *         data[6..7] = sync_errors LE16.  Serialised as a SLCAN 't' line. */
static void emit_diag_if_changed(void)
{
    uint32_t frames = g_bridge.mode_manager.stats.frames_decoded;
    uint32_t crc    = g_bridge.mode_manager.stats.crc_errors;
    uint32_t sync   = g_bridge.mode_manager.stats.sync_errors;

    if (frames == g_last_frames_decoded &&
        crc    == g_last_crc_errors &&
        sync   == g_last_sync_errors) {
        return;
    }

    sent_can_frame_t diag;
    diag.id       = SENT_CAN_ID_SENT_DIAG;
    diag.extended = false;
    diag.dlc      = 8U;
    diag.data[0]  = (uint8_t)(frames & 0xFFU);
    diag.data[1]  = (uint8_t)((frames >> 8U)  & 0xFFU);
    diag.data[2]  = (uint8_t)((frames >> 16U) & 0xFFU);
    diag.data[3]  = (uint8_t)((frames >> 24U) & 0xFFU);
    diag.data[4]  = (uint8_t)(crc & 0xFFU);
    diag.data[5]  = (uint8_t)((crc >> 8U) & 0xFFU);
    diag.data[6]  = (uint8_t)(sync & 0xFFU);
    diag.data[7]  = (uint8_t)((sync >> 8U) & 0xFFU);

    char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
    if (sent_slcan_serialize_frame(&diag, line, sizeof(line))) {
        usb_tx_push(line, (uint16_t)strlen(line));
        usb_tx_push("\r", 1U);
        g_last_frames_decoded = frames;
        g_last_crc_errors     = crc;
        g_last_sync_errors    = sync;
    }
}

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

    emit_diag_if_changed();
    usb_tx_flush();
}
