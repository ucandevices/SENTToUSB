/*
 * sent_app.c — SENT bridge application layer for STM32F042 USB-to-SENT converter.
 *
 * This file wires three hardware resources to the SENT library:
 *
 *   USB CDC RX  → byte ring → (main loop) SLCAN parser → bridge → TX HAL / RX mode control
 *   TIM2 CH3    → input-capture ISR   → RX HAL  (SENT signal from sensor on PA2)
 *   TIM3 CH3    → DMA-driven PWM1 output → TX HAL  (SENT signal to DUT on PB0)
 *
 * Only two things run in interrupt context: storing captured edges (TIM2) and
 * storing received USB bytes.  Parsing, bridge state changes, decoding and all
 * USB transmit queueing happen in the main loop, so none of that state is
 * shared with an ISR.
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
 *   data[0] = 0x04  Learn tick period, nibble count and CRC variant from the live signal
 *   data[0] = 0x05  Set TX tick period (data[1..2] = tick_x10_us, little-endian)
 *
 * SENT TX uses TIM3_CH3 (PB0, AF1) in PWM Mode 1 with DMA1_Channel3 reloading ARR.
 * DMA reloads TIM3→ARR from a pre-computed cycle buffer on each timer update,
 * eliminating the per-interval ISR of the previous TIM14 software approach.
 *
 * SENT TX frames use CAN ID 0x520 (SENT_CAN_ID_SENT_TX_FRAME):
 *   data[0]     = status nibble
 *   data[1..4]  = data nibbles, packed
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

/* USB RX byte ring — filled by the USB receive callback (interrupt context),
 * drained by the main loop.  Single producer / single consumer. */
#define USB_RX_RING_SIZE 128U   /* power of two */
static uint8_t          g_usb_rx[USB_RX_RING_SIZE];
static volatile uint8_t g_usb_rx_head;
static volatile uint8_t g_usb_rx_tail;

/* SLCAN input line accumulator (main loop only).
 * 64 bytes covers the longest SLCAN frame: 'T' + 8-char ID + 1-char DLC + 16 data hex + '\0'. */
#define SLCAN_IN_BUF_SIZE 64U
static char     g_slcan_in[SLCAN_IN_BUF_SIZE];
static uint16_t g_slcan_in_len;

/* USB TX ring buffer (main loop only) — responses and decoded frames are queued
 * here and handed to the CDC layer whenever its previous transfer has finished. */
#define USB_TX_BUF_SIZE 384U
static uint8_t  g_usb_tx[USB_TX_BUF_SIZE];
static uint16_t g_usb_tx_head;
static uint16_t g_usb_tx_tail;

/* ── Diagnostic counter snapshot (0x511 emission) ───────────────────────────
 * Last-emitted stats values.  A 0x511 diag frame is pushed to the host when any
 * of these changed, at most once per DIAG_PERIOD_MS, so the viewer's counters
 * follow the firmware without a host poll and without doubling USB traffic. */
#define DIAG_PERIOD_MS 100U
static uint32_t g_last_frames_decoded = 0U;
static uint32_t g_last_crc_errors     = 0U;
static uint32_t g_last_sync_errors    = 0U;
static uint32_t g_last_diag_ms        = 0U;

/* Edges the capture ring had dropped when the bridge was last told about it. */
static uint32_t g_last_dropped_edges  = 0U;

/* ── TX DMA state ───────────────────────────────────────────────────────────
 *
 * The TX HAL pre-expands each SENT frame into alternating [LOW_ticks, HIGH_ticks]
 * pairs in g_tx_hal.intervals[].  tim3_dma_start() folds each pair into a single
 * total-interval ARR value (LOW+HIGH ticks → timer counts−1) IN PLACE, writing the
 * result back into intervals[0..n_raw-1].  This avoids a separate DMA buffer.
 *
 * The timer prescaler is chosen per frame so that the longest interval (sync at
 * a long tick time, or a long pause pulse) still fits the 16-bit ARR.
 *
 * TIM3_CH3 (PB0, AF1) uses PWM Mode 1 with a fixed CCR3 = low_ticks in timer counts.
 * PWM Mode 1: OCxREF=1 when CNT < CCR3 (LOW pulse), OCxREF=0 otherwise (HIGH phase).
 * Through the gate transistor: OCxREF=1 → PB0 HIGH → transistor ON → SENT bus LOW.
 *
 * The sync interval (period 0) is special: OC3 is started in FORCED_ACTIVE so the
 * LOW pulse appears immediately on the bus, then a busy-wait + FORCED_INACTIVE
 * ends it and PWM1 takes over for the rest of the frame.
 *
 * DMA1_Channel3 reloads TIM3→ARR from intervals[1..n_raw-1] on each update event.
 * When DMA completes, the TC callback arms TIM_IT_UPDATE for one final cleanup event.
 * SentApp_OnTim3UpdateIrq() stops the counter and returns OC3 to FORCED_INACTIVE
 * (PB0 LOW → transistor OFF → SENT bus HIGH = idle). */

/* ── USB TX ring helpers ────────────────────────────────────────────────────── */

/* Append a complete response/line to the ring.  If it does not fit, the whole
 * item is dropped: the host must never see a truncated SLCAN line. */
static void usb_tx_push(const char *s, uint16_t len)
{
    uint16_t used = (uint16_t)((g_usb_tx_head + USB_TX_BUF_SIZE - g_usb_tx_tail) % USB_TX_BUF_SIZE);
    if (len > (uint16_t)(USB_TX_BUF_SIZE - 1U - used)) {
        return;   /* host not reading fast enough — drop */
    }
    for (uint16_t i = 0U; i < len; i++) {
        g_usb_tx[g_usb_tx_head] = (uint8_t)s[i];
        g_usb_tx_head = (uint16_t)((g_usb_tx_head + 1U) % USB_TX_BUF_SIZE);
    }
}

/* Queue one CAN frame as an SLCAN 't' line terminated by CR. */
static void usb_tx_push_frame(const sent_can_frame_t *frame)
{
    char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
    if (sent_slcan_serialize_frame(frame, line, sizeof(line) - 1U)) {
        size_t len = strlen(line);
        line[len++] = '\r';
        usb_tx_push(line, (uint16_t)len);
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

/* Output-stage compensation.  PB0 drives the gate of the open-drain transistor
 * through 10 kΩ, so the transistor switches off roughly 11 µs after PB0 goes
 * low (measured: a 15 µs pulse on PB0 is 26..29 µs LOW on the bus).  At short
 * tick times that leaves the bus almost no HIGH time, and a gate that has not
 * fully discharged makes the next falling edge come early — a data-dependent
 * timing error of up to half a tick at 3 µs.
 * PB0 is therefore driven for the nominal LOW time minus TX_LOW_STRETCH_US,
 * but never shorter than TX_LOW_MIN_US so the gate still reaches its threshold
 * with margin.  The bus LOW time stays above the 4 ticks J2716 requires. */
#define TX_LOW_STRETCH_US 10U
#define TX_LOW_MIN_US      8U

/* DMA transfer-complete callback: the ARR value of the LAST interval has just
 * been loaded (this runs right after the update event that starts it).
 * Disable the DMA request and arm the TIM3 update interrupt so the end of the
 * last interval fires SentApp_OnTim3UpdateIrq() for cleanup.
 *
 * CCR3 is preloaded (OC3PE): writing 0 here does not disturb the LOW pulse of
 * the interval in progress, but from the next update event on PWM1 compares
 * against 0 and keeps the output inactive.  Without this the output would go
 * active again at that update event until the cleanup ISR runs, putting a
 * runt LOW pulse of a few microseconds on the bus after every frame. */
static void tim3_tx_dma_complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    TIM3->DIER &= ~TIM_DMA_UPDATE;
    TIM3->CCR3  = 0U;
    TIM3->SR    = 0U;               /* clear any pending UIF before enabling IT */
    TIM3->DIER |= TIM_IT_UPDATE;
}

/* Start DMA-driven TX for the frame pre-loaded into g_tx_hal.
 * No-op if the timer is already running.
 *
 * Sequence:
 *   1. Pick the prescaler from the longest interval, then fold each
 *      [LOW_ticks, HIGH_ticks] pair into a single total ARR value (counts−1)
 *      written back to intervals[0..n_raw-1] in-place.
 *   2. URS+UG flush: load PSC, CCR3 and ARR=intervals[0] (sync) with OC3 =
 *      FORCED_ACTIVE.  URS=1 keeps UG from generating a UEV/DMA request.
 *   3. CR1 |= CEN — counter starts.  OC3 is FORCED_ACTIVE so OCxREF=1 → PB0
 *      HIGH → transistor ON → bus LOW.  This is the sync's active-LOW pulse.
 *   4. Spin until CNT >= CCR3 (5 ticks), then write OC3 = FORCED_INACTIVE
 *      (bus HIGH) to end the LOW pulse and switch to PWM1.  CNT is already past
 *      CCR3, so PWM1 keeps the output inactive for the rest of the sync.
 *      Interrupts are masked for these few ticks so that the LOW pulse cannot
 *      be stretched by an ISR.
 *   5. Arm DMA to transfer intervals[1..n_raw-1] → TIM3→ARR on update events,
 *      then enable TIM_DMA_UPDATE.
 *   6. At CNT=ARR (end of sync) PWM1 starts the status interval's LOW pulse
 *      and DMA loads intervals[1] into ARR — the rest of the frame runs DMA-only.
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
    /* Timer input cycles per SENT tick, times 10 (tick_x10 is in 0.1 µs). */
    uint32_t cycles_x10 = (TIM3_CLOCK_HZ / 1000000U) * (uint32_t)tick_x10;

    uint8_t low_ticks = g_tx_hal.config.low_ticks;
    if (low_ticks == 0U) { low_ticks = 5U; }

    /* The 16-bit ARR must hold the longest interval of this frame: 56 ticks of
     * sync exceed 65536 cycles above a 24 µs tick, and a 768-tick pause already
     * does at 3 µs.  Divide the timer clock just enough for it to fit. */
    uint32_t max_ticks = 0U;
    for (uint8_t i = 0U; i < n_raw; i++) {
        uint32_t total = (uint32_t)g_tx_hal.intervals[2U * i] +
                         (uint32_t)g_tx_hal.intervals[2U * i + 1U];
        if (total > max_ticks) { max_ticks = total; }
    }
    uint32_t div = ((max_ticks * cycles_x10) / 10U + 65535U) / 65536U;
    if (div == 0U) { div = 1U; }
    uint32_t den = 10U * div;                    /* cycles_x10 per timer count */

    /* Fold [LOW_ticks, HIGH_ticks, ...] pairs into total ARR values in-place.
     * intervals[i] = round((LOW[i] + HIGH[i]) × cycles_per_tick / div) − 1. */
    for (uint8_t i = 0U; i < n_raw; i++) {
        uint32_t total = (uint32_t)g_tx_hal.intervals[2U * i] +
                         (uint32_t)g_tx_hal.intervals[2U * i + 1U];
        uint32_t c = (total * cycles_x10 + den / 2U) / den;
        if (c < 2U)     { c = 2U; }
        if (c > 65536U) { c = 65536U; }
        g_tx_hal.intervals[i] = (uint16_t)(c - 1U);
    }
    /* LOW drive time in timer input cycles x10, shortened by the output
     * stage's switch-off delay (see TX_LOW_STRETCH_US). */
    uint32_t low_x10     = (uint32_t)low_ticks * cycles_x10;
    uint32_t stretch_x10 = TX_LOW_STRETCH_US * (TIM3_CLOCK_HZ / 1000000U) * 10U;
    uint32_t min_x10     = TX_LOW_MIN_US     * (TIM3_CLOCK_HZ / 1000000U) * 10U;
    low_x10 = (low_x10 > stretch_x10 + min_x10) ? (low_x10 - stretch_x10) : min_x10;
    uint32_t ccr3 = (low_x10 + den / 2U) / den;
    if (ccr3 == 0U) { ccr3 = 1U; }

    /* Flush stale TIM3 state from the previous frame.  URS=1 makes the upcoming
     * UG a register refresh only (no UEV → no DMA request); it loads the
     * prescaler and the preloaded CCR3.  Start in FORCED_ACTIVE so OCxREF=1 the
     * moment CEN goes 1 — this guarantees the sync's LOW pulse on the bus. */
    TIM3->CR1   |= TIM_CR1_URS;
    TIM3->PSC    = (uint16_t)(div - 1U);
    TIM3->CCR3   = (uint16_t)ccr3;
    TIM3->ARR    = g_tx_hal.intervals[0];
    TIM3->CCMR2  = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_FORCED_ACTIVE;
    TIM3->EGR    = TIM_EGR_UG;
    TIM3->SR     = 0U;
    TIM3->CR1   &= ~TIM_CR1_URS;
    TIM3->CNT    = 0U;

    /* Start the counter while OC3 is FORCED_ACTIVE — bus drops LOW immediately
     * (start of the sync's active-LOW pulse).  Spin until CNT crosses CCR3,
     * then end the LOW pulse (FORCED_INACTIVE, immediate) and hand over to
     * PWM1, which sees CNT >= CCR3 and stays inactive until the next update. */
    __disable_irq();
    TIM3->CR1 |= TIM_CR1_CEN;
    while ((uint16_t)TIM3->CNT < (uint16_t)ccr3) { /* LOW drive time */ }
    TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_FORCED_INACTIVE;
    TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | TIM_OCMODE_PWM1;
    __enable_irq();

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

/* Parse and act on one complete SLCAN line (no trailing CR/LF).  Main loop only.
 * Responses are queued in the USB TX ring buffer. */
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
        resp = "V0102\r";
        break;
    case SENT_SLCAN_CMD_FWVERSION:
        /* uCCBViewer calls .substring(1) on this; must not be empty */
        resp = "v0102\r";
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
     * 6 data nibbles, MSB-first, DATA_ONLY CRC, J2716 recommended CRC (start
     * value 0x03).  The tick range is wide open — 2.0 to 113 µs covers the
     * standard's 3..90 µs nominal range with the ±25 % a receiver must accept —
     * because frames are found by their structure, not by a fixed threshold.
     * The host can narrow it with a 0x001 config frame or the LEARN command
     * (CAN 0x600, data[0]=0x04). */
    sent_config_t cfg;
    cfg.data_nibbles         = 6U;
    cfg.crc_mode             = SENT_CRC_MODE_DATA_ONLY;
    cfg.order                = SENT_NIBBLE_ORDER_MSB_FIRST;
    cfg.pause_pulse_enabled  = false;
    cfg.min_tick_x10_us      = SENT_BRIDGE_LEARN_MIN_TICK_X10;
    cfg.max_tick_x10_us      = SENT_BRIDGE_LEARN_MAX_TICK_X10;
    cfg.crc_init_seed        = 0x03U;
    cfg.crc_legacy           = false;

    /* ── RX HAL ──
     * TIM2 is a 32-bit timer free-running at 48 MHz.  PA2 = TIM2_CH3 (AF2),
     * FALLING edge capture: J2716 defines every period from falling edge to
     * falling edge.  The ISR only stores the raw counter value; alignment and
     * decoding run in the main loop at the full 20.8 ns resolution. */
    sent_stm32f042_rx_config_t rx_cfg = {
        .timer_clock_hz = 48000000U,
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
    g_usb_rx_head  = 0U;
    g_usb_rx_tail  = 0U;
    g_usb_tx_head  = 0U;
    g_usb_tx_tail  = 0U;
}

/* Called from TIM2 CH3 capture ISR (HAL_TIM_IC_CaptureCallback in main.c).
 * captured_counter is the raw 32-bit TIM2 value at the moment of the FALLING edge. */
void SentApp_OnSentRxCaptureEdge(uint32_t captured_counter)
{
    sent_stm32f042_rx_on_capture_edge_isr(&g_rx_hal, captured_counter);
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
 * Only stores the bytes; SentApp_Process() parses them in the main loop.
 * Bytes that do not fit are dropped, which makes the affected line fail to
 * parse and be answered with a NACK. */
void SentApp_OnUsbRx(const uint8_t *data, uint32_t len)
{
    uint8_t head = g_usb_rx_head;
    for (uint32_t i = 0U; i < len; i++) {
        uint8_t next = (uint8_t)((head + 1U) & (USB_RX_RING_SIZE - 1U));
        if (next == g_usb_rx_tail) { break; }   /* ring full */
        g_usb_rx[head] = data[i];
        head = next;
    }
    g_usb_rx_head = head;
}

/* Drain the USB RX ring: accumulate bytes into a line and dispatch each
 * complete SLCAN command. */
static void process_usb_rx(void)
{
    uint8_t head = g_usb_rx_head;   /* snapshot; ISR may add more meanwhile */
    while (g_usb_rx_tail != head) {
        char c = (char)g_usb_rx[g_usb_rx_tail];
        g_usb_rx_tail = (uint8_t)((g_usb_rx_tail + 1U) & (USB_RX_RING_SIZE - 1U));

        if (c == '\r' || c == '\n') {
            /* CR or LF terminates a SLCAN command line */
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

/* Emit a diagnostic CAN frame (ID 0x511, DLC=8) when a counter changed, at
 * most every DIAG_PERIOD_MS.
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
    uint32_t now = HAL_GetTick();
    if ((now - g_last_diag_ms) < DIAG_PERIOD_MS) {
        return;
    }
    g_last_diag_ms = now;

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
    usb_tx_push_frame(&diag);

    g_last_frames_decoded = frames;
    g_last_crc_errors     = crc;
    g_last_sync_errors    = sync;
}

/* Main-loop pump — must be called as frequently as possible.
 *
 * Commands: parses SLCAN lines received over USB and acts on them.
 *
 * RX path: drains the falling-edge timestamps captured by the TIM2 ISR and
 *   feeds them to the bridge, which aligns to frames, decodes them and returns
 *   CAN frames to forward to the host as SLCAN 't' lines.
 *
 * TX path: started from dispatch_slcan_line(), then runs on TIM3 + DMA.
 *
 * USB flush: attempts to send the next contiguous ring-buffer segment to the host. */
void SentApp_Process(void)
{
    process_usb_rx();

    if (g_bridge.rx_hal.poll_timestamps_us != NULL) {
        uint32_t edges[16];
        size_t count = sizeof(edges) / sizeof(edges[0]);
        while (g_bridge.rx_hal.poll_timestamps_us(g_bridge.rx_hal.context, edges, &count)) {
            for (size_t i = 0U; i < count; i++) {
                sent_can_frame_t out_frame;
                if (sent_bridge_on_edge(&g_bridge, edges[i], &out_frame)) {
                    usb_tx_push_frame(&out_frame);
                }
            }
            count = sizeof(edges) / sizeof(edges[0]);
        }

        /* Edges lost because this loop fell behind the capture ISR: the stream
         * has a hole, so drop alignment instead of decoding across it. */
        uint32_t dropped = sent_stm32f042_rx_dropped_edges(&g_rx_hal);
        if (dropped != g_last_dropped_edges) {
            g_last_dropped_edges = dropped;
            sent_bridge_on_rx_overrun(&g_bridge);
        }
    }

    emit_diag_if_changed();
    usb_tx_flush();
}
