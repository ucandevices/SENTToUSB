/*
 * sent_app_c071.c — STM32C071 platform layer of the 4-channel SENT-to-USB
 * converter.  Protocol, channels, USB rings and diagnostics are in the
 * MCU-independent sent_core.c; this file owns the timers and DMA.
 *
 * Pin plan (plan section 2.5):
 *   channel  RX (TIM2, 32-bit)    TX timer / pin        DMA (update request via DMAMUX)
 *   0        PA0 TIM2_CH1 AF3     TIM3_CH3  PB0 AF1     DMA1 ch1, DMAMUX ch0, TIM3_UP  (37)
 *   1        PA1 TIM2_CH2 AF3     TIM1_CH1  PA8 AF2     DMA1 ch2, DMAMUX ch1, TIM1_UP  (25)
 *   2        PA2 TIM2_CH3 AF6     TIM16_CH1 PA6 AF5     DMA1 ch3, DMAMUX ch2, TIM16_UP (46)
 *   3        PA3 TIM2_CH4 AF3     TIM17_CH1 PA7 AF5     DMA1 ch4, DMAMUX ch3, TIM17_UP (49)
 *
 * RX: TIM2 free-runs over 32 bits at 48 MHz; all four channels capture the
 * FALLING edge with the v1.1 input filter (ICF = 3, 8 samples at 48 MHz).
 * The TIM2 interrupt is handled at register level (no HAL_TIM_IRQHandler)
 * and runs from SRAM: read SR once, store each captured CCRx in its
 * channel's ring, count every over-capture flag (CCxOF: an edge the hardware
 * lost) as a lost edge, which the main loop turns into an alignment reset
 * for that channel.  Rev 2b (concept review M4): edge-rate guard, see
 * rx_edge(); a channel's capture interrupt is only enabled while the channel
 * is in RX or learn mode (off while it transmits: it would see its own edges).
 *
 * Rev 2b board (plan 2.6): each TX pin drives the line through a
 * non-inverting SN74LV1T125 buffer whose OE is TX_EN (active low).  The
 * output channels therefore run with inverted polarity (CCxP = 1): the PWM
 * active phase (the SENT LOW pulse) is a LOW pin, and forced-inactive (idle)
 * is a HIGH pin.  TX_EN is low only while the channel is in TX mode.
 * Termination (TERM_EN, active high) and TX_EN follow the line state that
 * sent_core reports through line_control().
 *
 * Sensor supply (concept review M3): VS_EN on only while USB is configured
 * and not suspended.  VS is measured on PB1 every 100 ms (ADC, 16x hardware
 * oversampling, scaled with VREFINT) and reported by sent_core as 0x60E.
 * Rev 2d: VS_EN only switches the USB path (an external supply on EXT_IN is
 * always passed through), so VS is reported as measured, never assumed 0.
 * Also in 0x60E: sensor-rail current (I_SENSE, PB2) and the TPS2121 ST
 * output (VS_SRC, PB9).
 *
 * TX: one timer per channel, PWM mode 1, ARR reloaded per SENT interval by
 * the timer's update DMA request — the v1.1 TIM3 scheme, generalised into a
 * per-channel table.  Kept from v1.1: per-frame prescaler (long ticks and
 * 768-tick pauses fit the 16-bit ARR) and the preloaded CCR = 0 at the end of
 * the frame (no runt pulse).  New: the frame starts with a short dummy
 * period instead of the interrupts-off busy-wait — see tx_start().
 */
#include "sent_app.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "stm32c0xx_ll_adc.h"   /* VREFINT_CAL_ADDR / VREFINT_CAL_VREF only */

#include "sent_core.h"
#include "sent/hal_stm32.h"
#include "sent/sent_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#if SENT_NUM_CHANNELS != 4
#error "sent_app_c071.c is the 4-channel STM32C071 platform layer"
#endif

#define TIMER_CLOCK_HZ 48000000U   /* SYSCLK = AHB = APB; timers run at 48 MHz */

/* Output-stage compensation (plan 3.4): LOW-pulse stretch of the TX output
 * stage.  v1.1 hardware (FET, 10 kΩ gate) needed 10 µs / 8 µs.  The rev 2b
 * push-pull buffer (SN74LV1T125, about 5 µs rise and fall) is symmetric, so
 * both are 0 for this board. */
#ifndef TX_LOW_STRETCH_US
#define TX_LOW_STRETCH_US 0U
#endif
#ifndef TX_LOW_MIN_US
#define TX_LOW_MIN_US 0U
#endif

/* Length of the dummy first period in timer input cycles (2 µs).  It must be
 * longer than the time the DMA needs to load the sync length into ARR after
 * the update event that ends it. */
#define TX_START_DUMMY_CYCLES 96U

/* OCxM / OCxPE live at the same bit positions for channel 1 (CCMR1) and
 * channel 3 (CCMR2), the only output channels used here. */
#define OCM_MASK            TIM_CCMR1_OC1M
#define OCM_FORCED_INACTIVE TIM_CCMR1_OC1M_2
#define OCM_PWM1            (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)
#define OC_PRELOAD          TIM_CCMR1_OC1PE

typedef struct {
    TIM_TypeDef* tim;
    volatile uint32_t* ccmr;       /* CCMR register of the output channel */
    volatile uint32_t* ccr;        /* CCR of the output channel */
    uint32_t ccer_enable;          /* CCxE bit */
    uint32_t ccer_polarity;        /* CCxP bit: active low (rev 2b buffer) */
    bool has_bdtr;                 /* TIM1/16/17: outputs need MOE */
    DMA_Channel_TypeDef* dma;
    uint8_t dma_number;            /* 1..4: flag position in DMA1->ISR / IFCR */
    DMAMUX_Channel_TypeDef* mux;
    uint8_t mux_request;
    IRQn_Type tim_irq;
    IRQn_Type dma_irq;
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t af;
} tx_hw_t;

static const tx_hw_t k_tx_hw[SENT_NUM_CHANNELS] = {
    {TIM3,  &TIM3->CCMR2,  &TIM3->CCR3,  TIM_CCER_CC3E, TIM_CCER_CC3P, false, DMA1_Channel1, 1U, DMAMUX1_Channel0,
     37U /* DMA_REQUEST_TIM3_UP */,  TIM3_IRQn,  DMA1_Channel1_IRQn,      GPIOB, GPIO_PIN_0, 1U},
    {TIM1,  &TIM1->CCMR1,  &TIM1->CCR1,  TIM_CCER_CC1E, TIM_CCER_CC1P, true,  DMA1_Channel2, 2U, DMAMUX1_Channel1,
     25U /* DMA_REQUEST_TIM1_UP */,  TIM1_BRK_UP_TRG_COM_IRQn, DMA1_Channel2_3_IRQn, GPIOA, GPIO_PIN_8, 2U},
    {TIM16, &TIM16->CCMR1, &TIM16->CCR1, TIM_CCER_CC1E, TIM_CCER_CC1P, true,  DMA1_Channel3, 3U, DMAMUX1_Channel2,
     46U /* DMA_REQUEST_TIM16_UP */, TIM16_IRQn, DMA1_Channel2_3_IRQn,    GPIOA, GPIO_PIN_6, 5U},
    {TIM17, &TIM17->CCMR1, &TIM17->CCR1, TIM_CCER_CC1E, TIM_CCER_CC1P, true,  DMA1_Channel4, 4U, DMAMUX1_Channel3,
     49U /* DMA_REQUEST_TIM17_UP */, TIM17_IRQn, DMAMUX1_DMA1_CH4_5_IRQn, GPIOA, GPIO_PIN_7, 5U},
};

static sent_stm32_rx_hal_t g_rx_hal[SENT_NUM_CHANNELS];
static sent_stm32_tx_hal_t g_tx_hal[SENT_NUM_CHANNELS];

/* ── Receive: TIM2 CH1..CH4 ─────────────────────────────────────────────── */

static void rx_timer_init(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    /* No internal pull-up: the board has its own 10 k pull-up to the 5 V
     * sensor supply, and DS14693 forbids the internal pull-up on FT pins
     * that see more than VDD (concept_review.md finding 2). */
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
    g.Alternate = 3U;                 /* TIM2_CH1, CH2, CH4 */
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_2;
    g.Alternate = 6U;                 /* TIM2_CH3 */
    HAL_GPIO_Init(GPIOA, &g);

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0U;
    TIM2->PSC = 0U;
    TIM2->ARR = 0xFFFFFFFFU;          /* free-running 32 bits, wraps every 89 s */
    /* CCxS = 01 (input, TIx), ICxF = 0011 (fCK_INT, N = 8): rejects glitches
     * shorter than about 167 ns, passes every SENT edge (v1.1 setting). */
    const uint32_t ic = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1;
    TIM2->CCMR1 = ic | (ic << 8U);
    TIM2->CCMR2 = ic | (ic << 8U);
    /* Falling edge (CCxP = 1, CCxNP = 0): J2716 measures falling to falling. */
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P |
                 TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC4E | TIM_CCER_CC4P;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    /* Capture interrupts stay off until a channel enters RX/learn
     * (platform_line_control -> rx_irq_update). */
    TIM2->DIER = 0U;
    TIM2->CR1 = TIM_CR1_CEN;

    /* Highest priority: the capture register already holds the timestamp, so
     * latency only matters for over-capture, but the handler is short. */
    NVIC_SetPriority(TIM2_IRQn, 0U);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* ── Capture interrupt with edge-rate guard (concept review M4) ─────────────
 *
 * Guard: SysTick clears g_rx_edges_ms[] every 1 ms.  A channel that delivers
 * more than RX_GUARD_EDGES_PER_MS edges inside one such window (a valid SENT
 * signal has at most about 28: 3 µs tick, 12-tick nibbles) gets its CCxIE
 * cleared for RX_GUARD_HOLD_MS, one guard trip counted (reported in
 * 0x512 + 0x40n, data[6..7]) and one lost edge noted, so the main loop
 * resynchronises the channel.  The other channels keep running.
 *
 * Flag handling: CCxIF is cleared by the hardware when CCRx is read, so it
 * is never written: a write-0 clear could wipe a capture that arrives
 * between the CCRx read and the write and lose it silently.  CCxOF is
 * cleared with a write-0 mask of exactly the OF flags seen (TIM2->SR =
 * ~of; writing 1 leaves the other rc_w0 bits alone), never read-modify-write.
 * Only channels whose CCxIE is set are serviced; the flags of a disabled
 * channel are discarded when it is enabled again (rx_irq_update).
 */
#define RX_GUARD_EDGES_PER_MS 100U
#define RX_GUARD_HOLD_MS      10U

#define TIM2_CC_IF_MASK (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF)
#define TIM2_CC_OF_MASK (TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF)
#define TIM2_CC_IE_MASK (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE)
#if (TIM_SR_CC1IF != TIM_DIER_CC1IE) || (TIM_SR_CC1OF != (TIM_SR_CC1IF << 8U))
#error "TIM2 SR/DIER bit layout assumption broken"
#endif

/* Code that must run from SRAM (copied by the startup code with .data; see
 * the .RamFunc input section in STM32C071K8TX_FLASH.ld). */
#define RAMFUNC __attribute__((section(".RamFunc"), noinline))
#define ALWAYS_INLINE static inline __attribute__((always_inline))

static volatile uint8_t  g_rx_edges_ms[SENT_NUM_CHANNELS];   /* edges in the current 1 ms */
static volatile uint8_t  g_rx_guard_ms[SENT_NUM_CHANNELS];   /* hold time left; 0 = not held */
static volatile uint32_t g_rx_guard_trips[SENT_NUM_CHANNELS];
static volatile uint8_t  g_rx_listen;                        /* bit n: channel n in RX/learn */

ALWAYS_INLINE void rx_lost_isr(uint8_t ch)
{
    sent_stm32_rx_hal_t* hal = &g_rx_hal[ch];
    if (hal->running) {
        hal->dropped_edges++;
    }
}

ALWAYS_INLINE void rx_edge_isr(uint8_t ch, uint32_t captured)
{
    uint8_t n = (uint8_t)(g_rx_edges_ms[ch] + 1U);
    if (n > RX_GUARD_EDGES_PER_MS) {
        TIM2->DIER &= ~(TIM_DIER_CC1IE << ch);   /* priority 0: nothing preempts this */
        g_rx_guard_ms[ch] = RX_GUARD_HOLD_MS;
        g_rx_guard_trips[ch]++;
        rx_lost_isr(ch);
        return;
    }
    g_rx_edges_ms[ch] = n;

    /* sent_stm32_rx_on_capture_edge_isr(), written out so that nothing in
     * this handler calls code in flash. */
    sent_stm32_rx_hal_t* hal = &g_rx_hal[ch];
    if (!hal->running) {
        return;
    }
    uint8_t head = hal->head;
    uint8_t next = (uint8_t)((head + 1U) & (SENT_STM32_RX_RING_SIZE - 1U));
    if (next == hal->tail) {
        hal->dropped_edges++;
        return;
    }
    hal->edges[head] = captured;
    hal->head = next;
}

RAMFUNC void TIM2_IRQHandler(void)
{
    uint32_t ie = TIM2->DIER & TIM2_CC_IE_MASK;
    uint32_t sr = TIM2->SR;
    uint32_t cc = sr & ie;                        /* captures of enabled channels */
    uint32_t of = sr & (ie << 8U);                /* over-captures of enabled channels */

    /* CCRx first (clears CCxIF), then the OF flags that were seen. */
    if (cc & TIM_SR_CC1IF) { rx_edge_isr(0U, TIM2->CCR1); }
    if (cc & TIM_SR_CC2IF) { rx_edge_isr(1U, TIM2->CCR2); }
    if (cc & TIM_SR_CC3IF) { rx_edge_isr(2U, TIM2->CCR3); }
    if (cc & TIM_SR_CC4IF) { rx_edge_isr(3U, TIM2->CCR4); }
    if (of != 0U) {
        TIM2->SR = ~of;
        if (of & TIM_SR_CC1OF) { rx_lost_isr(0U); }
        if (of & TIM_SR_CC2OF) { rx_lost_isr(1U); }
        if (of & TIM_SR_CC3OF) { rx_lost_isr(2U); }
        if (of & TIM_SR_CC4OF) { rx_lost_isr(3U); }
    }
}

/* Set channel ch's CCxIE from its mode and guard state.  Any context; the
 * read-modify-write of DIER runs with interrupts off.  When the interrupt is
 * (re-)enabled, the stale capture and over-capture flags are discarded so the
 * first edge handed to the ring is a fresh one. */
static void rx_irq_update(uint8_t ch)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t ie = TIM_DIER_CC1IE << ch;
    bool want = ((g_rx_listen & (1U << ch)) != 0U) && (g_rx_guard_ms[ch] == 0U);
    if (want) {
        if ((TIM2->DIER & ie) == 0U) {
            (void)(&TIM2->CCR1)[ch];             /* clears CCxIF */
            TIM2->SR = ~(TIM_SR_CC1OF << ch);
            g_rx_edges_ms[ch] = 0U;
            TIM2->DIER |= ie;
        }
    } else {
        TIM2->DIER &= ~ie;
    }
    __set_PRIMASK(primask);
}

/* SysTick, every 1 ms: new guard window; release channels whose hold ended. */
void SentApp_OnTick1ms(void)
{
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        g_rx_edges_ms[ch] = 0U;
        uint8_t hold = g_rx_guard_ms[ch];
        if (hold != 0U) {
            g_rx_guard_ms[ch] = (uint8_t)(hold - 1U);
            if (hold == 1U) {
                rx_irq_update(ch);
            }
        }
    }
}

/* ── Transmit: one timer + update DMA per channel ───────────────────────── */

static void tx_timers_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();      /* also clocks DMAMUX1 */

    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        const tx_hw_t* hw = &k_tx_hw[ch];
        TIM_TypeDef* tim = hw->tim;

        /* ARPE = 0: an ARR written by DMA at an update event applies to the
         * period that event just started.  Output parked forced-inactive,
         * which with CCxP = 1 is a HIGH pin (buffer input high → line idle
         * HIGH once TX_EN enables the buffer); the PWM active phase is a LOW
         * pin = the SENT LOW pulse.  CCxNP/CCxNE (TIM1/16/17) stay 0.  CCR
         * preloaded so it can be changed during a frame and take effect at
         * the next update. */
        tim->CR1 = 0U;
        tim->PSC = 0U;
        tim->ARR = 0xFFFFU;
        *hw->ccmr = (*hw->ccmr & ~(OCM_MASK | OC_PRELOAD | TIM_CCMR1_CC1S)) |
                    OCM_FORCED_INACTIVE | OC_PRELOAD;
        *hw->ccr = 0U;
        tim->CCER |= hw->ccer_enable | hw->ccer_polarity;
        if (hw->has_bdtr) {
            tim->BDTR |= TIM_BDTR_MOE;   /* TIM1/TIM16/TIM17 outputs stay off without MOE */
        }
        tim->EGR = TIM_EGR_UG;
        tim->SR = 0U;
        NVIC_SetPriority(hw->tim_irq, 1U);
        NVIC_EnableIRQ(hw->tim_irq);

        /* DMA: memory (16-bit intervals, incrementing) → timer ARR (16-bit),
         * one transfer per update request, high priority, TC interrupt. */
        hw->dma->CCR = 0U;
        hw->dma->CPAR = (uint32_t)&tim->ARR;
        hw->dma->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                       DMA_CCR_PL_1 | DMA_CCR_TCIE;
        hw->mux->CCR = hw->mux_request;
        NVIC_SetPriority(hw->dma_irq, 1U);
        NVIC_EnableIRQ(hw->dma_irq);

        /* Pin last, once the output is defined (inactive). */
        GPIO_InitTypeDef g = {0};
        g.Pin = hw->pin;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = hw->af;
        HAL_GPIO_Init(hw->port, &g);
    }
}

/* Start the frame queued in g_tx_hal[ch] (no-op if the timer is running).
 *
 * The TX HAL holds [LOW_ticks, HIGH_ticks] pairs; they are folded in place
 * into one ARR value per SENT interval (v1.1 scheme), with a prescaler chosen
 * so the longest interval fits 16 bits.
 *
 * Frame start without busy-wait: the timer is started on a short dummy
 * period with CCR = 0 (output stays inactive).  The CCR for the frame is
 * already in the preload register, and the DMA holds intervals[0..n-1].  At
 * the update event that ends the dummy period the hardware loads CCR, PWM1
 * asserts the output (bus LOW = start of the sync pulse) and the DMA writes
 * the sync length into ARR — exactly what happens at the start of every other
 * interval.  (v1.1 needed FORCED_ACTIVE + a spin with interrupts off because
 * PWM1 does not assert the output at a counter start without an update
 * event.)  The dummy period only delays the frame by about 2 µs.
 *
 * End of frame (as v1.1): the DMA transfer-complete interrupt follows the
 * update event that starts the last interval (the pause); it writes CCR = 0
 * into the preload register, so no new LOW pulse starts at the next update,
 * and arms the update interrupt, which stops the timer. */
static void tx_start(uint8_t ch)
{
    const tx_hw_t* hw = &k_tx_hw[ch];
    TIM_TypeDef* tim = hw->tim;
    sent_stm32_tx_hal_t* tx = &g_tx_hal[ch];

    if (tim->CR1 & TIM_CR1_CEN) { return; }      /* frame in progress */

    uint8_t n = tx->count;
    if (n == 0U || (n & 1U)) { return; }        /* LOW+HIGH pairs */
    uint8_t n_raw = n >> 1U;                     /* SENT intervals in this frame */

    uint16_t tick_x10 = sent_stm32_tx_get_tick_x10_us(tx);
    if (tick_x10 == 0U) { tick_x10 = 30U; }
    uint32_t cycles_x10 = (TIMER_CLOCK_HZ / 1000000U) * (uint32_t)tick_x10;

    uint8_t low_ticks = tx->config.low_ticks;
    if (low_ticks == 0U) { low_ticks = 5U; }

    /* Per-frame prescaler: the longest interval must fit the 16-bit ARR. */
    uint32_t max_ticks = 0U;
    for (uint8_t i = 0U; i < n_raw; i++) {
        uint32_t total = (uint32_t)tx->intervals[2U * i] + (uint32_t)tx->intervals[2U * i + 1U];
        if (total > max_ticks) { max_ticks = total; }
    }
    uint32_t div = ((max_ticks * cycles_x10) / 10U + 65535U) / 65536U;
    if (div == 0U) { div = 1U; }
    uint32_t den = 10U * div;

    for (uint8_t i = 0U; i < n_raw; i++) {
        uint32_t total = (uint32_t)tx->intervals[2U * i] + (uint32_t)tx->intervals[2U * i + 1U];
        uint32_t c = (total * cycles_x10 + den / 2U) / den;
        if (c < 2U)     { c = 2U; }
        if (c > 65536U) { c = 65536U; }
        tx->intervals[i] = (uint16_t)(c - 1U);
    }

    uint32_t low_x10     = (uint32_t)low_ticks * cycles_x10;
    uint32_t stretch_x10 = TX_LOW_STRETCH_US * (TIMER_CLOCK_HZ / 1000000U) * 10U;
    uint32_t min_x10     = TX_LOW_MIN_US     * (TIMER_CLOCK_HZ / 1000000U) * 10U;
    low_x10 = (low_x10 > stretch_x10 + min_x10) ? (low_x10 - stretch_x10) : min_x10;
    uint32_t ccr = (low_x10 + den / 2U) / den;
    if (ccr == 0U) { ccr = 1U; }

    uint32_t dummy = (TX_START_DUMMY_CYCLES + div - 1U) / div;
    if (dummy < 2U) { dummy = 2U; }

    /* 1. Stopped timer: prescaler, dummy period, CCR shadow = 0, then PWM1.
     *    URS keeps the UG from raising a DMA request.  The UG that loads
     *    CCR shadow = 0 must come before the switch to PWM1: the shadow may
     *    still hold a non-zero CCR (1 after init, the frame value after an
     *    abort), and with CNT = 0 < CCR PWM1 would drive the pin active (bus
     *    LOW) for the few cycles until the UG. */
    tim->CR1 |= TIM_CR1_URS;
    tim->PSC = (uint16_t)(div - 1U);
    tim->ARR = (uint16_t)(dummy - 1U);
    *hw->ccr = 0U;
    tim->EGR = TIM_EGR_UG;                       /* loads PSC and CCR shadow, CNT = 0 */
    *hw->ccmr = (*hw->ccmr & ~OCM_MASK) | OCM_PWM1;   /* CNT 0 < CCR 0 is false: stays inactive */
    tim->SR = 0U;
    tim->CR1 &= ~TIM_CR1_URS;

    /* 2. LOW-pulse length for the frame, applied at the first update event. */
    *hw->ccr = ccr;

    /* 3. DMA: every update event loads the next interval into ARR. */
    hw->dma->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF1 << (4U * (hw->dma_number - 1U));
    hw->dma->CNDTR = n_raw;
    hw->dma->CMAR = (uint32_t)&tx->intervals[0];
    hw->dma->CCR |= DMA_CCR_EN;
    tim->DIER |= TIM_DIER_UDE;

    /* 4. Go. */
    tim->CR1 |= TIM_CR1_CEN;
}

/* DMA transfer complete for channel ch: the last ARR (pause) was just loaded. */
static void tx_dma_complete(uint8_t ch)
{
    const tx_hw_t* hw = &k_tx_hw[ch];
    DMA1->IFCR = DMA_IFCR_CGIF1 << (4U * (hw->dma_number - 1U));
    hw->dma->CCR &= ~DMA_CCR_EN;
    hw->tim->DIER &= ~TIM_DIER_UDE;
    *hw->ccr = 0U;                    /* preloaded: no LOW pulse after the pause */
    hw->tim->SR = ~TIM_SR_UIF;
    hw->tim->DIER |= TIM_DIER_UIE;
}

/* Update interrupt at the end of the pause: stop and park the output. */
static void tx_end_of_frame(uint8_t ch)
{
    const tx_hw_t* hw = &k_tx_hw[ch];
    TIM_TypeDef* tim = hw->tim;
    tim->SR = ~TIM_SR_UIF;
    if ((tim->DIER & TIM_DIER_UIE) == 0U) {
        return;
    }
    tim->CR1 &= ~TIM_CR1_CEN;
    tim->DIER &= ~TIM_DIER_UIE;
    *hw->ccmr = (*hw->ccmr & ~OCM_MASK) | OCM_FORCED_INACTIVE;
    g_tx_hal[ch].count = 0U;          /* release the TX HAL for the next frame */
    g_tx_hal[ch].active_index = 0U;
}

/* Stop a frame in progress (the channel left TX mode): timer, DMA and
 * interrupts off, output parked forced-inactive (pin HIGH), TX HAL released.
 * The partly sent frame ends at whatever interval it had reached. */
static void tx_abort(uint8_t ch)
{
    const tx_hw_t* hw = &k_tx_hw[ch];
    TIM_TypeDef* tim = hw->tim;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tim->CR1 &= ~TIM_CR1_CEN;
    tim->DIER &= ~(TIM_DIER_UDE | TIM_DIER_UIE);
    *hw->ccmr = (*hw->ccmr & ~OCM_MASK) | OCM_FORCED_INACTIVE;
    hw->dma->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF1 << (4U * (hw->dma_number - 1U));
    tim->SR = ~TIM_SR_UIF;
    NVIC_ClearPendingIRQ(hw->tim_irq);
    g_tx_hal[ch].count = 0U;
    g_tx_hal[ch].active_index = 0U;
    __set_PRIMASK(primask);
}

static bool dma_tc_pending(uint8_t dma_number)
{
    return (DMA1->ISR & (DMA_ISR_TCIF1 << (4U * (dma_number - 1U)))) != 0U;
}

void TIM3_IRQHandler(void)                 { tx_end_of_frame(0U); }
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)  { tx_end_of_frame(1U); }
void TIM16_IRQHandler(void)                { tx_end_of_frame(2U); }
void TIM17_IRQHandler(void)                { tx_end_of_frame(3U); }

void DMA1_Channel1_IRQHandler(void)
{
    if (dma_tc_pending(1U)) { tx_dma_complete(0U); }
}

void DMA1_Channel2_3_IRQHandler(void)
{
    if (dma_tc_pending(2U)) { tx_dma_complete(1U); }
    if (dma_tc_pending(3U)) { tx_dma_complete(2U); }
}

void DMAMUX1_DMA1_CH4_5_IRQHandler(void)
{
    if (dma_tc_pending(4U)) { tx_dma_complete(3U); }
}

/* ── Platform hooks for sent_core ───────────────────────────────────────── */

static uint32_t platform_now_ms(void)
{
    return HAL_GetTick();
}

static bool platform_usb_transmit(const uint8_t* data, uint16_t len)
{
    return CDC_Transmit_FS((uint8_t*)data, len) == USBD_OK;
}

static uint32_t platform_rx_lost_edges(uint8_t channel)
{
    return sent_stm32_rx_dropped_edges(&g_rx_hal[channel]);
}

static void platform_tx_kick(uint8_t channel)
{
    if (sent_stm32_tx_pending_frames(&g_tx_hal[channel]) > 0U) {
        tx_start(channel);
    }
}

/* Line state from sent_core (main loop): TX_EN, TERM_EN, capture interrupt.
 * Entering TX: capture interrupt off, termination, then the buffer on.
 * Leaving TX: frame stopped, buffer off, termination, capture interrupt on
 * (RX/learn only).  The TX output is forced-inactive (HIGH) whenever no
 * frame runs, so enabling the buffer never pulls the line low. */
static void platform_line_control(uint8_t ch, sent_bridge_mode_t mode, bool termination)
{
    static GPIO_TypeDef* const k_term_port[SENT_NUM_CHANNELS] = {
        TERM_EN1_GPIO_Port, TERM_EN2_GPIO_Port, TERM_EN34_GPIO_Port, TERM_EN34_GPIO_Port};
    static const uint16_t k_term_pin[SENT_NUM_CHANNELS] = {
        TERM_EN1_Pin, TERM_EN2_Pin, TERM_EN3_Pin, TERM_EN4_Pin};

    bool tx = (mode == SENT_BRIDGE_MODE_TX);
    bool listen = (mode == SENT_BRIDGE_MODE_RX) || (mode == SENT_BRIDGE_MODE_LEARN);

    if (!tx) {
        tx_abort(ch);
        TX_EN_GPIO_Port->BSRR = TX_EN_Pin(ch);           /* buffer off (high) */
    }
    if (listen) {
        g_rx_listen = (uint8_t)(g_rx_listen | (1U << ch));
    } else {
        g_rx_listen = (uint8_t)(g_rx_listen & ~(1U << ch));
        rx_irq_update(ch);                               /* off before the buffer drives */
    }

    if (termination) {
        k_term_port[ch]->BSRR = k_term_pin[ch];
    } else {
        k_term_port[ch]->BRR = k_term_pin[ch];
    }

    if (tx) {
        TX_EN_GPIO_Port->BRR = TX_EN_Pin(ch);            /* buffer on (low) */
    }
    if (listen) {
        rx_irq_update(ch);
    }
}

static uint32_t platform_rx_guard_trips(uint8_t channel)
{
    return g_rx_guard_trips[channel];
}

/* ── Sensor supply: VS_EN from the USB state, VS measurement (M3) ────────── */

static volatile bool g_usb_configured;
static volatile bool g_usb_suspended;

/* Interrupt or main context; BSRR/BRR writes are atomic. */
static void vs_apply(void)
{
    if (g_usb_configured && !g_usb_suspended) {
        VS_EN_GPIO_Port->BSRR = VS_EN_Pin;
    } else {
        VS_EN_GPIO_Port->BRR = VS_EN_Pin;
    }
}

/* CDC init = SET_CONFIGURATION; CDC de-init = configuration cleared, USB
 * reset or disconnect (usbd_cdc_if.c, usbd_conf.c). */
void SentApp_OnUsbConfigured(bool configured)
{
    g_usb_configured = configured;
    if (!configured) {
        g_usb_suspended = false;
    }
    vs_apply();
}

void SentApp_OnUsbSuspend(bool suspended)
{
    g_usb_suspended = suspended;
    vs_apply();
}

/* ADC: PB1 = VS x 22/122 (rev 2d: VS can be up to ~17 V from EXT_IN, so the
 * bottom resistor went 47 k -> 22 k).  Channel number from DS14693 (pin table: PB1 =
 * ADC1_IN18); the CubeC0 headers define channel 18 but contain no pin
 * mapping, so this is not cross-checked in software.  VREFINT = internal
 * channel 10 (LL_ADC_CHANNEL_VREFINT).  Rev 2d: PB2 = I_SENSE = ADC1_IN19:
 * 0.1 ohm x INA180A2 (50 V/V) = 5 mV per mA, through 1 k / 100 nF.  PB1 = IN18
 * and PB2 = IN19 also match the CubeMX 6.15 pin database (STM32C071K8Tx). */
#define VS_ADC_CHANNEL      18U
#define VREFINT_ADC_CHANNEL 10U
#define IS_ADC_CHANNEL      19U
#define VS_DIV_NUM          122U   /* (100 k + 22 k) / 22 k, rev 2d */
#define VS_DIV_DEN          22U
/* I_SENSE scale in mV per mA = IS_SCALE_NUM / IS_SCALE_DEN.
 * Rev 2f: the INA180 inputs tap the power path ~5.6 mm (IN+) and ~9 mm + 2 vias
 * (IN-) away from the R47 pads, adding ~25 mOhm of copper to the 0.1 ohm shunt.
 * Effective 0.125 ohm x 50 V/V = 6.25 mV/mA (full scale ~0.53 A at 3.3 V).
 * Verify on the first board against a meter and adjust NUM/DEN if needed. */
#define IS_SCALE_NUM        25U
#define IS_SCALE_DEN        4U
#define VS_SAMPLE_PERIOD_MS 100U
#define ADC_TIMEOUT_MS      5U

/* g_adc_raw[] index = position in the scan; the scan runs in ascending
 * channel order (10, 18, 19). */
#define ADC_IDX_VREFINT     0U
#define ADC_IDX_VS          1U
#define ADC_IDX_IS          2U
#define ADC_NUM_CONV        3U

typedef enum { ADC_OFF, ADC_IDLE, ADC_BUSY } adc_state_t;

static adc_state_t g_adc_state;
static uint32_t    g_adc_start_ms;
static uint8_t     g_adc_index;
static uint16_t    g_adc_raw[ADC_NUM_CONV];
static uint16_t    g_vs_mv;
static uint16_t    g_is_ma;
static bool        g_vs_valid;

static void spin_us(uint32_t us)
{
    for (volatile uint32_t i = 0U; i < us * (TIMER_CLOCK_HZ / 4000000U); i++) {
    }
}

static bool adc_wait(volatile uint32_t* reg, uint32_t mask, bool set)
{
    for (uint32_t i = 0U; i < 100000U; i++) {
        if (((*reg & mask) != 0U) == set) {
            return true;
        }
    }
    return false;
}

/* ADC clock PCLK/4 = 12 MHz, 160.5-cycle sampling (13.4 us; VREFINT needs
 * at least 4 us, the VS divider and I_SENSE are buffered by 100 nF), 16x
 * hardware oversampling shifted back to 12 bits, WAIT mode so a result is
 * never overwritten before the main loop reads it.  One scan (VREFINT, VS,
 * I_SENSE) takes about 0.7 ms and runs in the background. */
static void adc_init(void)
{
    g_adc_state = ADC_OFF;
    __HAL_RCC_ADC_CLK_ENABLE();
    ADC1->CR = 0U;
    ADC1->CFGR2 = ADC_CFGR2_CKMODE_1 |                          /* PCLK/4 */
                  ADC_CFGR2_OVSE | ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_0 | ADC_CFGR2_OVSS_2;
    ADC1->CR = ADC_CR_ADVREGEN;
    spin_us(20U);                                               /* tADCVREG_STUP */
    ADC1->CR |= ADC_CR_ADCAL;
    if (!adc_wait(&ADC1->CR, ADC_CR_ADCAL, false)) {
        return;
    }
    ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
    spin_us(12U);                                               /* VREFINT start-up */
    ADC1->CFGR1 = ADC_CFGR1_WAIT;                               /* 12 bit, right, single scan, up */
    ADC1->SMPR = ADC_SMPR_SMP1_2 | ADC_SMPR_SMP1_1 | ADC_SMPR_SMP1_0;
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    if (!adc_wait(&ADC1->ISR, ADC_ISR_ADRDY, true)) {
        return;
    }
    ADC1->CHSELR = (1UL << VREFINT_ADC_CHANNEL) | (1UL << VS_ADC_CHANNEL) |
                   (1UL << IS_ADC_CHANNEL);
    if (!adc_wait(&ADC1->ISR, ADC_ISR_CCRDY, true)) {
        return;
    }
    ADC1->ISR = ADC_ISR_CCRDY;
    g_adc_state = ADC_IDLE;
}

/* VDDA from VREFINT (factory value taken at 3.0 V). */
static uint32_t vdda_from_raw(uint32_t vref_raw)
{
    uint32_t cal = *VREFINT_CAL_ADDR;
    uint32_t vdda_mv = 3300U;
    if (cal != 0U && cal != 0xFFFFU && vref_raw != 0U) {
        vdda_mv = (VREFINT_CAL_VREF * cal + vref_raw / 2U) / vref_raw;
        if (vdda_mv < 2000U || vdda_mv > 3700U) {
            vdda_mv = 3300U;   /* implausible: assume the nominal 3.3 V */
        }
    }
    return vdda_mv;
}

/* VS through the divider. */
static uint16_t vs_from_raw(uint32_t vdda_mv, uint32_t vs_raw)
{
    /* vs_raw * vdda_mv * 122 <= 4095 * 3700 * 122 < 2^32 */
    const uint32_t den = 4095U * VS_DIV_DEN;
    return (uint16_t)((vs_raw * vdda_mv * VS_DIV_NUM + den / 2U) / den);
}

/* Sensor-rail current: V(PB2) / (IS_SCALE_NUM / IS_SCALE_DEN) mV per mA.
 * is_raw * vdda_mv * DEN <= 4095 * 3700 * 4, fits in 32 bits. */
static uint16_t is_from_raw(uint32_t vdda_mv, uint32_t is_raw)
{
    const uint32_t den = 4095U * IS_SCALE_NUM;
    return (uint16_t)((is_raw * vdda_mv * IS_SCALE_DEN + den / 2U) / den);
}

/* Main loop: start a scan every VS_SAMPLE_PERIOD_MS, collect it without
 * blocking. */
static void adc_poll(void)
{
    uint32_t now = HAL_GetTick();
    switch (g_adc_state) {
    case ADC_IDLE:
        if ((now - g_adc_start_ms) >= VS_SAMPLE_PERIOD_MS) {
            g_adc_start_ms = now;
            g_adc_index = 0U;
            ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
            ADC1->CR |= ADC_CR_ADSTART;
            g_adc_state = ADC_BUSY;
        }
        break;
    case ADC_BUSY:
        if ((ADC1->ISR & ADC_ISR_EOC) != 0U) {
            g_adc_raw[g_adc_index++] = (uint16_t)ADC1->DR;     /* clears EOC */
            if (g_adc_index >= ADC_NUM_CONV) {
                uint32_t vdda_mv = vdda_from_raw(g_adc_raw[ADC_IDX_VREFINT]);
                g_vs_mv = vs_from_raw(vdda_mv, g_adc_raw[ADC_IDX_VS]);
                g_is_ma = is_from_raw(vdda_mv, g_adc_raw[ADC_IDX_IS]);
                g_vs_valid = true;
                g_adc_state = ADC_IDLE;
            }
        } else if ((now - g_adc_start_ms) > ADC_TIMEOUT_MS) {
            ADC1->CR |= ADC_CR_ADSTP;                           /* lost: stop, retry later */
            (void)adc_wait(&ADC1->CR, ADC_CR_ADSTP, false);
            (void)ADC1->DR;
            g_adc_state = ADC_IDLE;
        }
        break;
    default:
        break;
    }
}

/* 0x60E source: only while USB is configured.  VS and current as measured
 * (an external supply may be on while VS_EN is off). */
static bool platform_supply_status(sent_core_supply_t* status)
{
    if (!g_usb_configured || !g_vs_valid) {
        return false;
    }
    uint8_t f = 0U;
    if ((VS_EN_GPIO_Port->ODR & VS_EN_Pin) != 0U) {
        f |= SENT_CORE_SUPPLY_EN;
    }
    if ((VS_FLT_GPIO_Port->IDR & VS_FLT_Pin) == 0U) {
        f |= SENT_CORE_SUPPLY_FAULT;
    }
    status->vs_mv = g_vs_mv;
    status->flags = f;
    status->is_ma = g_is_ma;
    status->source = ((VS_SRC_GPIO_Port->IDR & VS_SRC_Pin) != 0U) ? SENT_CORE_SUPPLY_SRC_EXT
                                                                  : SENT_CORE_SUPPLY_SRC_USB;
    return true;
}

/* ── Status LED (PB8, active high) ──────────────────────────────────────────
 *   off             USB not configured (no host / not enumerated / suspended)
 *   on              USB configured, idle
 *   short off-blink a SENT frame was decoded (at most every LED_ACT_PERIOD_MS)
 *   fast blink      load-switch fault (VS_FLT low: USB sensor supply in
 *                   current limit) */
#define LED_ACT_OFF_MS     40U
#define LED_ACT_PERIOD_MS  200U
#define LED_FAULT_HALF_MS  125U

static void led_poll(void)
{
    static uint32_t s_frames;
    static uint32_t s_blink_ms;
    static bool     s_blinking;

    uint32_t now = HAL_GetTick();
    bool on;

    uint32_t frames = 0U;
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        frames += sent_core_bridge(ch)->mode_manager.stats.frames_decoded;
    }

    if (!g_usb_configured || g_usb_suspended) {
        on = false;
    } else if ((VS_EN_GPIO_Port->ODR & VS_EN_Pin) != 0U &&
               (VS_FLT_GPIO_Port->IDR & VS_FLT_Pin) == 0U) {
        on = ((now / LED_FAULT_HALF_MS) & 1U) != 0U;
    } else {
        if (frames != s_frames && !s_blinking && (now - s_blink_ms) >= LED_ACT_PERIOD_MS) {
            s_blinking = true;
            s_blink_ms = now;
        }
        if (s_blinking && (now - s_blink_ms) >= LED_ACT_OFF_MS) {
            s_blinking = false;
        }
        on = !s_blinking;
    }
    s_frames = frames;

    if (on) {
        LED_GPIO_Port->BSRR = LED_Pin;
    } else {
        LED_GPIO_Port->BRR = LED_Pin;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void SentApp_Init(void)
{
    /* Same power-on configuration as v1.1, on every channel: 6 data nibbles,
     * data-only CRC with the recommended start value, tick 2.0..113 µs. */
    sent_config_t cfg;
    cfg.data_nibbles        = 6U;
    cfg.crc_mode            = SENT_CRC_MODE_DATA_ONLY;
    cfg.order               = SENT_NIBBLE_ORDER_MSB_FIRST;
    cfg.pause_pulse_enabled = false;
    cfg.min_tick_x10_us     = SENT_BRIDGE_LEARN_MIN_TICK_X10;
    cfg.max_tick_x10_us     = SENT_BRIDGE_LEARN_MAX_TICK_X10;
    cfg.crc_init_seed       = 0x03U;
    cfg.crc_legacy          = false;

    const sent_stm32_rx_config_t rx_cfg = { .timer_clock_hz = TIMER_CLOCK_HZ };
    const sent_stm32_tx_config_t tx_cfg = {
        .default_pause_ticks = 12U,
        .low_ticks           = 5U,
        .tx_tick_x10_us      = 30U,
    };
    sent_rx_hal_t rx_hal[SENT_NUM_CHANNELS];
    sent_tx_hal_t tx_hal[SENT_NUM_CHANNELS];
    for (uint8_t ch = 0U; ch < SENT_NUM_CHANNELS; ch++) {
        sent_stm32_rx_hal_init(&g_rx_hal[ch], &rx_cfg);
        sent_stm32_make_rx_hal(&g_rx_hal[ch], &rx_hal[ch]);
        sent_stm32_tx_hal_init(&g_tx_hal[ch], &tx_cfg);
        sent_stm32_make_tx_hal(&g_tx_hal[ch], &tx_hal[ch]);
    }

    /* 16-bit serial number for 'N': XOR-fold of the 96-bit unique ID. */
    const uint32_t uid0 = *(volatile const uint32_t*)(UID_BASE + 0x0U);
    const uint32_t uid1 = *(volatile const uint32_t*)(UID_BASE + 0x4U);
    const uint32_t uid2 = *(volatile const uint32_t*)(UID_BASE + 0x8U);
    const uint32_t h32  = uid0 ^ uid1 ^ uid2;

    /* Timers first: sent_core_init() reports every channel as OFF through
     * line_control(), which touches the TX timers and TIM2. */
    tx_timers_init();
    rx_timer_init();
    adc_init();

    static const sent_core_platform_t platform = {
        .now_ms         = platform_now_ms,
        .usb_transmit   = platform_usb_transmit,
        .rx_lost_edges  = platform_rx_lost_edges,
        .tx_kick        = platform_tx_kick,
        .line_control   = platform_line_control,
        .rx_guard_trips = platform_rx_guard_trips,
        .supply_status  = platform_supply_status,
        .features       = SENT_CORE_FEATURE_TERMINATION | SENT_CORE_FEATURE_SUPPLY_SENSE |
                          SENT_CORE_FEATURE_TX_BUFFER,
    };
    sent_core_init(&platform, &cfg, rx_hal, tx_hal, (uint16_t)(h32 ^ (h32 >> 16U)));
}

/* USB CDC receive callback (interrupt context): store bytes only. */
void SentApp_OnUsbRx(const uint8_t* data, uint32_t len)
{
    sent_core_on_usb_rx(data, len);
}

void SentApp_Process(void)
{
    adc_poll();
    sent_core_process();
    led_poll();
}
