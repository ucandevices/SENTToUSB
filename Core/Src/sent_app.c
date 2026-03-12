#include "sent_app.h"

#include "sent/sent_assert.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "usbd_cdc_if.h"

#include "sent/bridge.h"
#include "sent/hal_stm32f042.h"
#include "sent/slcan.h"

#define SENT_APP_USB_RX_RING_SIZE 128U
#define SENT_APP_USB_TX_RING_SIZE 128U
#define SENT_APP_SLCAN_LINE_MAX 64U

static uint16_t next_index(uint16_t index, uint16_t size) {
    return (uint16_t)((index + 1U) % size);
}

static sent_stm32f042_rx_hal_t g_rx_hal;
static sent_stm32f042_tx_hal_t g_tx_hal;
static sent_rx_hal_t g_rx_hal_if;
static sent_tx_hal_t g_tx_hal_if;
static sent_bridge_t g_bridge;

static uint8_t g_rx_ring[SENT_APP_USB_RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0U;
static volatile uint16_t g_rx_tail = 0U;

static uint8_t g_tx_ring[SENT_APP_USB_TX_RING_SIZE];
static uint16_t g_tx_head = 0U;
static uint16_t g_tx_tail = 0U;

static char g_line_buffer[SENT_APP_SLCAN_LINE_MAX];
static size_t g_line_len = 0U;

static bool rx_ring_push(uint8_t b) {
    uint16_t head = g_rx_head;
    uint16_t next = next_index(head, SENT_APP_USB_RX_RING_SIZE);
    if (next == g_rx_tail) {
        return false;
    }
    g_rx_ring[head] = b;
    g_rx_head = next;
    return true;
}

static bool rx_ring_pop(uint8_t* out) {
    SENT_ASSERT(out != NULL);

    uint16_t tail = g_rx_tail;
    if (tail == g_rx_head) {
        return false;
    }

    *out = g_rx_ring[tail];
    g_rx_tail = next_index(tail, SENT_APP_USB_RX_RING_SIZE);
    return true;
}

static bool tx_ring_push(uint8_t b) {
    uint16_t next = next_index(g_tx_head, SENT_APP_USB_TX_RING_SIZE);
    if (next == g_tx_tail) {
        return false;
    }
    g_tx_ring[g_tx_head] = b;
    g_tx_head = next;
    return true;
}

static void tx_enqueue_bytes(const uint8_t* data, size_t len) {
    SENT_ASSERT(data != NULL);

    for (size_t i = 0U; i < len; ++i) {
        if (!tx_ring_push(data[i])) {
            break;
        }
    }
}

static void tx_enqueue_cstr(const char* text) {
    SENT_ASSERT(text != NULL);
    tx_enqueue_bytes((const uint8_t*)text, strlen(text));
}

extern TIM_HandleTypeDef htim14;

static void kick_tx_timer(void) {
    if (sent_stm32f042_tx_pending_frames(&g_tx_hal) > 0U &&
        !(htim14.Instance->CR1 & TIM_CR1_CEN)) {
        SENT_TX_GPIO_Port->BSRR = SENT_TX_Pin;        /* ensure idle HIGH */
        __HAL_TIM_SET_COUNTER(&htim14, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim14, 1);         /* fire ISR ASAP */
        __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE); /* clear stale UIF */
        HAL_TIM_Base_Start_IT(&htim14);
    }
}

static void handle_slcan_line(const char* text) {
    sent_bridge_slcan_responses_t responses;
    if (!sent_bridge_on_slcan_line(&g_bridge, text, &responses)) {
        return;
    }

    for (uint8_t i = 0U; i < responses.count; ++i) {
        tx_enqueue_cstr(responses.lines[i]);
    }

    kick_tx_timer();
}

static void process_usb_rx(void) {
    uint8_t byte = 0U;
    while (rx_ring_pop(&byte)) {
        if (byte == '\r' || byte == '\n') {
            if (g_line_len > 0U) {
                g_line_buffer[g_line_len] = '\0';
                handle_slcan_line(g_line_buffer);
                g_line_len = 0U;
            }
            continue;
        }

        if (g_line_len >= (SENT_APP_SLCAN_LINE_MAX - 1U)) {
            g_line_len = 0U;
            continue;
        }

        g_line_buffer[g_line_len++] = (char)byte;
    }
}

static void process_sent_rx(void) {
    for (;;) {
        sent_can_frame_t frame;
        if (!sent_bridge_poll_rx_hal(&g_bridge, &frame)) {
            break;
        }

        char line[SENT_SLCAN_MAX_LINE_LEN + 2U];
        if (!sent_slcan_serialize_frame(&frame, line, sizeof(line))) {
            continue;
        }

        size_t len = strlen(line);
        line[len++] = '\r';
        line[len] = '\0';
        tx_enqueue_cstr(line);
    }
}

static void flush_usb_tx(void) {
    uint16_t tail = g_tx_tail;
    uint16_t head = g_tx_head;
    if (tail == head) {
        return;
    }

    /* Transmit the contiguous portion from tail towards end of buffer.
     * If data wraps around, the remainder is sent on the next call. */
    uint16_t count;
    if (head > tail) {
        count = head - tail;
    } else {
        count = SENT_APP_USB_TX_RING_SIZE - tail;
    }

    if (CDC_Transmit_FS(&g_tx_ring[tail], count) == USBD_OK) {
        tail += count;
        if (tail >= SENT_APP_USB_TX_RING_SIZE) {
            tail = 0U;
        }
        g_tx_tail = tail;
    }
}

void SentApp_Init(void) {
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_line_len = 0U;

    sent_stm32f042_rx_config_t rx_cfg;
    rx_cfg.timer_clock_hz = 48000000U;
    rx_cfg.timer_autoreload = 0xFFFFU;
    rx_cfg.capture_batch_size = SENT_STM32F042_RX_MAX_BATCH_SIZE;
    rx_cfg.ready_queue_depth = SENT_STM32F042_RX_MAX_READY_BATCHES;

    sent_stm32f042_tx_config_t tx_cfg;
    tx_cfg.max_pending_frames = 3U;
    tx_cfg.default_pause_ticks = 12U;

    sent_stm32f042_rx_hal_init(&g_rx_hal, &rx_cfg);
    sent_stm32f042_tx_hal_init(&g_tx_hal, &tx_cfg);
    sent_stm32f042_make_rx_hal(&g_rx_hal, &g_rx_hal_if);
    sent_stm32f042_make_tx_hal(&g_tx_hal, &g_tx_hal_if);

    sent_config_t cfg = sent_default_config();
    /* 04L906051B: 6 data nibbles, DATA_ONLY CRC, MSB-first, no pause pulse.
     * Lower min_tick bound to 2 us: the Q12 fixed-point converter at 48 MHz
     * underestimates by ~0.4%, so a 3 us-tick sensor's 168 us sync reads
     * back as 167 us (sync_x10=1670 < 56*30=1680 minimum). 20 gives ample
     * margin while still rejecting non-SENT noise. */
    cfg.min_tick_x10_us = 20U;
    sent_bridge_init(&g_bridge, &cfg, &g_rx_hal_if, &g_tx_hal_if);
}

void SentApp_Process(void) {
    process_usb_rx();
    process_sent_rx();
    flush_usb_tx();
}

void SentApp_OnUsbRx(const uint8_t* data, uint32_t len) {
    SENT_ASSERT(data != NULL);
    if (len == 0U) {
        return;
    }

    for (uint32_t i = 0U; i < len; ++i) {
        if (!rx_ring_push(data[i])) {
            break;
        }
    }
}

void SentApp_OnSentRxCaptureEdge(uint16_t captured_counter) {
    sent_stm32f042_rx_on_capture_edge_isr(&g_rx_hal, captured_counter);
}

void SentApp_OnSentRxTimerOverflow(void) {
    sent_stm32f042_rx_on_overflow_isr(&g_rx_hal);
}

uint8_t SentApp_PopNextTxIntervalTicks(uint16_t* out_ticks) {
    return sent_stm32f042_tx_pop_next_interval_ticks_from_isr(&g_tx_hal, out_ticks) ? 1U : 0U;
}

uint8_t SentApp_HasPendingTxIntervals(void) {
    return sent_stm32f042_tx_pending_frames(&g_tx_hal) > 0U ? 1U : 0U;
}

void SentApp_StartRxMode(void) {
    /* Start RX mode: CAN ID 0x600, DLC=1, data=[0x01] */
    handle_slcan_line("t600101");
}

void SentApp_StartTxMode(void) {
    /* Start TX mode: CAN ID 0x600, DLC=1, data=[0x02] */
    handle_slcan_line("t600102");
}

void SentApp_TestTxFrame(void) {
    /* Submit dummy SENT frame: CAN ID 0x520, DLC=5
     * data[0]=0x01 (status), data[1..4]=0x00ABCDEF (packed 6 nibbles) */
    handle_slcan_line("t52050100ABCDEF");
}
