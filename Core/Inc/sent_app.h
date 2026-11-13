#ifndef SENT_APP_H
#define SENT_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * sent_app.h — Public API for the SENT bridge application layer.
 *
 * The application wires USB CDC (SLCAN) to the SENT protocol:
 *   RX: host sends 'O' → device captures SENT falling edges on PA2 (TIM2 CH3)
 *                         and streams decoded frames back as SLCAN 't' lines.
 *   TX: host sends SLCAN 't' frame → device generates SENT signal on PB0 (TIM3 CH3).
 *       DMA1_Channel3 reloads TIM3→ARR on each update; a single TIM3 update ISR
 *       handles end-of-frame cleanup.
 */

/* Call once after peripheral initialisation */
void SentApp_Init(void);

/* Call from the main loop — parses host commands, decodes captured SENT edges
 * and flushes USB TX */
void SentApp_Process(void);

/* Called from USB CDC receive callback (USB interrupt context) — queues the bytes */
void SentApp_OnUsbRx(const uint8_t *data, uint32_t len);

/* Called from TIM2 CH3 input-capture ISR — records a SENT falling-edge timestamp
 * (raw value of the free-running 32-bit TIM2 counter) */
void SentApp_OnSentRxCaptureEdge(uint32_t captured_counter);

/* Called from TIM3 update ISR — end-of-frame cleanup after DMA TX completes */
void SentApp_OnTim3UpdateIrq(void);

#ifdef __cplusplus
}
#endif

#endif /* SENT_APP_H */
