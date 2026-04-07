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
 *   RX: host sends 'O' → device captures SENT frames from PA2 (TIM2 CH3)
 *                         and streams decoded frames back as SLCAN 't' lines.
 *   TX: host sends SLCAN 't' frame → device generates SENT signal on PA4 (TIM14).
 */

/* Call once after peripheral initialisation */
void SentApp_Init(void);

/* Call from the main loop — drains decoded RX frames and flushes USB TX */
void SentApp_Process(void);

/* Called from USB CDC receive callback (USB interrupt context) */
void SentApp_OnUsbRx(const uint8_t *data, uint32_t len);

/* Called from TIM2 CH3 input-capture ISR — records SENT rising-edge timestamp */
void SentApp_OnSentRxCaptureEdge(uint16_t captured_counter);

/* Called from TIM2 overflow ISR — extends the 16-bit capture counter */
void SentApp_OnSentRxTimerOverflow(void);

/* Called from TIM14 update ISR — drives the two-phase SENT TX pulse */
void SentApp_OnTim14UpdateIrq(void);

#ifdef __cplusplus
}
#endif

#endif /* SENT_APP_H */
