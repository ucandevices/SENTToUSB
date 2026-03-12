#ifndef SENT_APP_H
#define SENT_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void SentApp_Init(void);
void SentApp_Process(void);

void SentApp_OnUsbRx(const uint8_t* data, uint32_t len);

void SentApp_OnSentRxCaptureEdge(uint16_t captured_counter);
void SentApp_OnSentRxTimerOverflow(void);
uint8_t SentApp_PopNextTxIntervalTicks(uint16_t* out_ticks);
uint8_t SentApp_HasPendingTxIntervals(void);

void SentApp_StartRxMode(void);
void SentApp_StartTxMode(void);
void SentApp_TestTxFrame(void);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_APP_H */
