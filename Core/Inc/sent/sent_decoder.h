#ifndef SENT_SENT_DECODER_H
#define SENT_SENT_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENT_DECODE_OK = 0,
    SENT_DECODE_SYNC_ERROR = 1,
    SENT_DECODE_CRC_ERROR = 2,
    SENT_DECODE_SHAPE_ERROR = 3,
} sent_decode_status_t;

bool sent_decode_from_timestamps_us(const sent_config_t* config,
                                    const uint32_t* timestamps_us,
                                    size_t timestamp_count,
                                    sent_frame_t* out_frame,
                                    sent_decode_status_t* out_status);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_DECODER_H */
