#ifndef SENT_SENT_CRC_H
#define SENT_SENT_CRC_H

#include <stddef.h>
#include <stdint.h>

#include "sent/sent_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t sent_crc4_j2716(const uint8_t* data_nibbles,
                        size_t nibble_count,
                        sent_crc_mode_t mode,
                        uint8_t status_nibble);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_CRC_H */
