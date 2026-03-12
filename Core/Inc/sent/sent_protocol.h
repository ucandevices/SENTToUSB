#ifndef SENT_SENT_PROTOCOL_H
#define SENT_SENT_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_MAX_DATA_NIBBLES 8U
#define SENT_MAX_INTERVALS (SENT_MAX_DATA_NIBBLES + 4U)
#define SENT_MAX_TIMESTAMPS (SENT_MAX_INTERVALS + 1U)

typedef enum {
    SENT_CRC_MODE_DATA_ONLY = 0,
    SENT_CRC_MODE_STATUS_AND_DATA = 1,
} sent_crc_mode_t;

typedef enum {
    SENT_NIBBLE_ORDER_MSB_FIRST = 0,
    SENT_NIBBLE_ORDER_LSB_FIRST = 1,
} sent_nibble_order_t;

typedef struct {
    uint8_t data_nibbles;
    sent_crc_mode_t crc_mode;
    sent_nibble_order_t order;
    bool pause_pulse_enabled;
    uint16_t min_tick_x10_us;
    uint16_t max_tick_x10_us;
} sent_config_t;

typedef struct {
    uint8_t status;
    uint8_t data_nibbles[SENT_MAX_DATA_NIBBLES];
    uint8_t data_nibbles_count;
    uint8_t crc;
    uint16_t tick_x10_us;
    bool has_pause;
    uint16_t pause_ticks;
} sent_frame_t;

sent_config_t sent_default_config(void);
bool sent_validate_config(const sent_config_t* config);

uint32_t sent_pack_nibbles(const uint8_t* nibbles,
                           size_t nibble_count,
                           sent_nibble_order_t order);

bool sent_unpack_nibbles(uint32_t packed,
                         uint8_t nibble_count,
                         sent_nibble_order_t order,
                         uint8_t* out_nibbles);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_PROTOCOL_H */
