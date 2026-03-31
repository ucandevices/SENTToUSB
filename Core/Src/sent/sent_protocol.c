#include "sent/sent_protocol.h"

#include "sent/sent_assert.h"
#include <string.h>

/* Return default SENT configuration (6 data nibbles, CRC data-only, MSB-first, tick 3-90 us).
 * @return  sent_config_t with default field values */
sent_config_t sent_default_config(void) {
    sent_config_t cfg;
    cfg.data_nibbles = 6U;
    cfg.crc_mode = SENT_CRC_MODE_DATA_ONLY;
    cfg.order = SENT_NIBBLE_ORDER_MSB_FIRST;
    cfg.pause_pulse_enabled = false;
    cfg.min_tick_x10_us = 30U;
    cfg.max_tick_x10_us = 900U;
    cfg.crc_init_seed = 0x03U;  /* SAE J2716 APR2016 recommended */
    return cfg;
}

/* Validate SENT config fields (nibble count 1-8, tick range nonzero and ordered).
 * @param config  pointer to configuration struct
 * @return        true if valid */
bool sent_validate_config(const sent_config_t* config) {
    SENT_ASSERT(config != NULL);
    if (config->data_nibbles < 1U || config->data_nibbles > SENT_MAX_DATA_NIBBLES) {
        return false;
    }
    if (config->min_tick_x10_us == 0U || config->max_tick_x10_us == 0U ||
        config->min_tick_x10_us > config->max_tick_x10_us) {
        return false;
    }
    return true;
}

/* Pack an array of 4-bit nibbles into a single uint32 according to nibble order.
 * @param nibbles       array of 4-bit nibble values (0x0-0xF)
 * @param nibble_count  number of nibbles to pack (max 8)
 * @param order         MSB_FIRST or LSB_FIRST
 * @return              packed 32-bit value */
uint32_t sent_pack_nibbles(const uint8_t* nibbles,
                           size_t nibble_count,
                           sent_nibble_order_t order) {
    SENT_ASSERT(nibbles != NULL);
    if (nibble_count == 0U) {
        return 0U;
    }

    if (nibble_count > SENT_MAX_DATA_NIBBLES) {
        nibble_count = SENT_MAX_DATA_NIBBLES;
    }

    uint32_t packed = 0U;
    if (order == SENT_NIBBLE_ORDER_MSB_FIRST) {
        for (size_t i = 0; i < nibble_count; ++i) {
            packed = (packed << 4U) | (uint32_t)(nibbles[i] & 0x0FU);
        }
    } else {
        for (size_t i = 0; i < nibble_count; ++i) {
            packed |= ((uint32_t)(nibbles[i] & 0x0FU) << (4U * i));
        }
    }

    return packed;
}

/* Unpack a uint32 into an array of 4-bit nibbles according to nibble order.
 * @param packed        packed 32-bit value
 * @param nibble_count  number of nibbles to extract (max 8)
 * @param order         MSB_FIRST or LSB_FIRST
 * @param out_nibbles   [out] array to receive 4-bit nibble values
 * @return              true on success */
bool sent_unpack_nibbles(uint32_t packed,
                         uint8_t nibble_count,
                         sent_nibble_order_t order,
                         uint8_t* out_nibbles) {
    SENT_ASSERT(out_nibbles != NULL);
    if (nibble_count == 0U || nibble_count > SENT_MAX_DATA_NIBBLES) {
        return false;
    }

    if (order == SENT_NIBBLE_ORDER_MSB_FIRST) {
        for (uint8_t i = 0; i < nibble_count; ++i) {
            uint8_t shift = (uint8_t)(4U * (nibble_count - 1U - i));
            out_nibbles[i] = (uint8_t)((packed >> shift) & 0x0FU);
        }
    } else {
        for (uint8_t i = 0; i < nibble_count; ++i) {
            uint8_t shift = (uint8_t)(4U * i);
            out_nibbles[i] = (uint8_t)((packed >> shift) & 0x0FU);
        }
    }

    return true;
}
