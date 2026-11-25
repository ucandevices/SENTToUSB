#include "sent/sent_crc.h"

/* Advance the 4-bit CRC register by one nibble (4 shifts, polynomial
 * x^4 + x^3 + x^2 + 1).  Equivalent to the 16-entry table in J2716 5.4.2.2.2. */
static uint8_t crc4_step(uint8_t crc) {
    for (int bit = 0; bit < 4; ++bit) {
        if ((crc & 0x08U) != 0U) {
            crc = (uint8_t)((crc << 1U) ^ 0x0DU);
        } else {
            crc = (uint8_t)(crc << 1U);
        }
        crc &= 0x0FU;
    }
    return crc;
}

/* Compute SAE J2716 CRC-4 over data nibbles (polynomial 0x0D).
 * @param data_nibbles  array of 4-bit data nibbles
 * @param nibble_count  number of data nibbles
 * @param mode          DATA_ONLY or STATUS_AND_DATA
 * @param status_nibble 4-bit status nibble (used only when mode includes status)
 * @param init_seed     start value; 0x03 gives the J2716 recommended CRC
 * @return              4-bit CRC result (0x0-0xF) */
uint8_t sent_crc4_j2716(const uint8_t* data_nibbles,
                        size_t nibble_count,
                        sent_crc_mode_t mode,
                        uint8_t status_nibble,
                        uint8_t init_seed) {
    uint8_t crc = init_seed & 0x0FU;

    if (mode == SENT_CRC_MODE_STATUS_AND_DATA) {
        crc = crc4_step((uint8_t)(crc ^ (status_nibble & 0x0FU)));
    }

    if (data_nibbles != NULL) {
        for (size_t i = 0; i < nibble_count; ++i) {
            crc = crc4_step((uint8_t)(crc ^ (data_nibbles[i] & 0x0FU)));
        }
    }

    return (uint8_t)(crc & 0x0FU);
}

/* Compute the SAE J2716 legacy CRC-4 (section 5.4.2.1, FEB2008 and older):
 * seed 0101, checksum = nibble XOR T[checksum], no trailing zero nibble. */
uint8_t sent_crc4_j2716_legacy(const uint8_t* data_nibbles,
                               size_t nibble_count,
                               sent_crc_mode_t mode,
                               uint8_t status_nibble) {
    uint8_t crc = 0x05U;

    if (mode == SENT_CRC_MODE_STATUS_AND_DATA) {
        crc = (uint8_t)(crc4_step(crc) ^ (status_nibble & 0x0FU));
    }

    if (data_nibbles != NULL) {
        for (size_t i = 0; i < nibble_count; ++i) {
            crc = (uint8_t)(crc4_step(crc) ^ (data_nibbles[i] & 0x0FU));
        }
    }

    return (uint8_t)(crc & 0x0FU);
}

/* Compute the frame CRC with the algorithm selected in the configuration. */
uint8_t sent_crc4_frame(const sent_config_t* config,
                        uint8_t status_nibble,
                        const uint8_t* data_nibbles,
                        size_t nibble_count) {
    if (config->crc_legacy) {
        return sent_crc4_j2716_legacy(data_nibbles, nibble_count, config->crc_mode, status_nibble);
    }
    return sent_crc4_j2716(data_nibbles, nibble_count, config->crc_mode, status_nibble,
                           config->crc_init_seed);
}

uint8_t sent_crc6_esm_j2716(uint32_t message_bits) {
    uint8_t crc = 0x15U;

    for (int bit_idx = 23; bit_idx >= -6; --bit_idx) {
        uint8_t bit = (bit_idx >= 0) ? (uint8_t)((message_bits >> (uint8_t)bit_idx) & 1U) : 0U;
        uint8_t feedback = (uint8_t)(crc & 0x20U);
        crc = (uint8_t)(((crc << 1U) | bit) & 0x3FU);
        if (feedback != 0U) {
            crc ^= 0x19U;
        }
    }
    return (uint8_t)(crc & 0x3FU);
}
