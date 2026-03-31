#include "sent/sent_crc.h"

/* Compute SAE J2716 CRC-4 over data nibbles (polynomial 0x0D, init 0x05).
 * @param data_nibbles  array of 4-bit data nibbles
 * @param nibble_count  number of data nibbles
 * @param mode          DATA_ONLY or STATUS_AND_DATA
 * @param status_nibble 4-bit status nibble (used only when mode includes status)
 * @return              4-bit CRC result (0x0-0xF) */
uint8_t sent_crc4_j2716(const uint8_t* data_nibbles,
                        size_t nibble_count,
                        sent_crc_mode_t mode,
                        uint8_t status_nibble,
                        uint8_t init_seed) {
    uint8_t crc = init_seed & 0x0FU;

    if (mode == SENT_CRC_MODE_STATUS_AND_DATA) {
        uint8_t nibble = (uint8_t)(status_nibble & 0x0FU);
        crc ^= nibble;
        for (int bit = 0; bit < 4; ++bit) {
            if ((crc & 0x08U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x0DU);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
            crc &= 0x0FU;
        }
    }

    if (data_nibbles != NULL) {
        for (size_t i = 0; i < nibble_count; ++i) {
            uint8_t nibble = (uint8_t)(data_nibbles[i] & 0x0FU);
            crc ^= nibble;
            for (int bit = 0; bit < 4; ++bit) {
                if ((crc & 0x08U) != 0U) {
                    crc = (uint8_t)((crc << 1U) ^ 0x0DU);
                } else {
                    crc = (uint8_t)(crc << 1U);
                }
                crc &= 0x0FU;
            }
        }
    }

    return (uint8_t)(crc & 0x0FU);
}
