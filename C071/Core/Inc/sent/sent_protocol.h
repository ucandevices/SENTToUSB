/* SENT (SAE J2716) protocol constants, configuration, frame types, and
 * nibble pack/unpack helpers.
 *
 * All constants are expressed in ticks so they remain independent of the
 * chosen tick period (3 µs SAE default or sensor-specific). */
#ifndef SENT_SENT_PROTOCOL_H
#define SENT_SENT_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_MAX_DATA_NIBBLES 8U                           /* maximum payload nibbles per frame */
#define SENT_MAX_INTERVALS    (SENT_MAX_DATA_NIBBLES + 4U) /* sync + status + data×N + CRC [+ pause] */
#define SENT_MAX_TIMESTAMPS   (SENT_MAX_INTERVALS + 1U)    /* falling edges = intervals + 1 */

/* SAE J2716 protocol constants (tick-based, independent of tick period). */
#define SENT_SYNC_TICKS       56U   /* sync pulse duration in ticks */
#define SENT_NIBBLE_MIN_TICKS 12U   /* minimum data nibble ticks (J2716) */
#define SENT_NIBBLE_MAX_TICKS 27U   /* maximum data nibble ticks */
#define SENT_NIBBLE_OFFSET    12U   /* nibble value = ticks - OFFSET */
#define SENT_MAX_PAUSE_TICKS  768U  /* maximum pause pulse ticks */
#define SENT_MIN_PAUSE_TICKS  12U   /* minimum pause pulse ticks */

/* Controls which fields contribute to the CRC calculation. */
typedef enum {
    SENT_CRC_MODE_DATA_ONLY       = 0,  /* CRC covers data nibbles only */
    SENT_CRC_MODE_STATUS_AND_DATA = 1,  /* CRC covers status nibble followed by data nibbles */
} sent_crc_mode_t;

/* Byte order when packing/unpacking the multi-nibble payload value. */
typedef enum {
    SENT_NIBBLE_ORDER_MSB_FIRST = 0,  /* first nibble in the frame is the most significant */
    SENT_NIBBLE_ORDER_LSB_FIRST = 1,  /* first nibble in the frame is the least significant */
} sent_nibble_order_t;

/* Per-channel SENT protocol configuration. */
typedef struct {
    uint8_t data_nibbles;            /* payload nibble count [1–8] */
    sent_crc_mode_t crc_mode;        /* whether status nibble is mixed into CRC */
    sent_nibble_order_t order;       /* nibble packing order for sent_pack/unpack_nibbles */
    bool pause_pulse_enabled;        /* whether the encoder should append a pause pulse */
    uint16_t min_tick_x10_us;        /* minimum valid tick period [0.1-µs units] */
    uint16_t max_tick_x10_us;        /* maximum valid tick period [0.1-µs units] */
    /* Start value of the nibble-wise CRC (crc = T[crc ^ nibble]).
     *   0x03 = SAE J2716 recommended CRC (JAN2010 and later; equals seed 0101
     *          plus the standard's trailing zero nibble).
     *   0x05 = non-standard variant used by some sensors in the field.
     * Ignored when crc_legacy is set. */
    uint8_t crc_init_seed;
    /* true = SAE J2716 legacy CRC (FEB2008 and older, section 5.4.2.1):
     * seed 0101, no trailing zero nibble. */
    bool crc_legacy;
} sent_config_t;

/* Decoded or to-be-encoded SENT frame. */
typedef struct {
    uint8_t status;                          /* 4-bit status nibble */
    uint8_t data_nibbles[SENT_MAX_DATA_NIBBLES];  /* payload nibbles (4-bit each) */
    uint8_t data_nibbles_count;              /* number of valid entries in data_nibbles */
    uint8_t crc;                             /* 4-bit CRC nibble */
    uint16_t tick_x10_us;                   /* measured or intended tick period [0.1-µs units] */
    bool has_pause;                          /* true when a pause interval was present/requested */
    uint16_t pause_ticks;                    /* pause interval duration [ticks]; valid when has_pause */
} sent_frame_t;

/* Validate config: nibble count 1–8, tick range nonzero and min ≤ max. */
bool sent_validate_config(const sent_config_t* config);

/* Pack nibble_count 4-bit values from nibbles[] into a single uint32 (MSB or LSB first). */
uint32_t sent_pack_nibbles(const uint8_t* nibbles,
                           size_t nibble_count,
                           sent_nibble_order_t order);

/* Unpack nibble_count 4-bit values from packed into out_nibbles[]; returns false on bad args. */
bool sent_unpack_nibbles(uint32_t packed,
                         uint8_t nibble_count,
                         sent_nibble_order_t order,
                         uint8_t* out_nibbles);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SENT_PROTOCOL_H */
