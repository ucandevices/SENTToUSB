#include "sent/sent_slow_channel.h"

#include <stddef.h>
#include <string.h>

#include "sent/sent_crc.h"
#include "sent/sent_protocol.h"

#define SENT_ESM_SYNC_PATTERN 0x7EU

static void esm_reset_receive(sent_slow_channel_t* ctx) {
    ctx->esm_active = 0U;
    ctx->esm_frame_counter = 0U;
    ctx->esm_rx_crc = 0U;
    ctx->esm_config_bit = 0U;
    ctx->esm_crc_bits = 0U;
    ctx->esm_crc_message = 0U;
    ctx->esm_data_bits = 0U;
    ctx->esm_id_bits = 0U;
    ctx->esm_data_high_bits = 0U;
}

static void short_reset_receive(sent_slow_channel_t* ctx) {
    ctx->short_active = 0U;
    ctx->short_bit_count = 0U;
    ctx->short_bits = 0U;
}

void sent_slow_channel_init(sent_slow_channel_t* ctx) {
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->short_crc_seed = 0x03U;  /* J2716 recommended CRC */
}

void sent_slow_channel_set_crc(sent_slow_channel_t* ctx, uint8_t init_seed, bool legacy) {
    if (ctx == NULL) {
        return;
    }
    ctx->short_crc_seed = (uint8_t)(init_seed & 0x0FU);
    ctx->short_crc_legacy = legacy ? 1U : 0U;
}

void sent_slow_channel_reset(sent_slow_channel_t* ctx) {
    if (ctx == NULL) {
        return;
    }
    esm_reset_receive(ctx);
    short_reset_receive(ctx);
}

static void esm_append_crc_bit(sent_slow_channel_t* ctx, uint8_t bit) {
    ctx->esm_crc_message = (uint32_t)((ctx->esm_crc_message << 1U) | (uint32_t)(bit & 1U));
    ctx->esm_crc_bits++;
}

static void esm_start_at_frame7(sent_slow_channel_t* ctx, uint8_t b2) {
    esm_reset_receive(ctx);
    ctx->esm_active = 1U;
    ctx->esm_frame_counter = 7U;
    ctx->esm_rx_crc = (uint8_t)((ctx->esm_b2_shift >> 1U) & 0x3FU);
    ctx->esm_data_bits = (uint16_t)(b2 & 1U);
    esm_append_crc_bit(ctx, b2);
    esm_append_crc_bit(ctx, 0U);
}

static bool esm_finish(sent_slow_channel_t* ctx, sent_slow_message_t* out_message) {
    if (ctx->esm_crc_bits != 24U) {
        esm_reset_receive(ctx);
        return false;
    }

    uint8_t computed_crc = sent_crc6_esm_j2716(ctx->esm_crc_message & 0xFFFFFFUL);
    if (computed_crc != ctx->esm_rx_crc) {
        esm_reset_receive(ctx);
        return false;
    }

    if (out_message != NULL) {
        if (ctx->esm_config_bit == 0U) {
            out_message->format = SENT_SLOW_FORMAT_ENHANCED_12_8;
            out_message->message_id = (uint8_t)(ctx->esm_id_bits & 0xFFU);
            out_message->data = (uint16_t)(ctx->esm_data_bits & 0x0FFFU);
        } else {
            out_message->format = SENT_SLOW_FORMAT_ENHANCED_16_4;
            out_message->message_id = (uint8_t)(ctx->esm_id_bits & 0x0FU);
            out_message->data = (uint16_t)(((uint16_t)ctx->esm_data_high_bits << 12U) |
                                           (ctx->esm_data_bits & 0x0FFFU));
        }
    }

    esm_reset_receive(ctx);
    return true;
}

static bool esm_process(sent_slow_channel_t* ctx,
                        uint8_t b2,
                        uint8_t b3,
                        sent_slow_message_t* out_message) {
    ctx->esm_b3_shift = (uint8_t)(((uint16_t)ctx->esm_b3_shift << 1U) | (uint16_t)b3);
    ctx->esm_b2_shift = (uint8_t)(((uint16_t)ctx->esm_b2_shift << 1U) | (uint16_t)b2);

    if (!ctx->esm_active) {
        if (ctx->esm_b3_shift == SENT_ESM_SYNC_PATTERN) {
            esm_start_at_frame7(ctx, b2);
        }
        return false;
    }

    ctx->esm_frame_counter++;
    esm_append_crc_bit(ctx, b2);
    esm_append_crc_bit(ctx, b3);

    switch (ctx->esm_frame_counter) {
    case 8U:
        ctx->esm_data_bits = (uint16_t)((ctx->esm_data_bits << 1U) | b2);
        ctx->esm_config_bit = b3;
        break;
    case 9U:
    case 10U:
    case 11U:
    case 12U:
        ctx->esm_data_bits = (uint16_t)((ctx->esm_data_bits << 1U) | b2);
        ctx->esm_id_bits = (uint16_t)((ctx->esm_id_bits << 1U) | b3);
        break;
    case 13U:
        if (b3 != 0U) {
            esm_reset_receive(ctx);
            return false;
        }
        ctx->esm_data_bits = (uint16_t)((ctx->esm_data_bits << 1U) | b2);
        break;
    case 14U:
    case 15U:
    case 16U:
    case 17U:
        ctx->esm_data_bits = (uint16_t)((ctx->esm_data_bits << 1U) | b2);
        if (ctx->esm_config_bit == 0U) {
            ctx->esm_id_bits = (uint16_t)((ctx->esm_id_bits << 1U) | b3);
        } else {
            ctx->esm_data_high_bits = (uint8_t)((ctx->esm_data_high_bits << 1U) | b3);
        }
        break;
    case 18U:
        if (b3 != 0U) {
            esm_reset_receive(ctx);
            return false;
        }
        ctx->esm_data_bits = (uint16_t)((ctx->esm_data_bits << 1U) | b2);
        return esm_finish(ctx, out_message);
    default:
        esm_reset_receive(ctx);
        break;
    }

    return false;
}

static bool short_process(sent_slow_channel_t* ctx,
                          uint8_t b2,
                          uint8_t b3,
                          sent_slow_message_t* out_message) {
    if (b3 != 0U) {
        ctx->short_active = 1U;
        ctx->short_bit_count = 1U;
        ctx->short_bits = (uint16_t)(b2 & 1U);
        return false;
    }

    if (!ctx->short_active) {
        return false;
    }

    ctx->short_bits = (uint16_t)((ctx->short_bits << 1U) | b2);
    ctx->short_bit_count++;
    if (ctx->short_bit_count < 16U) {
        return false;
    }

    uint8_t id = (uint8_t)((ctx->short_bits >> 12U) & 0x0FU);
    uint8_t data = (uint8_t)((ctx->short_bits >> 4U) & 0xFFU);
    uint8_t rx_crc = (uint8_t)(ctx->short_bits & 0x0FU);
    uint8_t crc_nibbles[3] = {
        id,
        (uint8_t)(data >> 4U),
        (uint8_t)(data & 0x0FU),
    };
    /* J2716 5.2.4.1 / 5.4.2: the short message uses the same 4-bit CRC
     * implementation as the frame checksum of the transmitter. */
    uint8_t computed_crc = (ctx->short_crc_legacy != 0U)
        ? sent_crc4_j2716_legacy(crc_nibbles, 3U, SENT_CRC_MODE_DATA_ONLY, 0U)
        : sent_crc4_j2716(crc_nibbles, 3U, SENT_CRC_MODE_DATA_ONLY, 0U, ctx->short_crc_seed);
    short_reset_receive(ctx);

    if (computed_crc != rx_crc) {
        return false;
    }

    if (out_message != NULL) {
        out_message->format = SENT_SLOW_FORMAT_SHORT;
        out_message->message_id = id;
        out_message->data = data;
    }
    return true;
}

bool sent_slow_channel_process_status(sent_slow_channel_t* ctx,
                                      uint8_t status_nibble,
                                      sent_slow_message_t* out_message) {
    if (ctx == NULL) {
        return false;
    }
    if (out_message != NULL) {
        out_message->format = SENT_SLOW_FORMAT_NONE;
        out_message->message_id = 0U;
        out_message->data = 0U;
    }

    uint8_t b2 = (uint8_t)((status_nibble >> 2U) & 1U);
    uint8_t b3 = (uint8_t)((status_nibble >> 3U) & 1U);

    uint8_t was_esm_active = ctx->esm_active;
    if (esm_process(ctx, b2, b3, out_message)) {
        short_reset_receive(ctx);
        return true;
    }
    if (was_esm_active || ctx->esm_active) {
        short_reset_receive(ctx);
        return false;
    }

    return short_process(ctx, b2, b3, out_message);
}
