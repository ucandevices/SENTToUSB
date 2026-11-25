#include "sim_signal.h"

#include <string.h>

#include "sent/sent_crc.h"
#include "sent/sent_encoder.h"

bool sim_generate(const sim_channel_t* sim, uint64_t t0, sim_stream_t* out) {
    uint32_t counts_x10 = (uint32_t)sim->tick_x10_us * SIM_UNITS_PER_US;
    if ((counts_x10 % 10U) != 0U || sim->frames > SIM_MAX_FRAMES) {
        return false;
    }
    uint64_t per_tick = counts_x10 / 10U;
    uint32_t rng = sim->seed | 1U;
    uint64_t t = t0;
    memset(out, 0, sizeof(*out));
    out->t[out->count++] = t;
    for (uint32_t k = 0U; k < sim->frames; ++k) {
        sent_frame_t f;
        memset(&f, 0, sizeof(f));
        f.status = (uint8_t)(k & 0x3U);          /* bits 0..1 only: no slow-channel message */
        f.data_nibbles_count = sim->config.data_nibbles;
        for (uint8_t i = 0U; i < sim->config.data_nibbles; ++i) {
            rng = rng * 1103515245U + 12345U;
            f.data_nibbles[i] = (uint8_t)((rng >> 16) & 0x0FU);
        }
        out->status[k] = f.status;
        memcpy(out->data[k], f.data_nibbles, sizeof(f.data_nibbles));

        uint16_t iv[SENT_MAX_INTERVALS];
        size_t n = 0U;
        if (!sent_build_intervals_ticks(&f, &sim->config, sim->pause_ticks, iv, &n)) {
            return false;
        }
        for (size_t i = 0U; i < n; ++i) {
            t += (uint64_t)iv[i] * per_tick;
            out->t[out->count++] = t;
        }
    }
    return true;
}

uint8_t sim_expected_payload(const sim_stream_t* s, uint32_t frame, uint8_t nibbles,
                             uint8_t* out_bytes) {
    uint8_t bytes = (uint8_t)((nibbles + 1U) / 2U);
    uint8_t info = (bytes < 3U) ? 3U : bytes;
    memset(out_bytes, 0, SENT_CAN_MAX_DLC);
    for (uint8_t i = 0U; i < bytes; ++i) {
        uint8_t hi = (2U * i < nibbles) ? s->data[frame][2U * i] : 0U;
        uint8_t lo = (2U * i + 1U < nibbles) ? s->data[frame][2U * i + 1U] : 0U;
        out_bytes[i] = (uint8_t)((hi << 4) | lo);
    }
    out_bytes[info] = (uint8_t)(s->status[frame] << 4);
    return (uint8_t)(info + 1U);
}

size_t sim_merge(const sim_stream_t* streams, size_t n, sim_edge_t* out, size_t max) {
    size_t idx[8] = {0};
    size_t total = 0U;
    for (;;) {
        int best = -1;
        for (size_t c = 0U; c < n; ++c) {
            if (idx[c] < streams[c].count &&
                (best < 0 || streams[c].t[idx[c]] < streams[best].t[idx[best]])) {
                best = (int)c;
            }
        }
        if (best < 0 || total >= max) {
            return total;
        }
        out[total].t = (uint32_t)streams[best].t[idx[best]];   /* 32-bit timer view */
        out[total].ch = (uint8_t)best;
        total++;
        idx[best]++;
    }
}
