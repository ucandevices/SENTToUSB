#ifndef SENT_CAN_FRAME_H
#define SENT_CAN_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_CAN_MAX_DLC 8U

typedef struct {
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[SENT_CAN_MAX_DLC];
} sent_can_frame_t;

static inline bool sent_can_frame_equal(const sent_can_frame_t* lhs,
                                        const sent_can_frame_t* rhs) {
    if (lhs == NULL || rhs == NULL) {
        return false;
    }
    if (lhs->id != rhs->id || lhs->extended != rhs->extended || lhs->dlc != rhs->dlc) {
        return false;
    }
    for (size_t i = 0; i < SENT_CAN_MAX_DLC; ++i) {
        if (lhs->data[i] != rhs->data[i]) {
            return false;
        }
    }
    return true;
}

#ifdef __cplusplus
}
#endif

#endif  /* SENT_CAN_FRAME_H */
