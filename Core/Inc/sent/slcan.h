#ifndef SENT_SLCAN_H
#define SENT_SLCAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/can_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_SLCAN_MAX_LINE_LEN 32U

typedef enum {
    SENT_SLCAN_CMD_OPEN = 0,
    SENT_SLCAN_CMD_CLOSE,
    SENT_SLCAN_CMD_VERSION,
    SENT_SLCAN_CMD_SERIAL,
    SENT_SLCAN_CMD_FRAME,
    SENT_SLCAN_CMD_UNSUPPORTED,
    SENT_SLCAN_CMD_INVALID,
} sent_slcan_command_type_t;

typedef struct {
    sent_slcan_command_type_t type;
    bool has_frame;
    sent_can_frame_t frame;
} sent_slcan_command_t;

bool sent_slcan_parse_line(const char* line, sent_slcan_command_t* out_command);
bool sent_slcan_serialize_frame(const sent_can_frame_t* frame,
                                char* out_line,
                                size_t out_line_size);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_SLCAN_H */
