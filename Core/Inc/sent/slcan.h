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
    SENT_SLCAN_CMD_OPEN = 0,   /* O — open channel (normal mode) */
    SENT_SLCAN_CMD_LISTEN,     /* L — open channel (listen-only mode, treated as O) */
    SENT_SLCAN_CMD_CLOSE,      /* C — close channel */
    SENT_SLCAN_CMD_VERSION,    /* V — hardware version query */
    SENT_SLCAN_CMD_FWVERSION,  /* v — firmware version query */
    SENT_SLCAN_CMD_SERIAL,     /* N — serial number query */
    SENT_SLCAN_CMD_SETBAUD,    /* S/s — set bitrate (ignored, USB CDC) */
    SENT_SLCAN_CMD_STATUS,     /* F — read status flags */
    SENT_SLCAN_CMD_FRAME,      /* t/T — CAN frame */
    SENT_SLCAN_CMD_UNSUPPORTED,/* valid but unimplemented command → ack with \r */
    SENT_SLCAN_CMD_INVALID,    /* malformed / empty line → nack with \a */
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
