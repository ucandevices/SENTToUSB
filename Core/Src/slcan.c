#include "slcan.h"

#include "sent/sent_assert.h"

/* Return length of line excluding trailing CR/LF characters.
 * @param line  null-terminated input string
 * @return      trimmed length */
static size_t trim_len(const char* line) {
    size_t len = 0U;
    while (line[len] != '\0') {
        ++len;
    }
    while (len > 0U && (line[len - 1U] == '\r' || line[len - 1U] == '\n')) {
        --len;
    }
    return len;
}

/* Parse a hex string of given length into a uint32 value.
 * @param text       hex character string (not null-terminated, length-delimited)
 * @param len        number of hex characters to parse
 * @param out_value  [out] parsed 32-bit value
 * @return           true if all characters are valid hex */
static bool parse_hex_slice(const char* text, size_t len, uint32_t* out_value) {
    SENT_ASSERT(text != NULL && out_value != NULL);
    if (len == 0U) {
        return false;
    }

    uint32_t value = 0U;
    for (size_t i = 0; i < len; ++i) {
        char c = text[i];
        uint8_t nibble = 0U;
        if (c >= '0' && c <= '9') {
            nibble = (uint8_t)(c - '0');
        } else if (c >= 'a' && c <= 'f') {
            nibble = (uint8_t)(10 + (c - 'a'));
        } else if (c >= 'A' && c <= 'F') {
            nibble = (uint8_t)(10 + (c - 'A'));
        } else {
            return false;
        }

        if (value > (0xFFFFFFFFU >> 4U)) {
            return false;
        }
        value = (value << 4U) | nibble;
    }

    *out_value = value;
    return true;
}

/* Convert a 4-bit nibble value to its uppercase hex ASCII character.
 * @param nibble  value 0x0-0xF
 * @return        hex character '0'-'9' or 'A'-'F' */
static char nibble_to_hex(uint8_t nibble) {
    nibble &= 0x0FU;
    if (nibble < 10U) {
        return (char)('0' + (char)nibble);
    }
    return (char)('A' + (char)(nibble - 10U));
}

/* Append a value as uppercase hex digits to an output buffer at cursor position.
 * @param out     output character buffer
 * @param cursor  [in/out] current write position in buffer
 * @param value   value to encode
 * @param digits  number of hex digits to write */
static void append_hex(char* out, size_t* cursor, uint32_t value, size_t digits) {
    for (size_t i = 0; i < digits; ++i) {
        size_t shift = (digits - 1U - i) * 4U;
        uint8_t nibble = (uint8_t)((value >> shift) & 0x0FU);
        out[(*cursor)++] = nibble_to_hex(nibble);
    }
}

/* Parse an SLCAN text line into a command struct (O/C/V/N/t/T/r/R commands).
 * @param line         null-terminated SLCAN command string
 * @param out_command  [out] parsed command type and optional CAN frame
 * @return             true if parsing succeeded (command may still be INVALID) */
bool sent_slcan_parse_line(const char* line, sent_slcan_command_t* out_command) {
    SENT_ASSERT(line != NULL && out_command != NULL);

    out_command->type = SENT_SLCAN_CMD_INVALID;
    out_command->has_frame = false;

    size_t len = trim_len(line);
    if (len == 0U) {
        return true;
    }

    char op = line[0];
    if (op == 'O') {
        out_command->type = SENT_SLCAN_CMD_OPEN;
        return true;
    }
    if (op == 'L') {
        out_command->type = SENT_SLCAN_CMD_LISTEN;
        return true;
    }
    if (op == 'C') {
        out_command->type = SENT_SLCAN_CMD_CLOSE;
        return true;
    }
    if (op == 'V') {
        out_command->type = SENT_SLCAN_CMD_VERSION;
        return true;
    }
    if (op == 'v') {
        out_command->type = SENT_SLCAN_CMD_FWVERSION;
        return true;
    }
    if (op == 'N') {
        out_command->type = SENT_SLCAN_CMD_SERIAL;
        return true;
    }
    if (op == 'S' || op == 's') {
        out_command->type = SENT_SLCAN_CMD_SETBAUD;
        return true;
    }
    if (op == 'F') {
        out_command->type = SENT_SLCAN_CMD_STATUS;
        return true;
    }
    if (op == 'r' || op == 'R') {
        out_command->type = SENT_SLCAN_CMD_UNSUPPORTED;
        return true;
    }
    /* Any other single-char command: ack gracefully rather than nack */
    if (op != 't' && op != 'T') {
        out_command->type = SENT_SLCAN_CMD_UNSUPPORTED;
        return true;
    }

    bool extended = (op == 'T');
    size_t id_len = extended ? 8U : 3U;
    if (len < 1U + id_len + 1U) {
        return true;
    }

    uint32_t id = 0U;
    if (!parse_hex_slice(&line[1], id_len, &id)) {
        return true;
    }

    uint32_t dlc_u32 = 0U;
    if (!parse_hex_slice(&line[1U + id_len], 1U, &dlc_u32) || dlc_u32 > SENT_CAN_MAX_DLC) {
        return true;
    }
    uint8_t dlc = (uint8_t)dlc_u32;
    size_t payload_len = (size_t)dlc * 2U;
    if (len != 1U + id_len + 1U + payload_len) {
        return true;
    }

    sent_can_frame_t frame;
    frame.id = extended ? (id & 0x1FFFFFFFU) : (id & 0x7FFU);
    frame.extended = extended;
    frame.dlc = dlc;

    for (size_t i = 0U; i < dlc; ++i) {
        uint32_t byte_u32 = 0U;
        size_t at = 1U + id_len + 1U + (2U * i);
        if (!parse_hex_slice(&line[at], 2U, &byte_u32) || byte_u32 > 0xFFU) {
            return true;
        }
        frame.data[i] = (uint8_t)byte_u32;
    }

    out_command->type = SENT_SLCAN_CMD_FRAME;
    out_command->has_frame = true;
    out_command->frame = frame;
    return true;
}

/* Serialize a CAN frame into an SLCAN text line (t/T format).
 * @param frame          CAN frame to serialize (ID, DLC, data, extended flag)
 * @param out_line       [out] output buffer for null-terminated SLCAN string
 * @param out_line_size  size of output buffer [bytes]
 * @return               true if serialization succeeded */
bool sent_slcan_serialize_frame(const sent_can_frame_t* frame,
                                char* out_line,
                                size_t out_line_size) {
    SENT_ASSERT(frame != NULL && out_line != NULL);
    if (out_line_size == 0U) {
        return false;
    }

    uint8_t dlc = frame->dlc > SENT_CAN_MAX_DLC ? SENT_CAN_MAX_DLC : frame->dlc;
    size_t id_digits = frame->extended ? 8U : 3U;
    size_t needed = 1U + id_digits + 1U + ((size_t)dlc * 2U);
    if (out_line_size <= needed) {
        return false;
    }

    size_t w = 0U;
    out_line[w++] = frame->extended ? 'T' : 't';
    if (frame->extended) {
        append_hex(out_line, &w, frame->id & 0x1FFFFFFFU, 8U);
    } else {
        append_hex(out_line, &w, frame->id & 0x7FFU, 3U);
    }

    append_hex(out_line, &w, dlc, 1U);
    for (uint8_t i = 0U; i < dlc; ++i) {
        append_hex(out_line, &w, frame->data[i], 2U);
    }

    out_line[w] = '\0';
    return true;
}
