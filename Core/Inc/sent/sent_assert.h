#ifndef SENT_ASSERT_H
#define SENT_ASSERT_H

#include "sent/hal_config.h"

#if defined(SENT_HAL_STM32F042)

/* Lightweight assert for embedded: triggers a debug breakpoint and halts.
 * No strings, no printf, no code size overhead. */
#define SENT_ASSERT(expr) \
    do {                   \
        if (!(expr)) {     \
            __asm("bkpt"); \
            while (1) {}   \
        }                  \
    } while (0)

#else

#include <assert.h>
#define SENT_ASSERT(expr) assert(expr)

#endif

#endif /* SENT_ASSERT_H */
