/* HAL backend selector.
 *
 * Auto-detects the target from compiler-defined device macros:
 *   STM32F042x6, STM32C071xx → SENT_HAL_STM32 (hal_stm32.c)
 *   anything else            → SENT_HAL_HOST (x86 simulation via pthreads)
 *
 * Override by defining exactly one of SENT_HAL_HOST or SENT_HAL_STM32 on the
 * compiler command line before including any SENT header.  The old name
 * SENT_HAL_STM32F042 is still accepted and means SENT_HAL_STM32. */
#ifndef SENT_HAL_CONFIG_H
#define SENT_HAL_CONFIG_H

#if defined(SENT_HAL_STM32F042) && !defined(SENT_HAL_STM32)
#define SENT_HAL_STM32 1
#endif

#if defined(SENT_HAL_HOST) && defined(SENT_HAL_STM32)
#error "Only one HAL backend can be enabled at a time."
#endif

#if !defined(SENT_HAL_HOST) && !defined(SENT_HAL_STM32)
#if defined(STM32F042x6) || defined(STM32C071xx)
#define SENT_HAL_STM32 1
#else
#define SENT_HAL_HOST 1
#endif
#endif

#endif  /* SENT_HAL_CONFIG_H */
