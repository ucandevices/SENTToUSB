/* main.h — STM32C071 (4-channel board) build. */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32c0xx_hal.h"

void Error_Handler(void);
void SystemClock_Config(void);

/* Select system-memory (USB DFU) boot in the option bytes and reset. */
void EnterSystemDfu(void);

/* Board pins (rev 2f and rev 3, STM32C071K8T6 LQFP32, plan section 2.7).
 * Rev 3 only changed the termination switches Q1-Q4 (AO3400A -> 2N7002, same
 * gate polarity), so the pinout and all levels are those of rev 2f; checked
 * pin by pin against the rev 3 PCB netlist and the CubeMX 6.15 pin database.
 * Never configure PC6 (pin 20, tied to +3V3 on the board: VDDIO2 on the "N"
 * variant) as an output.  PA9/PA10 (pins 19/21) are not connected; they stay
 * in analog mode. */
#define LED_GPIO_Port     GPIOB      /* status LED, active high (see led_poll()) */
#define LED_Pin           GPIO_PIN_8
#define VS_EN_GPIO_Port   GPIOA      /* TPS2553 EN, active high, 100 k pull-down */
#define VS_EN_Pin         GPIO_PIN_5
#define VS_FLT_GPIO_Port  GPIOA      /* load switch fault, open drain, active low */
#define VS_FLT_Pin        GPIO_PIN_4
#define VS_SENSE_GPIO_Port GPIOB     /* ADC: VS x 22/122 (100 k / 22 k + 100 nF, rev 2d) */
#define VS_SENSE_Pin      GPIO_PIN_1
#define IS_SENSE_GPIO_Port GPIOB     /* rev 2d, ADC1_IN19: sensor current, ~6.25 mV/mA (INA180A2, see IS_SCALE_*) */
#define IS_SENSE_Pin      GPIO_PIN_2
#define VS_SRC_GPIO_Port  GPIOB      /* rev 2d, TPS2121 ST (open drain, 10 k pull-up): H = EXT/off, L = USB */
#define VS_SRC_Pin        GPIO_PIN_9

/* TX_EN1..4: OE of the SN74LV1T125 TX buffers, active LOW (100 k pull-up on
 * the board, so the buffers are off until the firmware drives the pin). */
#define TX_EN_GPIO_Port   GPIOB
#define TX_EN_Pins        (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6)
#define TX_EN_Pin(ch)     ((uint16_t)(GPIO_PIN_3 << (ch)))   /* channel 0..3 -> PB3..PB6 */

/* TERM_EN1..4: termination, active HIGH (100 k pull-down on the board).
 * Drives the OE of an SN74LV1T126 (10 k pull-up to 5 V) and the gate of the
 * 2N7002 that connects the 2.2 nF (rev 3; AO3400A before).  The 2N7002 has
 * a higher threshold (2.5 V max) than the AO3400A; 3.3 V gate drive still
 * switches it on; even tens of ohms on-resistance with 2.2 nF is ~0.1 us,
 * small against the J2716 load time constant.
 * PA15, PB7, PC14, PC15. */
#define TERM_EN1_GPIO_Port GPIOA
#define TERM_EN1_Pin       GPIO_PIN_15
#define TERM_EN2_GPIO_Port GPIOB
#define TERM_EN2_Pin       GPIO_PIN_7
#define TERM_EN34_GPIO_Port GPIOC
#define TERM_EN3_Pin       GPIO_PIN_14
#define TERM_EN4_Pin       GPIO_PIN_15

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
