/*
 * main_c071.c — STM32C071K8T6 (4-channel board, LQFP32) startup: option bytes / DFU,
 * clocks, GPIO, USB, then the SENT application (sent_app_c071.c).
 *
 * Hand-written (no CubeMX project yet, plan 4.1).  Pin plan: plan section 2.5.
 *
 * Clocks: SYSCLK = HSIUSB48 (48 MHz), the RC oscillator that the CRS trims to
 * the USB start-of-frame (configured in HAL_PCD_MspInit, usbd_conf.c), the
 * same arrangement as HSI48 on the F042.  The SENT timers therefore run at a
 * USB-locked 48 MHz.  USB clock = HSIUSB48.  AHB = APB = 48 MHz.
 */
#include "main.h"
#include "usb_device.h"
#include "sent_app.h"

/* STM32C0 system memory (ROM bootloader; USB DFU only on the C071, RM0490). */
#define SYSTEM_MEMORY_BASE 0x1FFF0000UL

/* Boot option bits for normal operation: boot from main flash and ignore the
 * PA14-BOOT0 pin (it is SWCLK on this board), as on the F042. */
#define OB_BOOT_MASK   (FLASH_OPTR_nBOOT_SEL | FLASH_OPTR_nBOOT1 | FLASH_OPTR_nBOOT0)
#define OB_BOOT_NORMAL (FLASH_OPTR_nBOOT_SEL | FLASH_OPTR_nBOOT1 | FLASH_OPTR_nBOOT0)
/* nBOOT0 = 0 with nBOOT1 = 1: boot from system memory (ROM bootloader). */
#define OB_BOOT_DFU    (FLASH_OPTR_nBOOT_SEL | FLASH_OPTR_nBOOT1)

static void MX_GPIO_Init(void);

/* Program the three boot option bits and reload the option bytes, which
 * resets the chip.  Only called when the bits differ from the target. */
static void program_boot_bits(uint32_t target)
{
  FLASH_OBProgramInitTypeDef ob = {0};
  ob.OptionType = OPTIONBYTE_USER;
  ob.USERType   = OB_USER_NBOOT_SEL | OB_USER_NBOOT1 | OB_USER_NBOOT0;
  ob.USERConfig = target;

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();
  if (HAL_FLASHEx_OBProgram(&ob) == HAL_OK) {
    HAL_FLASH_OB_Launch();      /* option byte reload = system reset */
  }
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
}

/* Called by the CDC 'boot' command (usbd_cdc_if.c).  Like the F042 build:
 * select system-memory boot and reset; the ROM bootloader then enumerates as
 * USB DFU.  When the DFU tool leaves, the bootloader starts the application,
 * which puts the normal boot bits back (main()).  The option-byte route is
 * used instead of a direct jump because a direct jump leaves USB/clock state
 * behind (a known source of enumeration failures on the C071). */
void EnterSystemDfu(void)
{
  __disable_irq();
  program_boot_bits(OB_BOOT_DFU);
  NVIC_SystemReset();   /* not reached unless programming failed */
}

int main(void)
{
  /* Restore normal boot bits after a DFU session (or on a factory-fresh part).
   * HAL_FLASHEx_OBProgram only needs the flash interface, so this runs
   * before HAL_Init. */
  if ((FLASH->OPTR & OB_BOOT_MASK) != OB_BOOT_NORMAL) {
    program_boot_bits(OB_BOOT_NORMAL);
  }

  /* Disable the USB IRQ immediately: the ROM DFU bootloader leaves it
   * enabled, and a USB reset that arrives before HAL_PCD_Init has set up the
   * handle ends in Error_Handler (seen on the F042). */
  NVIC_DisableIRQ(USB_DRD_FS_IRQn);
  NVIC_ClearPendingIRQ(USB_DRD_FS_IRQn);
  __DSB();
  __ISB();

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  /* Timers, DMA and the SENT core (TIM2 capture, TIM3/TIM1/TIM16/TIM17 TX). */
  SentApp_Init();

  while (1)
  {
    SentApp_Process();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState = RCC_HSI_ON;
  osc.HSIDiv = RCC_HSI_DIV1;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.HSI48State = RCC_HSI48_ON;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSIUSB48;
  clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider = RCC_HCLK_DIV1;
  clk.APB1CLKDivider = RCC_APB1_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Board GPIOs (rev 2b, plan 2.6).  The timer pins are set up in
 * sent_app_c071.c; USB DM/DP (PA11/PA12) need no GPIO configuration.
 * Every output level is written to ODR before the pin is switched to output
 * mode, so no pin glitches to the wrong level.  PC6 is not touched (tied to
 * +3V3 on the board, stays analog). */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Levels first: sensor supply OFF (switched on only after USB
   * SET_CONFIGURATION, sent_app_c071.c; TPS2553 EN is active high with a
   * 100 k pull-down), LED off, TX buffers disabled (TX_EN high), termination
   * off (TERM_EN low). */
  HAL_GPIO_WritePin(VS_EN_GPIO_Port, VS_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pins, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TERM_EN1_GPIO_Port, TERM_EN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TERM_EN2_GPIO_Port, TERM_EN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TERM_EN34_GPIO_Port, TERM_EN3_Pin | TERM_EN4_Pin, GPIO_PIN_RESET);

  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = VS_EN_Pin;
  HAL_GPIO_Init(VS_EN_GPIO_Port, &g);
  g.Pin = LED_Pin;
  HAL_GPIO_Init(LED_GPIO_Port, &g);
  g.Pin = TX_EN_Pins;
  HAL_GPIO_Init(TX_EN_GPIO_Port, &g);
  g.Pin = TERM_EN1_Pin;
  HAL_GPIO_Init(TERM_EN1_GPIO_Port, &g);
  g.Pin = TERM_EN2_Pin;
  HAL_GPIO_Init(TERM_EN2_GPIO_Port, &g);
  g.Pin = TERM_EN3_Pin | TERM_EN4_Pin;
  HAL_GPIO_Init(TERM_EN34_GPIO_Port, &g);

  g.Pin = VS_FLT_Pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VS_FLT_GPIO_Port, &g);

  g.Pin = VS_SRC_Pin;            /* rev 2d: pull-up on the board */
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VS_SRC_GPIO_Port, &g);

  /* Analog: VS_SENSE and I_SENSE (ADC) and PA9/PA10, which share the USB
   * pads with PA11/PA12 on this package (reset state, made explicit). */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin = VS_SENSE_Pin | IS_SENSE_Pin;
  HAL_GPIO_Init(VS_SENSE_GPIO_Port, &g);
  g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &g);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
