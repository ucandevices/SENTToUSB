/* stm32c0xx_it.c — core exception and USB interrupt handlers for the
 * STM32C071 build.  The SENT timer and DMA handlers are in sent_app_c071.c. */
#include "main.h"
#include "sent_app.h"

extern PCD_HandleTypeDef hpcd_USB_FS;

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  SentApp_OnTick1ms();   /* TIM2 edge-rate guard window */
}

void USB_DRD_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}
