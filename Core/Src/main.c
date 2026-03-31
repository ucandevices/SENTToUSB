/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sent_app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Disable USB IRQ immediately: DFU bootloader leaves it enabled.
   * If a USB reset fires before HAL_PCD_Init sets Init.speed, the
   * PCD_ResetCallback sees speed==0 and calls Error_Handler. */
  NVIC->ICER[0] = (1U << 31U);   /* IRQ31 = USB */
  __DSB();
  __ISB();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  SentApp_Init();

  /* Start RX: TIM2 CH3 input capture interrupt + overflow interrupt */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  /* TIM14 is started on-demand when TX frames are submitted */

  /* RX starts when host sends SLCAN 'O' command */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SentApp_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535 ;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  /* Prescaler 143: 48 MHz / 144 = 333.33 kHz → 1 TIM14 tick = 3 µs = 1 SENT tick
   * (sent_build_intervals_ticks returns raw SENT ticks, used directly as TIM14 ticks) */
  htim14.Init.Prescaler = 143U;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  /* Preload DISABLED: __HAL_TIM_SET_AUTORELOAD takes immediate effect
   * so the ISR can set exact low/high durations on each overflow. */
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENT_TX_GPIO_Port, SENT_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SENT_TX_Pin */
  GPIO_InitStruct.Pin = SENT_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENT_TX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

extern volatile uint32_t g_tim14_isr_count;

/* With TIM14 prescaler=143 → 1 tick = 3 µs.
 * Low pulse = 5 × 3 µs = 15 µs.
 * ISR latency at 48 MHz < 1 µs = < 0.33 ticks → round to 0. */
#define SENT_TX_LOW_TICKS 5U
#define SENT_TX_FIRST_LOW_COMP_TICKS 0U
#define SENT_TX_FALL_EDGE_ISR_LATENCY_TICKS 0U

static volatile uint8_t g_sent_tx_in_low = 0;
static volatile uint8_t g_sent_tx_stop_after_low = 0;
static volatile uint8_t g_sent_tx_need_terminal_edge = 0;
static volatile uint8_t g_sent_tx_first_interval_high_comp = 0;
static volatile uint16_t g_sent_tx_interval = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uint16_t captured = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    SentApp_OnSentRxCaptureEdge(captured);
  }
}

void SentApp_OnTim14UpdateIrq(void)
{
  g_tim14_isr_count++;
  if (!g_sent_tx_in_low)
  {
    if (!SentApp_HasPendingTxIntervals())
    {
      if (g_sent_tx_need_terminal_edge)
      {
        /* Emit one delimiter edge to close the last pause interval cleanly. */
        SENT_TX_GPIO_Port->BRR = SENT_TX_Pin;
        __HAL_TIM_SET_COUNTER(&htim14, 0U);
        __HAL_TIM_SET_AUTORELOAD(&htim14, SENT_TX_LOW_TICKS - 1U);
        g_sent_tx_in_low = 1;
        g_sent_tx_stop_after_low = 1U;
        g_sent_tx_need_terminal_edge = 0U;
        return;
      }

      SENT_TX_GPIO_Port->BSRR = SENT_TX_Pin;
      g_sent_tx_in_low = 0U;
      g_sent_tx_stop_after_low = 0U;
      g_sent_tx_first_interval_high_comp = 0U;
      HAL_TIM_Base_Stop_IT(&htim14);
      return;
    }

    /* Phase 0: falling edge marks start of next SENT interval. */
    SENT_TX_GPIO_Port->BRR = SENT_TX_Pin;
    __HAL_TIM_SET_COUNTER(&htim14, 0U);
    uint16_t low_ticks = SENT_TX_LOW_TICKS;
    if (g_sent_tx_need_terminal_edge == 0U &&
        low_ticks > SENT_TX_FIRST_LOW_COMP_TICKS)
    {
      low_ticks = (uint16_t)(low_ticks - SENT_TX_FIRST_LOW_COMP_TICKS);
    }
    __HAL_TIM_SET_AUTORELOAD(&htim14, low_ticks - 1U);
    g_sent_tx_in_low = 1;
    g_sent_tx_stop_after_low = 0U;

    uint16_t next_ticks;
    if (SentApp_PopNextTxIntervalTicks(&next_ticks))
    {
      g_sent_tx_interval = next_ticks;
      g_sent_tx_first_interval_high_comp = 0U;
      g_sent_tx_need_terminal_edge = 1U;
    }
    else
    {
      SENT_TX_GPIO_Port->BSRR = SENT_TX_Pin;
      g_sent_tx_in_low = 0;
      g_sent_tx_stop_after_low = 0U;
      g_sent_tx_need_terminal_edge = 0U;
      g_sent_tx_first_interval_high_comp = 0U;
      HAL_TIM_Base_Stop_IT(&htim14);
    }
  }
  else
  {
    /* Phase 1: end of low pulse. */
    SENT_TX_GPIO_Port->BSRR = SENT_TX_Pin;
    if (g_sent_tx_stop_after_low)
    {
      g_sent_tx_in_low = 0U;
      g_sent_tx_stop_after_low = 0U;
      HAL_TIM_Base_Stop_IT(&htim14);
    }
    else
    {
      uint16_t high_ticks = (uint16_t)(g_sent_tx_interval - SENT_TX_LOW_TICKS);
      if (high_ticks > SENT_TX_FALL_EDGE_ISR_LATENCY_TICKS)
      {
        high_ticks = (uint16_t)(high_ticks - SENT_TX_FALL_EDGE_ISR_LATENCY_TICKS);
      }
      else
      {
        high_ticks = 1U;
      }
      if (g_sent_tx_first_interval_high_comp != 0U)
      {
        if (high_ticks > 1U)
        {
          high_ticks = (uint16_t)(high_ticks - 1U);
        }
        g_sent_tx_first_interval_high_comp = 0U;
      }

      __HAL_TIM_SET_COUNTER(&htim14, 0U);
      __HAL_TIM_SET_AUTORELOAD(&htim14, high_ticks - 1U);
      g_sent_tx_in_low = 0;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    SentApp_OnSentRxTimerOverflow();
  }
  else if (htim->Instance == TIM14)
  {
    SentApp_OnTim14UpdateIrq();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
