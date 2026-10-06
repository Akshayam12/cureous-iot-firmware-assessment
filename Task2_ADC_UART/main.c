/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ADC sampling @10 Hz (TIM2 tick), 10-sample moving average,
  *                   JSON streaming over USART2 @115200 baud
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Moving-average window size (number of samples). */
#define SAMPLE_COUNT          10U

/* Timeouts (ms). Finite values so a hung peripheral can never block forever. */
#define ADC_TIMEOUT_MS        10U
#define UART_TIMEOUT_MS       100U

/* TIM2 timing: timer clock = 84 MHz (APB1 = 42 MHz, x2 because APB1 prescaler
 * is /2). 84 MHz / (8399+1) = 10 kHz tick; 10 kHz / (999+1) = 10 Hz
 * -> one update event exactly every 100 ms. */
#define TIM2_PRESCALER        8399U
#define TIM2_PERIOD           999U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

TIM_HandleTypeDef htim2;

/* Moving-average filter state */
static uint32_t adc_samples[SAMPLE_COUNT];  /* circular buffer of last N raw samples */
static uint32_t adc_sum         = 0U;       /* running sum of the samples in the buffer */
static uint8_t  sample_index    = 0U;       /* position of the oldest sample (next to overwrite) */
static uint8_t  samples_filled  = 0U;       /* how many valid samples are in the buffer (0..N) */
static uint32_t adc_average     = 0U;       /* latest filtered result */

/* Set by the TIM2 ISR every 100 ms, cleared by the main loop */
static volatile uint8_t sample_ready = 0U;

/* Diagnostics counters */
static volatile uint32_t missed_ticks = 0U; /* ticks that arrived before the previous one was handled */
static uint32_t adc_error_count       = 0U;
static uint32_t uart_error_count      = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM2_Init(void);
static HAL_StatusTypeDef Read_ADC_And_Calculate_Average(void);
static void Send_Average_Over_UART(uint32_t average);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM2_Init();

  /* Start TIM2 in interrupt mode. From now on the update interrupt fires
   * exactly every 100 ms (10 Hz) and sets sample_ready. */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Sampling period is defined by the TIM2 hardware tick, NOT by software
     * delays, so conversion/printf/UART time does not cause drift. */
    if (sample_ready)
    {
      sample_ready = 0U;

      if (Read_ADC_And_Calculate_Average() == HAL_OK)
      {
        /* One transmission per 100 ms tick */
        Send_Average_Over_UART(adc_average);
      }
      else
      {
        /* ADC fault: count it and toggle LD2 as a visible indicator */
        adc_error_count++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Configure TIM2 as a 10 Hz (100 ms) periodic interrupt source.
  * @note   Written by hand so this file is self-contained. If you instead
  *         enable TIM2 in CubeMX (Prescaler 8399, Period 999, update
  *         interrupt + NVIC enabled), delete this function, the TIM2_IRQHandler
  *         below, and the htim2 definition, since CubeMX will generate them.
  */
static void MX_TIM2_Init(void)
{
  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = TIM2_PRESCALER;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = TIM2_PERIOD;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  /* Enable the TIM2 peripheral clock and its interrupt line */
  __HAL_RCC_TIM2_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  TIM2 interrupt handler: forwards to the HAL, which then calls
  *         HAL_TIM_PeriodElapsedCallback().
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief  Timer period-elapsed callback, executed every 100 ms by TIM2.
  * @note   Kept minimal: just raises a flag. All the slow work (ADC read,
  *         formatting, UART transmit) happens in the main loop.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (sample_ready)
    {
      /* Main loop didn't finish the previous sample in time */
      missed_ticks++;
    }
    sample_ready = 1U;
  }
}

/**
  * @brief  Take one ADC reading and update the 10-sample moving average.
  * @retval HAL_OK on success, HAL_ERROR / HAL_TIMEOUT on ADC failure
  *         (the filter state is left untouched on failure).
  *
  * Moving-average algorithm (circular buffer + running sum):
  *   - The buffer holds the last SAMPLE_COUNT raw readings.
  *   - sample_index points at the OLDEST reading.
  *   - On each new sample we subtract the oldest value from the running sum,
  *     overwrite it with the new value, add the new value to the sum, and
  *     advance the index (wrapping to 0). That is O(1) per sample instead of
  *     re-adding all 10 values every time.
  *   - average = sum / number_of_valid_samples.
  */
static HAL_StatusTypeDef Read_ADC_And_Calculate_Average(void)
{
  uint32_t raw;

  /* Start a single software-triggered conversion */
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Wait for end of conversion with a finite timeout (never HAL_MAX_DELAY) */
  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return HAL_TIMEOUT;
  }

  raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  /* --- Moving average update --- */
  adc_sum -= adc_samples[sample_index];    /* remove oldest sample from sum */
  adc_samples[sample_index] = raw;         /* store newest sample in its place */
  adc_sum += raw;                          /* add newest sample to sum */

  sample_index++;                          /* advance circular index */
  if (sample_index >= SAMPLE_COUNT)
  {
    sample_index = 0U;
  }

  /* During start-up the buffer is not full yet (it was zero-initialised).
   * Dividing by SAMPLE_COUNT would give an artificially low average, so we
   * divide by the number of real samples until the window is full. */
  if (samples_filled < SAMPLE_COUNT)
  {
    samples_filled++;
  }
  adc_average = adc_sum / samples_filled;

  return HAL_OK;
}

/**
  * @brief  Format the average as a JSON line and send it over USART2.
  *         Output example: {"adc_avg":2048}\r\n
  * @note   Called once per 100 ms tick. At 115200 baud the ~20-byte message
  *         takes ~2 ms, well within the 100 ms period.
  */
static void Send_Average_Over_UART(uint32_t average)
{
  char msg[48];

  int len = snprintf(msg, sizeof(msg),
                     "{\"adc_avg\":%lu}\r\n", (unsigned long)average);

  /* Reject formatting errors and truncated output */
  if ((len <= 0) || ((size_t)len >= sizeof(msg)))
  {
    uart_error_count++;
    return;
  }

  /* Only transmit if the UART is idle; otherwise skip and count the error */
  if (huart2.gState != HAL_UART_STATE_READY)
  {
    uart_error_count++;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    return;
  }

  if (HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len,
                        UART_TIMEOUT_MS) != HAL_OK)
  {
    uart_error_count++;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
