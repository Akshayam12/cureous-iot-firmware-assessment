/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FreeRTOS producer-consumer button logger
  *
  *  ISR (EXTI) --eventQueue--> Producer task --logQueue--> UART Logger task
  *
  *  - Two switches (B1 = PC13 -> ID 1, SW2 = PB5 -> ID 2) raise EXTI interrupts.
  *  - The ISR only records {button_id, tick} and posts it with the ISR-safe
  *    API. No printf, no blocking, no UART in interrupt context.
  *  - The Producer task turns each event into a JSON string, e.g.
  *        {"button_id": 1, "timestamp": 12500}
  *    and posts it to the log queue.
  *  - The UART Logger task blocks on the log queue and is the ONLY user of
  *    USART2, so the UART needs no mutex.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Button IDs reported in the JSON payload */
#define BTN1_ID                 1U      /* B1  (PC13, on-board button) */
#define BTN2_ID                 2U      /* SW2 (PB5, external switch)  */

/* Software debounce: ignore edges closer together than this (ms) */
#define DEBOUNCE_MS             50U

/* Queue sizes */
#define EVENT_QUEUE_LEN         16U     /* ISR -> Producer */
#define LOG_QUEUE_LEN           8U      /* Producer -> Logger */
#define LOG_MSG_MAX_LEN         64U     /* max JSON line length incl. \r\n */

/* Timeouts (ms). Every blocking call has a finite timeout. */
#define PRODUCER_WAIT_MS        1000U   /* Producer waiting for an event */
#define LOGGER_WAIT_MS          5000U   /* Logger waiting for a message */
#define QUEUE_POST_TIMEOUT_MS   10U     /* Producer posting to a full log queue */
#define UART_TX_TIMEOUT_MS      100U    /* HAL_UART_Transmit timeout */
#define UART_BUSY_RETRIES       3U

/* Task parameters (stack sizes are in 32-bit words) */
#define PRODUCER_STACK_WORDS    512U
#define LOGGER_STACK_WORDS      384U
#define PRODUCER_PRIORITY       3U
#define LOGGER_PRIORITY         2U      /* consumer below producer */

/* Event posted by the ISR */
typedef struct
{
  uint8_t  button_id;   /* 1 or 2 */
  uint32_t tick;        /* RTOS tick count when the press happened */
} ButtonEvent_t;

/* Message posted to the logger (copied by value into the queue) */
typedef struct
{
  uint16_t len;                     /* number of valid bytes in json[] */
  char     json[LOG_MSG_MAX_LEN];   /* JSON string, ends with \r\n */
} LogMsg_t;

/* The timestamp is the tick count, so 1 tick must equal 1 ms.
 * Set TICK_RATE_HZ = 1000 in CubeMX (FreeRTOS config) if this fails. */
_Static_assert(configTICK_RATE_HZ == 1000, "configTICK_RATE_HZ must be 1000 (1 tick = 1 ms)");

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static QueueHandle_t eventQueue = NULL;   /* ISR      -> Producer */
static QueueHandle_t logQueue   = NULL;   /* Producer -> Logger   */

/* Diagnostics: incremented when a queue is full and data had to be dropped */
static volatile uint32_t isr_dropped_events = 0U;
static volatile uint32_t log_dropped_msgs   = 0U;
static uint32_t          uart_error_count   = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
//void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void ButtonProducerTask(void *pvParameters);
static void UartLoggerTask(void *pvParameters);
static HAL_StatusTypeDef Uart_SendString(const char *str, uint16_t len);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Create the queues first so the ISR and tasks always find them ready.
   * Items are copied BY VALUE, so no shared buffers and no lifetime issues. */
  eventQueue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(ButtonEvent_t));
  logQueue   = xQueueCreate(LOG_QUEUE_LEN,   sizeof(LogMsg_t));
  if ((eventQueue == NULL) || (logQueue == NULL))
  {
    Error_Handler();   /* out of heap */
  }

  /* Create the two application tasks */
  if (xTaskCreate(ButtonProducerTask, "Producer", PRODUCER_STACK_WORDS,
                  NULL, PRODUCER_PRIORITY, NULL) != pdPASS)
  {
    Error_Handler();
  }
  if (xTaskCreate(UartLoggerTask, "Logger", LOGGER_STACK_WORDS,
                  NULL, LOGGER_PRIORITY, NULL) != pdPASS)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  EXTI callback, runs in INTERRUPT context for both switches.
  *
  * Rules followed here: no blocking, no printf, no UART, and only the
  * "...FromISR" FreeRTOS API. The ISR just stamps the event and hands it to
  * the Producer task through eventQueue.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static TickType_t last_tick[3] = {0U, 0U, 0U};   /* per-button debounce time */
  static uint8_t    seen[3]      = {0U, 0U, 0U};   /* has the button fired before? */

  BaseType_t    xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt;
  uint8_t       id;
  TickType_t    now;

  /* Map the EXTI pin to a button ID (ignore any other pin) */
  if (GPIO_Pin == B1_Pin)
  {
    id = BTN1_ID;
  }
  else if (GPIO_Pin == SW2_Pin)
  {
    id = BTN2_ID;
  }
  else
  {
    return;
  }

  /* EXTI can fire before the queues exist (very early after reset) */
  if (eventQueue == NULL)
  {
    return;
  }

  /* Capture the timestamp (RTOS tick count = ms since scheduler start) */
  now = xTaskGetTickCountFromISR();

  /* Debounce: drop edges that arrive too soon after the previous one */
  if (seen[id] && ((now - last_tick[id]) < pdMS_TO_TICKS(DEBOUNCE_MS)))
  {
    return;
  }
  seen[id]      = 1U;
  last_tick[id] = now;

  evt.button_id = id;
  evt.tick      = (uint32_t)now;

  /* Non-blocking post. If the queue is full the event is dropped and counted
   * (the ISR must never wait). */
  if (xQueueSendFromISR(eventQueue, &evt, &xHigherPriorityTaskWoken) != pdPASS)
  {
    isr_dropped_events++;
  }

  /* If the Producer was woken and has higher priority than the interrupted
   * task, switch to it immediately on exit from the ISR. */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  Task 1 / Event Producer.
  *         Blocks on eventQueue (0% CPU while idle), converts each event to a
  *         JSON string and posts it to logQueue.
  */
static void ButtonProducerTask(void *pvParameters)
{
  (void)pvParameters;

  ButtonEvent_t evt;
  LogMsg_t      msg;

  for (;;)
  {
    /* Wait for a button event, with a finite timeout */
    if (xQueueReceive(eventQueue, &evt, pdMS_TO_TICKS(PRODUCER_WAIT_MS)) == pdPASS)
    {
      /* Build the JSON payload, e.g. {"button_id": 1, "timestamp": 12500} */
      int len = snprintf(msg.json, sizeof(msg.json),
                         "{\"button_id\": %u, \"timestamp\": %lu}\r\n",
                         (unsigned int)evt.button_id,
                         (unsigned long)evt.tick);

      /* Reject formatting errors / truncated output */
      if ((len <= 0) || ((size_t)len >= sizeof(msg.json)))
      {
        continue;
      }
      msg.len = (uint16_t)len;

      /* Post to the logger. If the log queue is full, wait a short time
       * (QUEUE_POST_TIMEOUT_MS); if it is still full, drop the message and
       * count it instead of blocking forever. */
      if (xQueueSend(logQueue, &msg, pdMS_TO_TICKS(QUEUE_POST_TIMEOUT_MS)) != pdPASS)
      {
        log_dropped_msgs++;
      }
    }
    /* else: timeout, queue empty -> no buttons pressed, just wait again */
  }
}

/**
  * @brief  Task 2 / Consumer (UART logger).
  *         Sleeps until a message arrives in logQueue, then transmits it.
  *         This task is the only owner of USART2, so no mutex is needed.
  *         When the queue is empty it simply times out and loops, and it
  *         reports any dropped events/messages so overflow is visible.
  */
static void UartLoggerTask(void *pvParameters)
{
  (void)pvParameters;

  LogMsg_t msg;
  char     warn[LOG_MSG_MAX_LEN];
  uint32_t reported_isr = 0U;
  uint32_t reported_log = 0U;

  static const char banner[] = "{\"status\": \"logger_ready\"}\r\n";
  (void)Uart_SendString(banner, (uint16_t)(sizeof(banner) - 1U));

  for (;;)
  {
    /* Block (no CPU use) until a message arrives or the timeout expires */
    if (xQueueReceive(logQueue, &msg, pdMS_TO_TICKS(LOGGER_WAIT_MS)) == pdPASS)
    {
      if (Uart_SendString(msg.json, msg.len) != HAL_OK)
      {
        uart_error_count++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* visible UART fault */
      }
    }
    /* else: timeout, queue empty -> nothing to do, fall through */

    /* Report overflow once per change (queue-full edge case) */
    uint32_t isr_d = isr_dropped_events;
    uint32_t log_d = log_dropped_msgs;
    if ((isr_d != reported_isr) || (log_d != reported_log))
    {
      int len = snprintf(warn, sizeof(warn),
                         "{\"warning\": \"queue_full\", \"dropped_isr\": %lu, \"dropped_log\": %lu}\r\n",
                         (unsigned long)isr_d, (unsigned long)log_d);
      if ((len > 0) && ((size_t)len < sizeof(warn)))
      {
        (void)Uart_SendString(warn, (uint16_t)len);
      }
      reported_isr = isr_d;
      reported_log = log_d;
    }
  }
}

/**
  * @brief  Transmit a string over USART2 with a finite timeout.
  *         Retries a few times if the UART is busy.
  * @retval HAL status of the last attempt
  */
static HAL_StatusTypeDef Uart_SendString(const char *str, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  for (uint8_t attempt = 0U; attempt < UART_BUSY_RETRIES; attempt++)
  {
    status = HAL_UART_Transmit(&huart2, (uint8_t *)str, len, UART_TX_TIMEOUT_MS);
    if (status != HAL_BUSY)
    {
      break;                                 /* HAL_OK or a real error */
    }
    vTaskDelay(pdMS_TO_TICKS(2U));           /* UART busy: yield and retry */
  }
  return status;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
