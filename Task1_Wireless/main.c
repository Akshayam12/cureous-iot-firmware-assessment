/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 host + ESP32 AT-command Wi-Fi/MQTT modem
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
  *
  * OPTION B - Cloud connectivity via AT commands (STM32 host + ESP32 modem)
  * ------------------------------------------------------------------------
  *  USART2 (ST-Link virtual COM, 115200 8N1) : serial terminal / provisioning
  *  USART1 (115200 8N1)                      : UART link to ESP32 (ESP-AT fw)
  *
  *  Wiring (Nucleo-64):  PA9  (USART1_TX, D8) -> ESP32 AT-RX (default GPIO16)
  *                       PA10 (USART1_RX, D2) <- ESP32 AT-TX (default GPIO17)
  *                       GND                  -- ESP32 GND
  *
  *  Features
  *   - Dynamic provisioning of Wi-Fi SSID / password (and MQTT broker) from the
  *     serial terminal - nothing is hard-coded. Press B1 or type "reprov" at
  *     any time to provision again.
  *   - ESP32 configured with AT+CWMODE=1 and joined with AT+CWJAP.
  *   - MQTT over AT (AT+MQTTUSERCFG / MQTTCONNCFG / MQTTCONN / MQTTSUB).
  *   - Heartbeat task publishes a "keep-alive" message every 30 s.
  *   - Interrupt-driven UART Rx + dedicated parser task monitor the ESP32
  *     continuously for incoming cloud commands (+MQTTSUBRECV) and link events.
  *   - Automatic reconnect with exponential back-off, LWT "offline" message.
  *
  *  Cloud commands (publish to  stm32esp/<device-id>/cmd ):
  *     LED_ON | LED_OFF | LED_TOGGLE | PING | STATUS
  *  Replies are published to    stm32esp/<device-id>/resp
  *
  *  REQUIRED CubeMX / project settings (see also the notes at the end)
  *   - NVIC: enable "USART1 global interrupt", "USART2 global interrupt" and
  *           "EXTI line[15:10] interrupts" (B1 button).
  *   - FreeRTOS: configTOTAL_HEAP_SIZE >= 24576 bytes.
  *   - FreeRTOS: configUSE_TIMERS = 1 (needed for event flags from an ISR).
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  AT_OK = 0,
  AT_ERROR,
  AT_TIMEOUT,
  AT_BUSY,
  AT_BAD_PARAM
} AtStatus_t;

typedef enum {
  SESS_OK = 0,
  SESS_RETRY,
  SESS_BAD_CREDS,
  SESS_ESP_DEAD
} SessResult_t;

typedef enum {
  MON_REPROV = 0,
  MON_LINK_LOST
} MonResult_t;

typedef struct {
  char     ssid[33];     /* WIFI_SSID_MAX + 1 */
  char     pass[64];     /* WIFI_PASS_MAX + 1 */
  char     host[65];     /* MQTT_HOST_MAX + 1 */
  uint16_t port;
} Provision_t;

typedef struct {
  char text[48];
} CloudCmd_t;

typedef struct {
  char     buf[80];
  uint16_t len;
  uint8_t  lastCR;
} LineEditor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Debug ---- */
#define AT_TRACE                1       /* 1 = print AT traffic on terminal (passwords hidden) */

/* ---- Limits ---- */
#define AT_MAX_CMD_LEN          256U    /* ESP-AT limit for one command line */
#define AT_RESP_BUF_LEN         192U
#define ESP_LINE_MAX            384U
#define CONSOLE_LINE_MAX        80U
#define WIFI_SSID_MAX           32U
#define WIFI_PASS_MAX           63U
#define MQTT_HOST_MAX           64U
#define CMD_TEXT_MAX            48U

/* ---- Timing (ms) ---- */
#define AT_MUTEX_TIMEOUT_MS     30000U
#define ESP_BOOT_TIMEOUT_MS     8000U
#define WIFI_JOIN_TIMEOUT_MS    25000U
#define MQTT_CONN_TIMEOUT_MS    20000U
#define HEARTBEAT_PERIOD_MS     30000U  /* requirement: 30 s keep-alive */
#define HEARTBEAT_MAX_FAILS     3U
#define BACKOFF_MIN_MS          2000U
#define BACKOFF_MAX_MS          30000U
#define WIFI_MAX_JOIN_FAILS     3U

/* ---- MQTT ---- */
#define MQTT_DEFAULT_HOST       "broker.emqx.io"
#define MQTT_DEFAULT_PORT       "1883"
#define MQTT_KEEPALIVE_S        60U
#define MQTT_LINK_ID            0

/* ---- Interrupt priority (must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) ---- */
#define APP_IRQ_PRIO            5U

/* ---- Event flags: system level ---- */
#define EVT_WIFI_LOST           (1UL << 0)
#define EVT_MQTT_LOST           (1UL << 1)
#define EVT_REPROV              (1UL << 2)
#define EVT_ALL                 (EVT_WIFI_LOST | EVT_MQTT_LOST | EVT_REPROV)

/* ---- Event flags: AT response ---- */
#define AT_EVT_OK               (1UL << 0)
#define AT_EVT_ERR              (1UL << 1)
#define AT_EVT_READY            (1UL << 2)

/* ms -> RTOS ticks */
#define MS2T(ms)                (((uint32_t)(ms) * osKernelGetTickFreq()) / 1000U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* RTOS objects */
static osMutexId_t         atMutexHandle;
static osMutexId_t         respMutexHandle;
static osMutexId_t         logMutexHandle;
static osMessageQueueId_t  espRxQueueHandle;
static osMessageQueueId_t  consoleRxQueueHandle;
static osMessageQueueId_t  cmdQueueHandle;
static osEventFlagsId_t    sysEventsHandle;
static osEventFlagsId_t    atEventsHandle;
static osThreadId_t        espRxTaskHandle;
static osThreadId_t        heartbeatTaskHandle;
static osThreadId_t        cmdTaskHandle;

static const osMutexAttr_t atMutex_attr   = { .name = "atMutex",   .attr_bits = osMutexPrioInherit };
static const osMutexAttr_t respMutex_attr = { .name = "respMutex", .attr_bits = osMutexPrioInherit };
static const osMutexAttr_t logMutex_attr  = { .name = "logMutex",  .attr_bits = osMutexPrioInherit };

static const osThreadAttr_t espRxTask_attr = {
  .name = "espRx", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityAboveNormal,
};
static const osThreadAttr_t heartbeatTask_attr = {
  .name = "heartbeat", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal,
};
static const osThreadAttr_t cmdTask_attr = {
  .name = "cloudCmd", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal,
};

/* UART single-byte receive buffers (interrupt driven) */
static uint8_t espRxByte;
static uint8_t consoleRxByte;
static volatile uint32_t espRxDropped;

/* Link state shared between tasks */
static volatile bool wifiUp;
static volatile bool mqttUp;
static volatile bool atPending;
static volatile uint32_t hbCount;
static volatile uint32_t cmdCount;

/* AT engine buffers (protected by atMutex / respMutex / logMutex) */
static char   atTxBuf[AT_MAX_CMD_LEN + 3U];
static char   respBuf[AT_RESP_BUF_LEN];
static size_t respLen;
static char   logBuf[320];

/* Provisioned configuration (RAM only - entered at run time) */
static Provision_t cfg;
static bool        credsVerified;

/* Device identity / topics (built at start-up) */
static char deviceId[24];
static char topicHb[64];
static char topicCmd[64];
static char topicResp[64];
static char topicStatus[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void Console_Out(bool stamp, const char *fmt, va_list ap);
static void Log(const char *fmt, ...);
static void Console_Printf(const char *fmt, ...);
static bool Console_Feed(LineEditor_t *ed, uint8_t c, bool mask, uint16_t maxLen);
static void Console_ReadLineBlocking(char *out, size_t outSize, bool mask);
static bool Console_Poll(uint32_t timeoutMs, char *out, size_t outSize);
static void Console_Prompt(const char *label, const char *def, bool required,
                           bool mask, char *out, size_t outSize);
static void Console_Command(const char *line);

static bool AT_Escape(const char *in, char *out, size_t outSize);
static AtStatus_t AT_Command(uint32_t timeoutMs, char *respOut, size_t respSize,
                             const char *fmt, ...);
static void ESP_ProcessLine(char *line);
static void ESP_HandleMqttRx(const char *line);
static bool ESP_Init(void);
static SessResult_t WiFi_Connect(void);
static SessResult_t MQTT_Connect(void);
static SessResult_t Session_Establish(void);
static bool MQTT_Publish(const char *topic, const char *payload, int qos, int retain);

static void Provision_Run(void);
static MonResult_t Monitor_Loop(void);
static void App_BuildIdentity(void);

static void ESP_RxTask(void *argument);
static void Heartbeat_Task(void *argument);
static void Cmd_Task(void *argument);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Interrupt priorities must be FreeRTOS-safe because the UART/EXTI callbacks
     use the RTOS "FromISR" APIs. (Interrupts only fire once reception is armed
     from inside the tasks, i.e. after the scheduler is running.) */
  HAL_NVIC_SetPriority(USART1_IRQn, APP_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, APP_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, APP_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  atMutexHandle   = osMutexNew(&atMutex_attr);
  respMutexHandle = osMutexNew(&respMutex_attr);
  logMutexHandle  = osMutexNew(&logMutex_attr);
  if ((atMutexHandle == NULL) || (respMutexHandle == NULL) || (logMutexHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  espRxQueueHandle     = osMessageQueueNew(1024, sizeof(uint8_t),     NULL);
  consoleRxQueueHandle = osMessageQueueNew(64,   sizeof(uint8_t),     NULL);
  cmdQueueHandle       = osMessageQueueNew(4,    sizeof(CloudCmd_t),  NULL);
  if ((espRxQueueHandle == NULL) || (consoleRxQueueHandle == NULL) || (cmdQueueHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  espRxTaskHandle     = osThreadNew(ESP_RxTask,       NULL, &espRxTask_attr);
  heartbeatTaskHandle = osThreadNew(Heartbeat_Task,   NULL, &heartbeatTask_attr);
  cmdTaskHandle       = osThreadNew(Cmd_Task,         NULL, &cmdTask_attr);
  if ((defaultTaskHandle == NULL) || (espRxTaskHandle == NULL) ||
      (heartbeatTaskHandle == NULL) || (cmdTaskHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  sysEventsHandle = osEventFlagsNew(NULL);
  atEventsHandle  = osEventFlagsNew(NULL);
  if ((sysEventsHandle == NULL) || (atEventsHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_EVENTS */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

/* ===========================================================================
 *  HAL callbacks (ISR context)
 * ========================================================================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)            /* byte from ESP32 */
  {
    if (osMessageQueuePut(espRxQueueHandle, &espRxByte, 0, 0) != osOK)
    {
      espRxDropped++;
    }
    (void)HAL_UART_Receive_IT(&huart1, &espRxByte, 1);
  }
  else if (huart->Instance == USART2)       /* byte from serial terminal */
  {
    (void)osMessageQueuePut(consoleRxQueueHandle, &consoleRxByte, 0, 0);
    (void)HAL_UART_Receive_IT(&huart2, &consoleRxByte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Overrun / framing / noise: make sure reception keeps running.
     (Returns HAL_BUSY, harmlessly, if the HAL already kept it running.) */
  if (huart->Instance == USART1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &espRxByte, 1);
  }
  else if (huart->Instance == USART2)
  {
    (void)HAL_UART_Receive_IT(&huart2, &consoleRxByte, 1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t lastPress = 0;

  if (GPIO_Pin == B1_Pin)
  {
    uint32_t now = HAL_GetTick();
    if (((now - lastPress) > 300U) && (sysEventsHandle != NULL) &&
        (osKernelGetState() == osKernelRunning))
    {
      lastPress = now;
      (void)osEventFlagsSet(sysEventsHandle, EVT_REPROV);   /* B1 = re-provision */
    }
  }
}

/* ===========================================================================
 *  Console (USART2) helpers
 * ========================================================================= */

static void Console_Out(bool stamp, const char *fmt, va_list ap)
{
  if (osMutexAcquire(logMutexHandle, MS2T(1000)) != osOK)
  {
    return;
  }

  int n = 0;
  if (stamp)
  {
    unsigned long t = (unsigned long)HAL_GetTick();
    n = snprintf(logBuf, sizeof(logBuf), "[%5lu.%03lu] ", t / 1000UL, t % 1000UL);
  }

  size_t cap = sizeof(logBuf) - (size_t)n - 3U;           /* room for "\r\n" + NUL */
  int m = vsnprintf(&logBuf[n], cap, fmt, ap);
  if (m < 0)
  {
    m = 0;
  }
  if ((size_t)m >= cap)
  {
    m = (int)cap - 1;                                      /* truncated */
  }
  n += m;

  if (stamp)
  {
    logBuf[n++] = '\r';
    logBuf[n++] = '\n';
  }

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)logBuf, (uint16_t)n, 500);
  (void)osMutexRelease(logMutexHandle);
}

/* Time-stamped log line (adds CRLF) */
static void Log(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  Console_Out(true, fmt, ap);
  va_end(ap);
}

/* Raw output (no time stamp, no automatic newline) */
static void Console_Printf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  Console_Out(false, fmt, ap);
  va_end(ap);
}

/* Feed one received character into a line editor. Returns true when a full
   line is available in ed->buf (NUL-terminated). Handles echo + backspace. */
static bool Console_Feed(LineEditor_t *ed, uint8_t c, bool mask, uint16_t maxLen)
{
  if ((c == '\r') || (c == '\n'))
  {
    if ((c == '\n') && ed->lastCR)          /* second half of CRLF */
    {
      ed->lastCR = 0;
      return false;
    }
    ed->lastCR = (c == '\r') ? 1U : 0U;
    ed->buf[ed->len] = '\0';
    ed->len = 0;
    Console_Printf("\r\n");
    return true;
  }

  ed->lastCR = 0;

  if ((c == 0x08U) || (c == 0x7FU))         /* backspace / delete */
  {
    if (ed->len > 0U)
    {
      ed->len--;
      Console_Printf("\b \b");
    }
    return false;
  }

  if ((c >= 0x20U) && (c < 0x7FU) && (ed->len < maxLen))
  {
    ed->buf[ed->len++] = (char)c;
    Console_Printf("%c", mask ? '*' : (char)c);
  }
  return false;
}

/* Blocking line read (used for provisioning prompts) */
static void Console_ReadLineBlocking(char *out, size_t outSize, bool mask)
{
  LineEditor_t ed;
  uint8_t c;
  uint16_t maxLen = (uint16_t)((outSize - 1U) < (CONSOLE_LINE_MAX - 1U) ? (outSize - 1U)
                                                                         : (CONSOLE_LINE_MAX - 1U));
  memset(&ed, 0, sizeof(ed));
  (void)osMessageQueueReset(consoleRxQueueHandle);

  for (;;)
  {
    if (osMessageQueueGet(consoleRxQueueHandle, &c, NULL, osWaitForever) == osOK)
    {
      if (Console_Feed(&ed, c, mask, maxLen))
      {
        strncpy(out, ed.buf, outSize - 1U);
        out[outSize - 1U] = '\0';
        return;
      }
    }
  }
}

/* Non-blocking-ish poll used by the monitor loop. true = a full line arrived */
static bool Console_Poll(uint32_t timeoutMs, char *out, size_t outSize)
{
  static LineEditor_t ed;
  uint8_t c;

  if (osMessageQueueGet(consoleRxQueueHandle, &c, NULL, MS2T(timeoutMs)) != osOK)
  {
    return false;
  }
  if (Console_Feed(&ed, c, false, CONSOLE_LINE_MAX - 1U))
  {
    strncpy(out, ed.buf, outSize - 1U);
    out[outSize - 1U] = '\0';
    return true;
  }
  return false;
}

static void Console_Prompt(const char *label, const char *def, bool required,
                           bool mask, char *out, size_t outSize)
{
  for (;;)
  {
    if ((def != NULL) && (def[0] != '\0'))
    {
      Console_Printf("  %s [%s]: ", label, def);
    }
    else
    {
      Console_Printf("  %s: ", label);
    }

    Console_ReadLineBlocking(out, outSize, mask);

    if (out[0] == '\0')
    {
      if ((def != NULL) && (def[0] != '\0'))
      {
        strncpy(out, def, outSize - 1U);
        out[outSize - 1U] = '\0';
        return;
      }
      if (!required)
      {
        return;
      }
      Console_Printf("  -> a value is required\r\n");
      continue;
    }
    return;
  }
}

/* ===========================================================================
 *  Application identity
 * ========================================================================= */

static void App_BuildIdentity(void)
{
  uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

  snprintf(deviceId,    sizeof(deviceId),    "stm32-%08lX", (unsigned long)uid);
  snprintf(topicHb,     sizeof(topicHb),     "stm32esp/%s/heartbeat", deviceId);
  snprintf(topicCmd,    sizeof(topicCmd),    "stm32esp/%s/cmd",       deviceId);
  snprintf(topicResp,   sizeof(topicResp),   "stm32esp/%s/resp",      deviceId);
  snprintf(topicStatus, sizeof(topicStatus), "stm32esp/%s/status",    deviceId);
}

/* ===========================================================================
 *  AT command engine
 *   - one command in flight at a time (atMutex)
 *   - ESP_RxTask classifies every line coming from the ESP32:
 *       * unsolicited (URC) lines  -> handled immediately
 *       * response lines           -> collected, OK/ERROR wakes the sender
 * ========================================================================= */

/* Escape , " and \ in a string parameter (required by ESP-AT) */
static bool AT_Escape(const char *in, char *out, size_t outSize)
{
  size_t j = 0;

  for (; *in != '\0'; in++)
  {
    if ((*in == '"') || (*in == ',') || (*in == '\\'))
    {
      if ((j + 2U) >= outSize)
      {
        return false;
      }
      out[j++] = '\\';
    }
    else if ((j + 1U) >= outSize)
    {
      return false;
    }
    out[j++] = *in;
  }
  out[j] = '\0';
  return true;
}

/* Send one AT command and wait for OK / ERROR / FAIL.
   If respOut != NULL the response lines are copied there. */
static AtStatus_t AT_Command(uint32_t timeoutMs, char *respOut, size_t respSize,
                             const char *fmt, ...)
{
  AtStatus_t status;
  va_list ap;
  int n;

  if ((respOut != NULL) && (respSize > 0U))
  {
    respOut[0] = '\0';
  }

  if (osMutexAcquire(atMutexHandle, MS2T(AT_MUTEX_TIMEOUT_MS)) != osOK)
  {
    return AT_BUSY;
  }

  va_start(ap, fmt);
  n = vsnprintf(atTxBuf, sizeof(atTxBuf), fmt, ap);
  va_end(ap);

  if ((n <= 0) || ((size_t)n > AT_MAX_CMD_LEN))
  {
    Log("AT: command too long / invalid");
    (void)osMutexRelease(atMutexHandle);
    return AT_BAD_PARAM;
  }

#if AT_TRACE
  if (strstr(atTxBuf, "CWJAP") != NULL)
  {
    Log(">> AT+CWJAP=<hidden>");
  }
  else
  {
    Log(">> %s", atTxBuf);
  }
#endif

  atTxBuf[n]     = '\r';
  atTxBuf[n + 1] = '\n';

  (void)osEventFlagsClear(atEventsHandle, AT_EVT_OK | AT_EVT_ERR);

  if (osMutexAcquire(respMutexHandle, MS2T(100)) == osOK)
  {
    respLen = 0;
    respBuf[0] = '\0';
    (void)osMutexRelease(respMutexHandle);
  }
  atPending = true;

  if (HAL_UART_Transmit(&huart1, (uint8_t *)atTxBuf, (uint16_t)(n + 2), 1000) != HAL_OK)
  {
    atPending = false;
    (void)osMutexRelease(atMutexHandle);
    return AT_ERROR;
  }

  uint32_t f = osEventFlagsWait(atEventsHandle, AT_EVT_OK | AT_EVT_ERR,
                                osFlagsWaitAny, MS2T(timeoutMs));
  atPending = false;

  if ((f & osFlagsError) != 0U)
  {
    status = AT_TIMEOUT;
  }
  else if ((f & AT_EVT_ERR) != 0U)
  {
    status = AT_ERROR;
  }
  else
  {
    status = AT_OK;
  }

  if ((respOut != NULL) && (respSize > 0U) &&
      (osMutexAcquire(respMutexHandle, MS2T(100)) == osOK))
  {
    strncpy(respOut, respBuf, respSize - 1U);
    respOut[respSize - 1U] = '\0';
    (void)osMutexRelease(respMutexHandle);
  }

  (void)osMutexRelease(atMutexHandle);

  if (status == AT_TIMEOUT)
  {
    Log("AT: timeout waiting for response");
  }
  return status;
}

/* ===========================================================================
 *  ESP32 receive path: asynchronous monitoring of UART Rx
 * ========================================================================= */

/* Parse   +MQTTSUBRECV:<id>,"<topic>",<len>,<data>   and queue the command */
static void ESP_HandleMqttRx(const char *line)
{
  const char *p = strchr(line, '"');
  if (p == NULL) { return; }
  const char *q = strchr(p + 1, '"');
  if ((q == NULL) || (q[1] != ',')) { return; }

  char *end;
  long len = strtol(q + 2, &end, 10);
  if ((end == (q + 2)) || (*end != ',') || (len < 0)) { return; }

  const char *data = end + 1;
  while (*data == ' ') { data++; }

  CloudCmd_t cmd;
  size_t n = strlen(data);
  if ((long)n > len)                { n = (size_t)len; }
  if (n >= sizeof(cmd.text))        { n = sizeof(cmd.text) - 1U; }
  memcpy(cmd.text, data, n);
  cmd.text[n] = '\0';

  while ((n > 0U) && ((cmd.text[n - 1U] == ' ') || (cmd.text[n - 1U] == '\r') ||
                      (cmd.text[n - 1U] == '\n')))
  {
    cmd.text[--n] = '\0';
  }
  if (n == 0U) { return; }

  if (osMessageQueuePut(cmdQueueHandle, &cmd, 0, 0) != osOK)
  {
    Log("Cloud command queue full - command dropped");
  }
}

static void ESP_ProcessLine(char *line)
{
#if AT_TRACE
  Log("<< %s", line);
#endif

  /* ---- Unsolicited result codes (can arrive at any time) ---- */
  if (strncmp(line, "+MQTTSUBRECV:", 13) == 0)
  {
    ESP_HandleMqttRx(line);
    return;
  }
  if (strncmp(line, "+MQTTDISCONNECTED:", 18) == 0)
  {
    mqttUp = false;
    (void)osEventFlagsSet(sysEventsHandle, EVT_MQTT_LOST);
    return;
  }
  if (strcmp(line, "WIFI DISCONNECT") == 0)
  {
    wifiUp = false;
    (void)osEventFlagsSet(sysEventsHandle, EVT_WIFI_LOST);
    return;
  }
  if (strcmp(line, "ready") == 0)
  {
    (void)osEventFlagsSet(atEventsHandle, AT_EVT_READY);
    return;
  }

  /* ---- Response to the command currently in flight ---- */
  if (atPending)
  {
    if (osMutexAcquire(respMutexHandle, MS2T(20)) == osOK)
    {
      size_t l = strlen(line);
      if ((respLen + l + 2U) < sizeof(respBuf))
      {
        memcpy(&respBuf[respLen], line, l);
        respLen += l;
        respBuf[respLen++] = '\n';
        respBuf[respLen]   = '\0';
      }
      (void)osMutexRelease(respMutexHandle);
    }

    if (strcmp(line, "OK") == 0)
    {
      (void)osEventFlagsSet(atEventsHandle, AT_EVT_OK);
    }
    else if ((strcmp(line, "ERROR") == 0) || (strcmp(line, "FAIL") == 0) ||
             (strcmp(line, "SEND FAIL") == 0))
    {
      (void)osEventFlagsSet(atEventsHandle, AT_EVT_ERR);
    }
  }
}

static void ESP_RxTask(void *argument)
{
  static char line[ESP_LINE_MAX];
  size_t len = 0;
  uint8_t c;

  (void)argument;
  (void)HAL_UART_Receive_IT(&huart1, &espRxByte, 1);

  for (;;)
  {
    if (osMessageQueueGet(espRxQueueHandle, &c, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    if (c == '\n')
    {
      line[len] = '\0';
      if (len > 0U)
      {
        ESP_ProcessLine(line);
      }
      len = 0;
    }
    else if (c == '\r')
    {
      /* ignore */
    }
    else if (len < (ESP_LINE_MAX - 1U))
    {
      line[len++] = (char)c;
    }
    /* else: line too long -> extra characters dropped */
  }
}

/* ===========================================================================
 *  ESP32 bring-up, Wi-Fi and MQTT
 * ========================================================================= */

static SessResult_t MapStatus(AtStatus_t st)
{
  return (st == AT_TIMEOUT) ? SESS_ESP_DEAD : SESS_RETRY;
}

/* Sync, reset and configure the ESP32 (AT, AT+RST, ATE0, AT+CWMODE=1) */
static bool ESP_Init(void)
{
  bool alive = false;

  Log("Initialising ESP32 AT modem ...");

  for (int i = 0; (i < 5) && !alive; i++)
  {
    alive = (AT_Command(1000, NULL, 0, "AT") == AT_OK);
  }
  if (!alive)
  {
    Log("ESP32 does not answer 'AT'. Check wiring (PA9->ESP RX, PA10<-ESP TX, GND), "
        "ESP-AT firmware and 115200 baud.");
    return false;
  }

  /* Software reset for a known state, wait for the "ready" banner */
  (void)osEventFlagsClear(atEventsHandle, AT_EVT_READY);
  if (AT_Command(2000, NULL, 0, "AT+RST") == AT_OK)
  {
    uint32_t f = osEventFlagsWait(atEventsHandle, AT_EVT_READY, osFlagsWaitAny,
                                  MS2T(ESP_BOOT_TIMEOUT_MS));
    if ((f & osFlagsError) != 0U)
    {
      Log("ESP32 did not report 'ready' after reset - continuing");
    }
  }
  osDelay(MS2T(500));

  alive = false;
  for (int i = 0; (i < 10) && !alive; i++)
  {
    alive = (AT_Command(1000, NULL, 0, "AT") == AT_OK);
  }
  if (!alive)
  {
    Log("ESP32 did not come back after reset");
    return false;
  }

  (void)AT_Command(1000, NULL, 0, "ATE0");                 /* echo off */

  if (AT_Command(2000, NULL, 0, "AT+CWMODE=1") != AT_OK)  /* station mode */
  {
    Log("AT+CWMODE=1 failed");
    return false;
  }

  Log("ESP32 ready (station mode)");
  return true;
}

static SessResult_t WiFi_Connect(void)
{
  static uint8_t joinFails = 0;
  char ssidEsc[2U * WIFI_SSID_MAX + 1U];
  char passEsc[2U * WIFI_PASS_MAX + 1U];
  char resp[AT_RESP_BUF_LEN];

  if (!AT_Escape(cfg.ssid, ssidEsc, sizeof(ssidEsc)) ||
      !AT_Escape(cfg.pass, passEsc, sizeof(passEsc)))
  {
    return SESS_BAD_CREDS;
  }

  Log("Joining Wi-Fi \"%s\" ...", cfg.ssid);
  AtStatus_t st = AT_Command(WIFI_JOIN_TIMEOUT_MS, resp, sizeof(resp),
                             "AT+CWJAP=\"%s\",\"%s\"", ssidEsc, passEsc);

  if (st == AT_OK)
  {
    joinFails = 0;
    credsVerified = true;
    wifiUp = true;

    if (AT_Command(2000, resp, sizeof(resp), "AT+CIPSTA?") == AT_OK)
    {
      const char *ip = strstr(resp, "ip:\"");
      if (ip != NULL)
      {
        ip += 4;
        const char *e = strchr(ip, '"');
        if (e != NULL)
        {
          Log("Wi-Fi connected, IP address %.*s", (int)(e - ip), ip);
        }
      }
    }
    return SESS_OK;
  }

  if (st == AT_TIMEOUT)
  {
    return SESS_ESP_DEAD;
  }
  if (st != AT_ERROR)
  {
    return SESS_RETRY;
  }

  /* AT+CWJAP failed: "+CWJAP:<code>" then "FAIL" */
  const char *p = strstr(resp, "+CWJAP:");
  int code = (p != NULL) ? atoi(p + 7) : 0;
  const char *why;
  switch (code)
  {
    case 1:  why = "connection timeout";            break;
    case 2:  why = "wrong password";                break;
    case 3:  why = "access point not found";        break;
    case 4:  why = "connection failed";             break;
    default: why = "unknown error";                 break;
  }
  Log("Wi-Fi join failed: %s (code %d)", why, code);

  if (credsVerified)
  {
    return SESS_RETRY;               /* credentials worked before: keep retrying */
  }
  if ((code == 2) || (code == 3) || (++joinFails >= WIFI_MAX_JOIN_FAILS))
  {
    joinFails = 0;
    return SESS_BAD_CREDS;           /* ask the user for new credentials */
  }
  return SESS_RETRY;
}

static bool MQTT_Publish(const char *topic, const char *payload, int qos, int retain)
{
  char esc[2U * 100U + 1U];

  if (!AT_Escape(payload, esc, sizeof(esc)))
  {
    return false;
  }
  return (AT_Command(5000, NULL, 0, "AT+MQTTPUB=%d,\"%s\",\"%s\",%d,%d",
                     MQTT_LINK_ID, topic, esc, qos, retain) == AT_OK);
}

static SessResult_t MQTT_Connect(void)
{
  char hostEsc[2U * MQTT_HOST_MAX + 1U];
  AtStatus_t st;

  if (!AT_Escape(cfg.host, hostEsc, sizeof(hostEsc)))
  {
    return SESS_RETRY;
  }

  Log("Connecting to MQTT broker %s:%u as %s ...", cfg.host, (unsigned)cfg.port, deviceId);

  /* scheme 1 = MQTT over TCP */
  st = AT_Command(3000, NULL, 0, "AT+MQTTUSERCFG=%d,1,\"%s\",\"\",\"\",0,0,\"\"",
                  MQTT_LINK_ID, deviceId);
  if (st != AT_OK) { return MapStatus(st); }

  /* keep-alive + Last-Will ("offline", retained) */
  st = AT_Command(3000, NULL, 0, "AT+MQTTCONNCFG=%d,%u,0,\"%s\",\"offline\",1,1",
                  MQTT_LINK_ID, (unsigned)MQTT_KEEPALIVE_S, topicStatus);
  if (st != AT_OK) { return MapStatus(st); }

  st = AT_Command(MQTT_CONN_TIMEOUT_MS, NULL, 0, "AT+MQTTCONN=%d,\"%s\",%u,0",
                  MQTT_LINK_ID, hostEsc, (unsigned)cfg.port);
  if (st != AT_OK)
  {
    Log("MQTT connect failed");
    return MapStatus(st);
  }

  st = AT_Command(5000, NULL, 0, "AT+MQTTSUB=%d,\"%s\",1", MQTT_LINK_ID, topicCmd);
  if (st != AT_OK)
  {
    Log("MQTT subscribe failed");
    return MapStatus(st);
  }

  (void)MQTT_Publish(topicStatus, "online", 1, 1);

  mqttUp = true;
  Log("MQTT connected. Subscribed to %s", topicCmd);
  return SESS_OK;
}

static SessResult_t Session_Establish(void)
{
  SessResult_t r;

  (void)AT_Command(3000, NULL, 0, "AT+MQTTCLEAN=%d", MQTT_LINK_ID);  /* ignore result */

  r = WiFi_Connect();
  if (r != SESS_OK)
  {
    return r;
  }

  /* drop stale link events produced while (re)joining */
  (void)osEventFlagsClear(sysEventsHandle, EVT_WIFI_LOST | EVT_MQTT_LOST);

  return MQTT_Connect();
}

/* ===========================================================================
 *  Provisioning menu (serial terminal)
 * ========================================================================= */

static void Provision_Run(void)
{
  char tmp[16];
  char *e;

  (void)osEventFlagsClear(sysEventsHandle, EVT_REPROV);
  credsVerified = false;

  Console_Printf("\r\n==============================================\r\n");
  Console_Printf("   Wi-Fi / Cloud provisioning\r\n");
  Console_Printf("==============================================\r\n");

  for (;;)
  {
    Console_Prompt("Wi-Fi SSID", NULL, true, false, cfg.ssid, sizeof(cfg.ssid));
    Console_Prompt("Wi-Fi password (empty = open network)", NULL, false, true,
                   cfg.pass, sizeof(cfg.pass));
    Console_Prompt("MQTT broker host", MQTT_DEFAULT_HOST, false, false,
                   cfg.host, sizeof(cfg.host));

    for (;;)
    {
      Console_Prompt("MQTT broker port", MQTT_DEFAULT_PORT, false, false, tmp, sizeof(tmp));
      long p = strtol(tmp, &e, 10);
      if ((*e == '\0') && (p > 0L) && (p < 65536L))
      {
        cfg.port = (uint16_t)p;
        break;
      }
      Console_Printf("  -> invalid port number\r\n");
    }

    Console_Printf("\r\n  SSID     : %s\r\n", cfg.ssid);
    Console_Printf("  Password : %s (%u chars)\r\n",
                   (cfg.pass[0] != '\0') ? "********" : "<none>", (unsigned)strlen(cfg.pass));
    Console_Printf("  Broker   : %s:%u\r\n\r\n", cfg.host, (unsigned)cfg.port);

    Console_Prompt("Use these settings? (Y/n)", "y", false, false, tmp, sizeof(tmp));
    if ((tmp[0] == 'y') || (tmp[0] == 'Y'))
    {
      break;
    }
  }
  Console_Printf("\r\n");
}

/* ===========================================================================
 *  Runtime console commands
 * ========================================================================= */

static void Console_Command(const char *line)
{
  if (line[0] == '\0')
  {
    return;
  }

  if (strcasecmp(line, "help") == 0)
  {
    Console_Printf("Commands:\r\n"
                   "  help    - this text\r\n"
                   "  status  - show link / counters\r\n"
                   "  reprov  - enter new Wi-Fi credentials (same as button B1)\r\n"
                   "  reset   - reboot the STM32\r\n");
  }
  else if (strcasecmp(line, "status") == 0)
  {
    uint32_t up = osKernelGetTickCount() / osKernelGetTickFreq();
    Console_Printf("Device    : %s\r\n"
                   "Uptime    : %lu s\r\n"
                   "Wi-Fi     : %s (%s)\r\n"
                   "MQTT      : %s (%s:%u)\r\n"
                   "Heartbeats: %lu sent\r\n"
                   "Commands  : %lu received\r\n"
                   "ESP Rx drops: %lu bytes\r\n",
                   deviceId, (unsigned long)up,
                   wifiUp ? "up" : "down", cfg.ssid,
                   mqttUp ? "up" : "down", cfg.host, (unsigned)cfg.port,
                   (unsigned long)hbCount, (unsigned long)cmdCount,
                   (unsigned long)espRxDropped);
  }
  else if (strcasecmp(line, "reprov") == 0)
  {
    (void)osEventFlagsSet(sysEventsHandle, EVT_REPROV);
  }
  else if (strcasecmp(line, "reset") == 0)
  {
    Log("Rebooting ...");
    osDelay(MS2T(50));
    NVIC_SystemReset();
  }
  else
  {
    Console_Printf("Unknown command '%s' - type 'help'\r\n", line);
  }
}

/* Stay here while the link is healthy; leave on link loss or re-provision */
static MonResult_t Monitor_Loop(void)
{
  char line[CONSOLE_LINE_MAX];

  Console_Printf("\r\nOnline. Type 'help' for commands, press B1 to re-provision.\r\n");

  for (;;)
  {
    if (Console_Poll(100, line, sizeof(line)))
    {
      Console_Command(line);
    }

    uint32_t ev = osEventFlagsWait(sysEventsHandle, EVT_ALL, osFlagsWaitAny, 0);
    if ((ev & osFlagsError) == 0U)
    {
      if ((ev & EVT_REPROV) != 0U)
      {
        return MON_REPROV;
      }
      if ((ev & (EVT_WIFI_LOST | EVT_MQTT_LOST)) != 0U)
      {
        return MON_LINK_LOST;
      }
    }
  }
}

/* ===========================================================================
 *  Heartbeat task  -  "keep-alive" message every 30 seconds
 * ========================================================================= */

static void Heartbeat_Task(void *argument)
{
  char payload[96];
  uint32_t seq = 0;
  uint8_t fails = 0;
  uint32_t period = MS2T(HEARTBEAT_PERIOD_MS);
  uint32_t next = osKernelGetTickCount();

  (void)argument;

  for (;;)
  {
    next += period;
    (void)osDelayUntil(next);                 /* drift-free 30 s period */

    if (!mqttUp)
    {
      fails = 0;
      continue;
    }

    uint32_t up = osKernelGetTickCount() / osKernelGetTickFreq();
    snprintf(payload, sizeof(payload), "HEARTBEAT id=%s seq=%lu uptime=%lus",
             deviceId, (unsigned long)(++seq), (unsigned long)up);

    if (MQTT_Publish(topicHb, payload, 0, 0))
    {
      fails = 0;
      hbCount++;
      Log("Heartbeat #%lu sent", (unsigned long)seq);
    }
    else
    {
      Log("Heartbeat #%lu FAILED (%u/%u)", (unsigned long)seq,
          (unsigned)(fails + 1U), (unsigned)HEARTBEAT_MAX_FAILS);
      if (++fails >= HEARTBEAT_MAX_FAILS)
      {
        fails = 0;
        mqttUp = false;
        (void)osEventFlagsSet(sysEventsHandle, EVT_MQTT_LOST);
      }
    }
  }
}

/* ===========================================================================
 *  Cloud command task  -  executes commands received from the broker
 * ========================================================================= */

static void Cmd_Task(void *argument)
{
  CloudCmd_t cmd;
  char reply[96];

  (void)argument;

  for (;;)
  {
    if (osMessageQueueGet(cmdQueueHandle, &cmd, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    cmdCount++;
    Log("Cloud command: '%s'", cmd.text);

    if (strcasecmp(cmd.text, "LED_ON") == 0)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      snprintf(reply, sizeof(reply), "ACK LED_ON");
    }
    else if (strcasecmp(cmd.text, "LED_OFF") == 0)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      snprintf(reply, sizeof(reply), "ACK LED_OFF");
    }
    else if (strcasecmp(cmd.text, "LED_TOGGLE") == 0)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      snprintf(reply, sizeof(reply), "ACK LED_TOGGLE led=%s",
               (HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin) == GPIO_PIN_SET) ? "on" : "off");
    }
    else if (strcasecmp(cmd.text, "PING") == 0)
    {
      snprintf(reply, sizeof(reply), "PONG");
    }
    else if (strcasecmp(cmd.text, "STATUS") == 0)
    {
      snprintf(reply, sizeof(reply), "OK uptime=%lus hb=%lu cmds=%lu led=%s",
               (unsigned long)(osKernelGetTickCount() / osKernelGetTickFreq()),
               (unsigned long)hbCount, (unsigned long)cmdCount,
               (HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin) == GPIO_PIN_SET) ? "on" : "off");
    }
    else
    {
      snprintf(reply, sizeof(reply), "ERR unknown-command");
    }

    if (mqttUp)
    {
      (void)MQTT_Publish(topicResp, reply, 0, 0);
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  *         Connection manager: provisioning -> ESP32 init -> Wi-Fi -> MQTT ->
  *         monitor, with automatic recovery and exponential back-off.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  bool     haveCreds = false;
  bool     espReady  = false;
  uint32_t backoffMs = BACKOFF_MIN_MS;

  (void)argument;

  App_BuildIdentity();
  (void)HAL_UART_Receive_IT(&huart2, &consoleRxByte, 1);

  Console_Printf("\r\n\r\n**********************************************\r\n");
  Console_Printf("  STM32 host + ESP32 AT modem - MQTT cloud client\r\n");
  Console_Printf("  Device ID : %s\r\n", deviceId);
  Console_Printf("  Cmd topic : %s\r\n", topicCmd);
  Console_Printf("**********************************************\r\n");

  for (;;)
  {
    /* 1) Dynamic provisioning ------------------------------------------- */
    if (!haveCreds)
    {
      Provision_Run();
      haveCreds = true;
      backoffMs = BACKOFF_MIN_MS;
    }

    /* 2) ESP32 bring-up (AT, RST, ATE0, CWMODE) ------------------------- */
    if (!espReady)
    {
      if (!ESP_Init())
      {
        Log("Retrying in %lu s ...", (unsigned long)(backoffMs / 1000U));
        osDelay(MS2T(backoffMs));
        backoffMs = (backoffMs * 2U > BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : (backoffMs * 2U);
        continue;
      }
      espReady = true;
    }

    /* 3) Wi-Fi (CWJAP) + MQTT session ----------------------------------- */
    SessResult_t r = Session_Establish();

    if (r == SESS_OK)
    {
      backoffMs = BACKOFF_MIN_MS;

      MonResult_t m = Monitor_Loop();       /* returns on link loss / re-provision */
      mqttUp = false;

      if (m == MON_REPROV)
      {
        Log("Re-provisioning requested - disconnecting");
        (void)AT_Command(3000, NULL, 0, "AT+MQTTCLEAN=%d", MQTT_LINK_ID);
        (void)AT_Command(3000, NULL, 0, "AT+CWQAP");
        wifiUp = false;
        haveCreds = false;
      }
      else
      {
        Log("Connection lost - reconnecting ...");
        osDelay(MS2T(BACKOFF_MIN_MS));
      }
      continue;
    }

    switch (r)
    {
      case SESS_BAD_CREDS:
        Log("Wi-Fi credentials rejected - please enter them again");
        haveCreds = false;
        break;

      case SESS_ESP_DEAD:
        Log("ESP32 not responding - will re-initialise it");
        espReady = false;
        osDelay(MS2T(backoffMs));
        break;

      case SESS_RETRY:
      default:
        Log("Retrying in %lu s ...", (unsigned long)(backoffMs / 1000U));
        osDelay(MS2T(backoffMs));
        break;
    }
    backoffMs = (backoffMs * 2U > BACKOFF_MAX_MS) ? BACKOFF_MAX_MS : (backoffMs * 2U);
  }
  /* USER CODE END 5 */
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
