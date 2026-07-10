/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "File_Handling_RTOS.h"

/* Devices */
#include "bmp581.h"
#include "lidar.h"
#include "cansat_core.h"
#include "gnss_reader.h"
#include "imu.h"
#include "sx1276.h"
#include "hmi.h"
#include "vortex_logo.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c3_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for TaskFSM */
osThreadId_t TaskFSMHandle;
const osThreadAttr_t TaskFSM_attributes = {
  .name = "TaskFSM",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for TaskSensors */
osThreadId_t TaskSensorsHandle;
const osThreadAttr_t TaskSensors_attributes = {
  .name = "TaskSensors",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 512 * 4
};
/* Definitions for TaskSDCard */
osThreadId_t TaskSDCardHandle;
const osThreadAttr_t TaskSDCard_attributes = {
  .name = "TaskSDCard",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for TaskLoRa */
osThreadId_t TaskLoRaHandle;
const osThreadAttr_t TaskLoRa_attributes = {
  .name = "TaskLoRa",
  .priority = (osPriority_t) osPriorityAboveNormal1,
  .stack_size = 512 * 4
};
/* Definitions for TaskHMIHandle */
osThreadId_t TaskHMIHandleHandle;
const osThreadAttr_t TaskHMIHandle_attributes = {
  .name = "TaskHMIHandle",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for I2C1_Mutex */
osMutexId_t I2C1_MutexHandle;
const osMutexAttr_t I2C1_Mutex_attributes = {
  .name = "I2C1_Mutex"
};
/* USER CODE BEGIN PV */

/* --- Flight session and file naming ----------------------------------------*/
/* Session number is auto-incremented at boot by scanning existing files on SD. */
uint8_t flight_session = 1;
char data_filename[16];   /* e.g. "DATA_001.CSV" */
char lidar_filename[16];  /* e.g. "LIDA_001.CSV" */
uint8_t is_calibrated = 0;

uint8_t TX_to_Baro [] = "A";
uint8_t RX_from_Baro [] = "A";

extern volatile uint8_t FatFsCnt;
extern void SDTimer_Handler(void);
FIL fil_lidar;
uint8_t lidar_file_open = 0;
/* DATA_xxx.CSV kept open for the whole flight, same as fil_lidar: going
   through Update_File (f_stat + f_open + f_write + f_close per line) while
   the LiDAR stream monopolizes the SD bus throttled DATA to ~1 line/s and
   backed up qSDCard by several seconds — rows then carried an enqueue-time
   timestamp but write-time flags, which is why DATA_002.CSV showed
   RECOVERY flags ~6 s before the FSM actually transitioned. */
FIL fil_data;
uint8_t data_file_open = 0;

/* --- Global state and sensor data (shared across all tasks) ----------------*/
volatile uint8_t configFlag = 1;  /* Driven by HMI button; triggers CONFIG/READY transitions */
CanSatState_t currentState = STATE_STANDBY;
float current_height = 0.0f;
/* Tick of the last BMP581 sample whose pressure actually changed (0 = none
   yet). A healthy sensor at 20 Hz never returns bit-identical pressure twice
   in a row for long (0.3 Pa RMS noise), so staleness here means the sensor or
   the I2C1 bus is stuck — see the freeze watchdog in EVENT_BARO_READY. */
volatile uint32_t baro_last_data_ms = 0;
uint8_t imu_is_connected = 0;
uint32_t calibration_duration_ms = 0;
/* Gates the one-shot LW20 wake-up sequence in startTaskFSM's STATE_CONFIG case.
   Must run after the main loop begins (not before) so STANDBY -> CONFIG
   already happened by the time startTaskHMI's splash-screen deadline checks
   currentState. */
uint8_t lidar_boot_done = 0;

/* Stack headroom per task, in bytes (uxTaskGetStackHighWaterMark), updated at
   1Hz from startTaskFSM. Watch in the debugger — attach without reset so as
   not to disturb real boot timing. */
uint32_t stack_free_fsm     = 0;
uint32_t stack_free_sensors = 0;
uint32_t stack_free_sdcard  = 0;
uint32_t stack_free_lora    = 0;
uint32_t stack_free_hmi     = 0;
/* Failed f_write/f_sync count on the SD log files — watch in the debugger.
   Write errors used to be ignored, letting a corrupted FAT (torn directory
   write on a power cut) kill logging silently after the first rows. */
volatile uint32_t sd_write_errors = 0;
/* Failed f_open count on the SD log files. Separate from sd_write_errors
   because the causes differ: FR_TOO_MANY_OPEN_FILES points at the _FS_LOCK
   table (3 FILs exist: fil, fil_lidar, fil_data), FR_NO_FILE/FR_DISK_ERR at
   the volume itself. */
volatile uint32_t sd_open_errors = 0;

GNSS_Data_t latest_gnss;
Barometer_Data_t latest_baro;
IMU_Data_t latest_imu;
LIDAR_Data_t latest_lidar = { .distance = 99.0f };
BMP_t bmp_sensor;
Battery_Data_t latest_battery;
extern HMI_Display_Data_t HMI_display_data;

/* LiDAR DMA ping-pong: stores the carry-over fragment when a line spans a half-buffer boundary */
char lidar_cut_char[32] = {0};
uint8_t cut_char_len = 0;
extern uint8_t lidar_dma_buf[];

/* IMU DMA receive buffer: 34 bytes from BNO055_GYRO_DATA_X_LSB (0x14),
   covering gyro, Euler angles, quaternion, and linear acceleration registers. */
uint8_t imu_dma_rx_buf[34];

/* --- Barometer calibration references --------------------------------------*/
double reference_pressure_Pa = 101325.0;
double reference_temp_C = 25.0;

/* --- FreeRTOS queue handles ------------------------------------------------*/
QueueHandle_t qSensorEvents;
QueueHandle_t qSDCard;
QueueHandle_t qLoRa;
QueueHandle_t qSDCard_LIDAR;
QueueHandle_t qHMI_Events;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
void startTaskFSM(void *argument);
void startTaskSensors(void *argument);
void startTaskSDCard(void *argument);
void startTaskLoRa(void *argument);
void startTaskHMI(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Retarget printf/fwrite to USART1 for debug output */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* Set to 1 to disable GNSS and keep USART1 RX free for a pure-TX debug session */
uint8_t debug = 0;  /* 0 required for GNSS: USART1 is shared with the debug console */

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
  /* Force-reset both UARTs to guarantee a clean electrical state regardless
     of how the previous session ended (debugger, power glitch, etc.). */
  __HAL_RCC_USART3_FORCE_RESET();
  __HAL_RCC_USART1_FORCE_RESET();
  HAL_Delay(10);
  __HAL_RCC_USART3_RELEASE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* --- SPI: LoRa NSS must idle HIGH — a low NSS at power-on locks the bus --*/
  HAL_GPIO_WritePin(GPIOB, LORA_NSS_Pin, GPIO_PIN_SET);

  if (debug) {
      char startup_msg[] = "\r\n[i] System Booting... Testing Hardware\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);
      HAL_Delay(2000);
  }

  /* --- Battery voltage (ADC1, single conversion) ---------------------------*/
  if (debug) {
    char adc_msg[128];
    sprintf(adc_msg, "\r\n[*] Testing Power (ADC1)...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)adc_msg, strlen(adc_msg), 100);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
        float pin_voltage = ((float)raw_adc / 4095.0f) * 3.3f;

        /* Adjust this ratio to match the physical PCB resistor divider.
           Example: 10k/10k → 2.0, 20k/10k → 3.0. */
        float voltage_divider_ratio = 3.0f;

        float true_battery_voltage = pin_voltage * voltage_divider_ratio * 1.025f;
        int part_ent = (int)true_battery_voltage;
        int part_dec = (int)((true_battery_voltage - part_ent) * 100);
        sprintf(adc_msg, "    -> SUCCESS! Battery: %d.%02d V (Raw ADC: %lu)\r\n",
                part_ent, part_dec, raw_adc);
        HAL_UART_Transmit(&huart1, (uint8_t*)adc_msg, strlen(adc_msg), 100);
    }
    HAL_ADC_Stop(&hadc1);
  }

  /* --- I2C bus scan --------------------------------------------------------*/
  if (debug) {
    char scan_msg[64];
    sprintf(scan_msg, "\r\n[*] Scanning I2C Bus 1...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);

    for (uint8_t i = 1; i < 128; i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5) == HAL_OK) {
            sprintf(scan_msg, "    -> Found device at address: 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
        }
    }

    sprintf(scan_msg, "\r\n[*] Scanning I2C Bus 3...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
    for (uint8_t i = 1; i < 128; i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i << 1), 3, 5) == HAL_OK) {
            sprintf(scan_msg, "    -> Found device at address: 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
        }
    }
    sprintf(scan_msg, "[*] Scan Complete.\r\n\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
  }

  /* --- LiDAR UART ping (USART3) --------------------------------------------*/
  if (debug) {
    char diag_msg[128];
    sprintf(diag_msg, "    -> STM32 UART3 BaudRate: %lu\r\n", huart3.Init.BaudRate);
    HAL_UART_Transmit(&huart1, (uint8_t*)diag_msg, strlen(diag_msg), 100);

    __HAL_UART_FLUSH_DRREGISTER(&huart3);

    /* Send a space to try to wake the LiDAR menu, then listen for a reply */
    uint8_t test_tx = ' ';
    HAL_UART_Transmit(&huart3, &test_tx, 1, 100);

    uint8_t test_rx[1] = {0};
    HAL_StatusTypeDef uart_status = HAL_UART_Receive(&huart3, test_rx, 1, 500);

    if (uart_status == HAL_OK) {
        if (test_rx[0] >= 32 && test_rx[0] <= 126) {
            sprintf(diag_msg, "    -> SUCCESS! Received valid ASCII: '%c' (0x%02X)\r\n\n",
                    test_rx[0], test_rx[0]);
        } else {
            sprintf(diag_msg, "    -> WARNING! Received garbage byte: 0x%02X (Baud rate mismatch?)\r\n\n",
                    test_rx[0]);
        }
    } else {
        sprintf(diag_msg, "    -> FAILED! No response from LiDAR. (Check TX/RX wiring)\r\n\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)diag_msg, strlen(diag_msg), 100);
  }

  /* --- Sensor initialization -----------------------------------------------*/
  {
      uint8_t baro_ok = (bmp581_init_precise_normal(&bmp_sensor) == 0);
      if (debug) {
          if (baro_ok) { char m[] = "[+] BMP581 OK\r\n";   HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100); }
          else         { char m[] = "[-] BMP581 FAIL\r\n"; HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100); }
      }
  }
  {
      imu_is_connected = (IMU_Init(&hi2c3) == 1);
      if (debug) {
          if (imu_is_connected) { char m[] = "[+] BNO055 OK\r\n";   HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100); }
          else                  { char m[] = "[-] BNO055 FAIL\r\n"; HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100); }
      }
  }

  if (debug) {
      char vortex_art[] =
        "\r\n"
        "---------------------------------------------------------------\r\n"
        " __     __         _               ___  ____  \r\n"
        " \\ \\   / /__  _ __| |_ _____  __  / _ \\/ ___| \r\n"
        "  \\ \\ / / _ \\| '__| __/ _ \\ \\/ / | | | \\___ \\ \r\n"
        "   \\ V / (_) | |  | ||  __/>  <  | |_| |___) |\r\n"
        "    \\_/ \\___/|_|   \\__\\___/_/\\_\\  \\___/|____/ \r\n"
        "---------------------------------------------------------------\r\n"
        "\r\n[i] Starting RTOS...\r\n\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)vortex_art, strlen(vortex_art), HAL_MAX_DELAY);
  }

  /* --- GNSS (skipped when debug=1 to keep USART1 RX free) -----------------*/
  if (debug) {
      char gnss_init_msg[] = "[*] Arming GNSS Interrupt... SKIPPED (debug=1)\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)gnss_init_msg, strlen(gnss_init_msg), 100);
  } else {
      GNSS_ConfigureM10();  /* SAM-M10Q has no config flash: 10 Hz + airborne
                               <2g must be re-sent at every power-up */
      GNSS_Init();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of I2C1_Mutex */
  I2C1_MutexHandle = osMutexNew(&I2C1_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  qSensorEvents = xQueueCreate(20, sizeof(SensorEvent_t));
  qSDCard       = xQueueCreate(5,  sizeof(TelemetryPacket_t));
  qLoRa         = xQueueCreate(5,  sizeof(TelemetryPacket_t));
  /* Depth trimmed to 65 to keep runtime heap usage within budget.
     Each LidarPacket_t is 76 bytes; 65 slots give ~1.3 s of burst buffering
     at 50 Hz. Without this trim, TaskHMI (created last) silently failed
     osThreadNew() because the heap was exhausted by the time it was created. */
  /* 65 slots (4940B) was the single largest RAM consumer in .bss — trimmed to
     recover margin for RAM overflows on new pin/peripheral configs. 50 slots
     at 76B each = 3800B, still ~1s of burst buffering at 50Hz. */
  qSDCard_LIDAR = xQueueCreate(50, sizeof(LidarPacket_t));
  qHMI_Events   = xQueueCreate(10, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskFSM */
  TaskFSMHandle = osThreadNew(startTaskFSM, NULL, &TaskFSM_attributes);

  /* creation of TaskSensors */
  TaskSensorsHandle = osThreadNew(startTaskSensors, NULL, &TaskSensors_attributes);

  /* creation of TaskSDCard */
  TaskSDCardHandle = osThreadNew(startTaskSDCard, NULL, &TaskSDCard_attributes);

  /* creation of TaskLoRa */
  TaskLoRaHandle = osThreadNew(startTaskLoRa, NULL, &TaskLoRa_attributes);

  /* creation of TaskHMIHandle */
  TaskHMIHandleHandle = osThreadNew(startTaskHMI, NULL, &TaskHMIHandle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* osThreadNew() returns NULL silently on heap exhaustion — the task simply
     never starts. Surface any failure over UART before the scheduler takes over. */
  if (TaskFSMHandle == NULL || TaskSensorsHandle == NULL || TaskSDCardHandle == NULL ||
      TaskLoRaHandle == NULL || TaskHMIHandleHandle == NULL) {
      char fail_msg[96];
      sprintf(fail_msg,
              "[-] TASK CREATE FAILED (heap exhausted?) FSM:%d Sensors:%d SDCard:%d LoRa:%d HMI:%d\r\n",
              TaskFSMHandle == NULL, TaskSensorsHandle == NULL, TaskSDCardHandle == NULL,
              TaskLoRaHandle == NULL, TaskHMIHandleHandle == NULL);
      HAL_UART_Transmit(&huart1, (uint8_t*)fail_msg, strlen(fail_msg), 100);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40621236;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 839;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_GPIO_CS_GPIO_Port, SD_GPIO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GPIO_OUT_Pin|HMILED_GPIO_Pin|LORA_NSS_Pin|LORA_RST_Pin
                          |VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HMIBTN_EXTI13_Pin */
  GPIO_InitStruct.Pin = HMIBTN_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HMIBTN_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_GPIO_CS_Pin */
  GPIO_InitStruct.Pin = SD_GPIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_GPIO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GPIO_OUT_Pin HMILED_GPIO_Pin LORA_NSS_Pin LORA_RST_Pin
                           VPOWER_EN_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = LED_GPIO_OUT_Pin|HMILED_GPIO_Pin|LORA_NSS_Pin|LORA_RST_Pin
                          |VPOWER_EN_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_GPIO_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_GPIO_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_GPIO_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ===========================================================================
 * CALLBACKS
 * =========================================================================*/

/* --- HMI button (PC13, EXTI13) with debounce --------------------------------*/
static uint32_t last_button_press = 0;
#define DEBOUNCE_DELAY_MS 200
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        uint32_t now = HAL_GetTick();
        if (now - last_button_press < DEBOUNCE_DELAY_MS) {
            return;
        }
        last_button_press = now;

        /* Guard against writing to the queue before it is created */
        if (qHMI_Events != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            uint8_t event = 1;
            xQueueSendFromISR(qHMI_Events, &event, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* --- IMU DMA transfer complete (I2C3) -------------------------------------*/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3) {
        IMU_ProcessData_DMA(&latest_imu);
        /* Timestamp after processing — this marks the actual measurement instant */
        latest_imu.timestamp_ms = HAL_GetTick();
    }
}

/* --- I2C bus error recovery ------------------------------------------------
   Without this, a single bus glitch (NACK, arbitration loss) leaves the HAL
   I2C state stuck, and every subsequent read on that bus fails forever —
   no recovery short of a full power cycle. Re-init the affected peripheral
   so the next scheduled read can succeed again. */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3) {
        HAL_I2C_DeInit(&hi2c3);
        MX_I2C3_Init();
    } else if (hi2c->Instance == I2C1) {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
    }
}

/* --- I2C1 stuck-bus recovery -----------------------------------------------
   The BMP581 and SSD1306 use blocking HAL calls, which never reach
   HAL_I2C_ErrorCallback above. A slave left holding SDA low mid-byte (power
   glitch, bus noise) cannot be cleared by re-initialising the peripheral:
   the remaining bits must be clocked out manually (up to 9 SCL pulses) until
   the slave releases SDA, followed by a STOP condition.
   Task context only (uses osDelay) — call with I2C1_Mutex held. */
static void I2C1_Bus_Recover(void)
{
    GPIO_InitTypeDef g = {0};

    HAL_I2C_DeInit(&hi2c1);

    /* Both lines as open-drain GPIO, released (external pull-ups hold high) */
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = BARO_I2C1_SCL_Pin;
    HAL_GPIO_WritePin(BARO_I2C1_SCL_GPIO_Port, BARO_I2C1_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(BARO_I2C1_SCL_GPIO_Port, &g);
    g.Pin = BARO_I2C1_SDA_Pin;
    HAL_GPIO_WritePin(BARO_I2C1_SDA_GPIO_Port, BARO_I2C1_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(BARO_I2C1_SDA_GPIO_Port, &g);

    /* Clock until the slave releases SDA (9 pulses = one full byte + ACK) */
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(BARO_I2C1_SDA_GPIO_Port, BARO_I2C1_SDA_Pin) == GPIO_PIN_SET) {
            break;
        }
        HAL_GPIO_WritePin(BARO_I2C1_SCL_GPIO_Port, BARO_I2C1_SCL_Pin, GPIO_PIN_RESET);
        osDelay(1);
        HAL_GPIO_WritePin(BARO_I2C1_SCL_GPIO_Port, BARO_I2C1_SCL_Pin, GPIO_PIN_SET);
        osDelay(1);
    }

    /* STOP condition: SDA low -> high while SCL is high */
    HAL_GPIO_WritePin(BARO_I2C1_SDA_GPIO_Port, BARO_I2C1_SDA_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(BARO_I2C1_SDA_GPIO_Port, BARO_I2C1_SDA_Pin, GPIO_PIN_SET);
    osDelay(1);

    MX_I2C1_Init();  /* restores both pins to their I2C alternate function */
}

/* --- float_to_str ---------------------------------------------------------*/
/**
 * @brief Converts a float to a decimal string without using sprintf.
 * @param value    Value to convert.
 * @param decimals Number of decimal places (1, 2, or 6).
 * @param buffer   Output buffer (minimum 16 bytes).
 */
void float_to_str(float value, uint8_t decimals, char* buffer)
{
    int idx = 0;

    /* Handle negative sign */
    if (value < 0) {
        buffer[idx++] = '-';
        value = -value;
    }

    int int_part = (int)value;

    if (int_part == 0) {
        buffer[idx++] = '0';
    } else {
        char temp[12];
        int temp_idx = 0;
        int num = int_part;

        while (num > 0) {
            temp[temp_idx++] = '0' + (num % 10);
            num /= 10;
        }

        /* Reverse: digits were accumulated least-significant first */
        for (int i = temp_idx - 1; i >= 0; i--) {
            buffer[idx++] = temp[i];
        }
    }

    if (decimals > 0) {
        buffer[idx++] = '.';

        float dec_part = value - int_part;
        for (uint8_t i = 0; i < decimals; i++) {
            dec_part *= 10;
            int digit = (int)dec_part;
            buffer[idx++] = '0' + digit;
            dec_part -= digit;
        }
    }

    buffer[idx] = '\0';
}

/* --- UART error routing ---------------------------------------------------*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Route to GNSS module (USART1) */
    if (huart->Instance == USART1) {
        GNSS_UART_Error_Handler(huart);
    }
    /* Route to LiDAR module (USART3) */
    else if (huart->Instance == USART3) {
        Lidar_UART_Error_Handler(huart);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startTaskFSM */
/**
  * @brief  Function implementing the TaskFSM thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTaskFSM */
void startTaskFSM(void *argument)
{
  /* USER CODE BEGIN 5 */
    extern UART_HandleTypeDef huart1;
    char debug_msg[128];

    /* =========================================================================
     * MAIN FSM LOOP  —  10 ms master tick
     * ========================================================================= */
    CanSatState_t last_printed_state = (CanSatState_t)-1;
    uint32_t tick_10ms = 0;

  /* Infinite loop */
  for(;;)
  {
      /* Battery + GNSS: 1 Hz, independent of flight state — the HMI pages
         showing them are only visible during STATE_CONFIG (see startTaskHMI),
         not STATE_READY+. GNSS reception itself (USART1 RX IRQ -> parsed_gnss)
         already runs regardless of state; only the copy into latest_gnss was
         gated.

         SENSOR_BOOT_GRACE_MS delays the first tick: TaskSensors is the
         highest-priority task in the app, and firing it immediately would let
         it preempt TaskFSM's one-shot LiDAR UART/DMA arming and TaskHMI's
         one-shot SSD1306 readiness check, both early in the boot sequence. */
      #define SENSOR_BOOT_GRACE_MS 3000
      static uint32_t task_start_tick = 0;
      static uint8_t task_start_tick_set = 0;
      if (!task_start_tick_set) { task_start_tick = HAL_GetTick(); task_start_tick_set = 1; }

      static uint32_t last_bat_gnss_fetch = 0;
      if (HAL_GetTick() - task_start_tick > SENSOR_BOOT_GRACE_MS &&
          HAL_GetTick() - last_bat_gnss_fetch > 1000) {
          last_bat_gnss_fetch = HAL_GetTick();

          /* Stack headroom per task (see globals above) — cheap TCB read. */
          stack_free_fsm     = uxTaskGetStackHighWaterMark(TaskFSMHandle)       * sizeof(StackType_t);
          stack_free_sensors = uxTaskGetStackHighWaterMark(TaskSensorsHandle)   * sizeof(StackType_t);
          stack_free_sdcard  = uxTaskGetStackHighWaterMark(TaskSDCardHandle)    * sizeof(StackType_t);
          stack_free_lora    = uxTaskGetStackHighWaterMark(TaskLoRaHandle)      * sizeof(StackType_t);
          stack_free_hmi     = uxTaskGetStackHighWaterMark(TaskHMIHandleHandle) * sizeof(StackType_t);

          SensorEvent_t bat_ticket = EVENT_BATTERY_READY;
          xQueueSend(qSensorEvents, &bat_ticket, 0);
          SensorEvent_t gnss_ticket = EVENT_GNSS_READY;
          xQueueSend(qSensorEvents, &gnss_ticket, 0);
      }

      /* --- 100 Hz block: sensor event dispatch ----------------------------*/
      if (currentState >= STATE_READY && currentState < STATE_OFF) {

          /* IMU at 100 Hz via DMA (non-blocking).
             Barometer is polled at 20 Hz in the low-frequency block below:
             polling it at 100 Hz saturated TaskSensors with blocking I2C reads
             and starved TaskLoRa of CPU. 20 Hz is sufficient for altitude. */
          SensorEvent_t imu_ticket = EVENT_IMU_READY;
          xQueueSend(qSensorEvents, &imu_ticket, 0);
      }

      /* --- 20 Hz block (every 5th tick) -----------------------------------*/
      if (tick_10ms % 5 == 0)
      {
          /* Barometer at 20 Hz */
          if (currentState >= STATE_READY && currentState < STATE_OFF) {
              SensorEvent_t baro_ticket = EVENT_BARO_READY;
              xQueueSend(qSensorEvents, &baro_ticket, 0);
          }

          /* A. Log state transitions over UART */
          if (currentState != last_printed_state) {
              if (currentState == STATE_READY) {
                  /* Only nudge the LiDAR if no distance line arrived recently:
                     a keystroke sent to an LW20 that is already streaming is
                     interpreted as menu navigation and silently switches the
                     streamed variable, which kills the LIDA_xxx.CSV log after
                     its first lines. */
                  if (HAL_GetTick() - lidar_last_data_ms > 2000) {
                      Lidar_ForceStream();
                  }
              }
              if (debug) {
                  switch(currentState) {
                      case STATE_STANDBY:   sprintf(debug_msg, "\r\n[FSM] State: STANDBY\r\n");   break;
                      case STATE_CONFIG:    sprintf(debug_msg, "\r\n[FSM] State: CONFIG\r\n");    break;
                      case STATE_READY:     sprintf(debug_msg, "\r\n[FSM] State: READY\r\n");     break;
                      case STATE_ASCENSION: sprintf(debug_msg, "\r\n[FSM] State: ASCENSION\r\n"); break;
                      case STATE_DROP:      sprintf(debug_msg, "\r\n[FSM] State: DROP\r\n");      break;
                      case STATE_RECOVERY:  sprintf(debug_msg, "\r\n[FSM] State: RECOVERY\r\n");  break;
                      case STATE_OFF:       sprintf(debug_msg, "\r\n[FSM] State: OFF\r\n");       break;
                  }
                  HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
              }
              last_printed_state = currentState;
          }

          /* B. Telemetry dispatch (LoRa + DATA.CSV) */
          if (currentState >= STATE_READY && currentState < STATE_OFF) {
              static uint32_t last_lora_tx = 0;
              if (HAL_GetTick() - last_lora_tx > TIME_BETWEEN_PACKET_LORA_mS) {
                  last_lora_tx = HAL_GetTick();

                  TelemetryPacket_t telemetry_pkt;
                  memset(&telemetry_pkt, 0, sizeof(TelemetryPacket_t));

                  sprintf(telemetry_pkt.baro.timestamp, "%lu", HAL_GetTick());
                  telemetry_pkt.baro.height      = current_height;
                  telemetry_pkt.baro.temperature = latest_baro.temperature;
                  telemetry_pkt.imu.pitch        = latest_imu.pitch;
                  telemetry_pkt.imu.roll         = latest_imu.roll;
                  telemetry_pkt.imu.yaw          = latest_imu.yaw;
                  telemetry_pkt.imu.accelX       = latest_imu.accelX;
                  telemetry_pkt.imu.accelY       = latest_imu.accelY;
                  telemetry_pkt.imu.accelZ       = latest_imu.accelZ;
                  telemetry_pkt.imu.gyroX        = latest_imu.gyroX;
                  telemetry_pkt.imu.gyroY        = latest_imu.gyroY;
                  telemetry_pkt.imu.gyroZ        = latest_imu.gyroZ;
                  telemetry_pkt.gnss.latitude    = latest_gnss.latitude;
                  telemetry_pkt.gnss.longitude   = latest_gnss.longitude;
                  telemetry_pkt.gnss.satellites  = latest_gnss.satellites;
                  /* Altitude read straight from the parser rather than from
                     latest_gnss: the GGA now arrives at 10 Hz (SAM-M10Q
                     configured at boot) while the EVENT_GNSS_READY copy only
                     runs at 1 Hz. 32-bit float read is atomic on Cortex-M4. */
                  telemetry_pkt.gnss.altitude    = parsed_gnss.altitude;
                  telemetry_pkt.bat.voltage      = latest_battery.voltage;

                  xQueueSend(qLoRa, &telemetry_pkt, 0);
                  xQueueSend(qSDCard, &telemetry_pkt, 0);
              }
          }

          /* C. Flight state machine transitions */
          switch(currentState) {
              case STATE_STANDBY:
                  if (configFlag == 1) currentState = STATE_CONFIG;
                  break;

              case STATE_CONFIG:
              {
                  if (!lidar_boot_done) {
                      /* Must run after STANDBY -> CONFIG (case STATE_STANDBY, above),
                         not before this loop starts — startTaskHMI's splash screen
                         checks currentState on a timer and permanently suspends itself
                         if it still sees STANDBY. */
                      if (debug) {
                          sprintf(debug_msg, "\r\n[*] Executing LW20-C Advanced Boot Sequence...\r\n");
                          HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
                      }

                      /* 1. Hardware re-initialization: start from a clean UART state */
                      HAL_UART_DeInit(&huart3);
                      extern void MX_USART3_UART_Init(void);
                      MX_USART3_UART_Init();

                      /* 2. LiDAR baud-rate auto-detection: send spaces/CR to satisfy the
                            LW20 auto-detect sequence described in the datasheet (p.7). */
                      osDelay(1000);
                      uint8_t dummy_cmd[] = {' ', ' ', '\r', '\n'};
                      HAL_UART_Transmit(&huart3, dummy_cmd, 4, 100);
                      osDelay(100);

                      /* 3. Force streaming mode: ESC closes any open menu, Down Arrow
                            starts the data stream. */
                      uint8_t stream_cmd[] = {0x1B, 0x1B, 0x5B, 0x42};
                      HAL_UART_Transmit(&huart3, stream_cmd, 4, 100);
                      osDelay(100);

                      /* 4. Clear hardware errors: the LW20 started streaming before we
                            were listening, so the STM32 UART accumulated an Overrun
                            error (ORE). Purge everything before arming DMA. */
                      HAL_UART_AbortReceive(&huart3);
                      __HAL_UART_CLEAR_OREFLAG(&huart3);
                      __HAL_UART_CLEAR_NEFLAG(&huart3);
                      __HAL_UART_CLEAR_FEFLAG(&huart3);
                      __HAL_UART_FLUSH_DRREGISTER(&huart3);

                      /* 5. Arm DMA receiver */
                      Lidar_Init(&huart3);

                      if (debug) {
                          sprintf(debug_msg, "[+] LiDAR Stream Locked & DMA Armed\r\n");
                          HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
                      }

                      lidar_boot_done = 1;
                  }
                  /* --- LiDAR stream watchdog (CONFIG only) ----------------
                     The LW20 keeps its internal state across an MCU-only
                     reset (reflash / NRST), so the one-shot boot sequence
                     above assumes a cold device and can leave a warm one
                     silent (or stuck in its menu). Re-send the stream
                     request every 2 s until distance lines actually flow,
                     making CONFIG deterministic for both reset types and
                     feeding the HMI sensors page a live distance.
                     Streaming during CONFIG is safe for the event queue:
                     at 115200 baud the DMA half/full events arrive at
                     ~115 Hz and each 100-byte chunk parse is microseconds,
                     so TaskSensors still blocks between events. (The
                     starvation previously blamed on CONFIG streaming
                     traced to the local_line stack overflow fixed in
                     Process_Lidar_Buffer_Chunk.) */
                  else if (HAL_GetTick() - lidar_last_data_ms > 2000) {
                      static uint32_t last_stream_retry = 0;
                      if (HAL_GetTick() - last_stream_retry > 2000) {
                          last_stream_retry = HAL_GetTick();
                          Lidar_ForceStream();
                      }
                  }

                  /* Auto-calibration on CONFIG entry runs from startTaskHMI instead of
                     here, since it draws to the SSD1306 (HMI_display_recalib/_progress/
                     _done) — the framebuffer isn't mutex-protected, so only the task
                     that owns the display should touch it. Just wait for it to finish. */
                  if (configFlag == 0 && is_calibrated) {
                      is_calibrated = 0;  /* Reset so a future CONFIG re-entry (READY -> CONFIG) recalibrates */
                      currentState = STATE_READY;
                  }
                  break;
              }

              /* Every flight transition below requires its condition to hold
                 for a THRESH_*_HOLD_MS window. Single-sample tests let ground
                 handling walk the FSM READY -> RECOVERY in 2 s during the
                 DATA_002 test run. Inside a window, up to
                 THRESH_*_MAX_OUTLIERS samples may violate the condition
                 without consequence (baro noise, gyro gust) — one more aborts
                 the window, which restarts on the next passing sample. */

              case STATE_READY:
              {
                  /* Launch detection on barometric altitude alone: > 10 m
                     rules out any ground handling, which the old
                     "lidar-in-box + 0.3 m" single-sample test did not. */
                  static uint32_t ascension_cond_since_ms = 0;
                  static uint8_t  ascension_outliers = 0;
                  if (configFlag == 1) {
                      currentState = STATE_CONFIG;
                      ascension_cond_since_ms = 0;
                      ascension_outliers = 0;
                  } else if (current_height > THRESH_ALTITUDE_ASCENSION_M) {
                      if (ascension_cond_since_ms == 0) {
                          ascension_cond_since_ms = HAL_GetTick();
                          ascension_outliers = 0;
                      } else if (HAL_GetTick() - ascension_cond_since_ms >= THRESH_ASCENSION_HOLD_MS) {
                          currentState = STATE_ASCENSION;
                          ascension_cond_since_ms = 0;
                      }
                  } else if (ascension_cond_since_ms != 0
                             && ++ascension_outliers > THRESH_ASCENSION_MAX_OUTLIERS) {
                      ascension_cond_since_ms = 0;
                      ascension_outliers = 0;
                  }
                  break;
              }

              case STATE_ASCENSION:
              {
                  static uint32_t drop_cond_since_ms = 0;
                  static uint8_t  drop_outliers = 0;
                  if (latest_lidar.distance > THRESH_LIDAR_DEPLOYED_M
                      && current_height > THRESH_ALTITUDE_DEPLOY_MIN_M) {
                      if (drop_cond_since_ms == 0) {
                          drop_cond_since_ms = HAL_GetTick();
                          drop_outliers = 0;
                      } else if (HAL_GetTick() - drop_cond_since_ms >= THRESH_DROP_HOLD_MS) {
                          currentState = STATE_DROP;
                          drop_cond_since_ms = 0;
                      }
                  } else if (drop_cond_since_ms != 0
                             && ++drop_outliers > THRESH_DROP_MAX_OUTLIERS) {
                      drop_cond_since_ms = 0;
                      drop_outliers = 0;
                  }
                  break;
              }

              case STATE_DROP:
              {
                  /* Landing needs two independent sensors to agree: barometer
                     low AND IMU motionless. In descent the CanSat swings and
                     spins under its parachute (tens of deg/s), on the ground
                     the gyro sits near zero — so a baro glitch below 5 m
                     cannot end the flight on its own. */
                  static uint32_t landing_cond_since_ms = 0;
                  static uint8_t  landing_outliers = 0;
                  uint8_t is_still = fabsf(latest_imu.gyroX) < THRESH_GYRO_STILL_DPS
                                  && fabsf(latest_imu.gyroY) < THRESH_GYRO_STILL_DPS
                                  && fabsf(latest_imu.gyroZ) < THRESH_GYRO_STILL_DPS;
                  if (current_height < THRESH_ALTITUDE_LANDING_M && is_still) {
                      if (landing_cond_since_ms == 0) {
                          landing_cond_since_ms = HAL_GetTick();
                          landing_outliers = 0;
                      } else if (HAL_GetTick() - landing_cond_since_ms >= THRESH_LANDING_HOLD_MS) {
                          currentState = STATE_RECOVERY;
                          landing_cond_since_ms = 0;
                      }
                  } else if (landing_cond_since_ms != 0
                             && ++landing_outliers > THRESH_LANDING_MAX_OUTLIERS) {
                      landing_cond_since_ms = 0;
                      landing_outliers = 0;
                  }
                  break;
              }

              case STATE_RECOVERY:
                  break;

              case STATE_OFF:
                  break;
          }

          /* Heartbeat LED at 20 Hz */
          HAL_GPIO_TogglePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin);

      } /* end 20 Hz block */

      tick_10ms++;
      osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTaskSensors */
/**
* @brief Function implementing the TaskSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskSensors */
void startTaskSensors(void *argument)
{
  /* USER CODE BEGIN startTaskSensors */

    SensorEvent_t current_event;
    extern UART_HandleTypeDef huart1;

    /* Infinite loop */
    for(;;)
    {
        if (xQueueReceive(qSensorEvents, &current_event, portMAX_DELAY) == pdTRUE) {

            uint32_t current_ms = HAL_GetTick();

            switch(current_event) {

                /* --- Barometer: 20 Hz, blocking I2C read on shared I2C1 bus ------*/
                case EVENT_BARO_READY:
                {
                    extern double bmptemp;
                    extern double bmppress;

                    /* I2C1 is shared with the SSD1306 HMI; take the mutex per-sample
                       so the HMI task can still access the bus between reads. */
                    if (osMutexAcquire(I2C1_MutexHandle, osWaitForever) != osOK) {
                        break;
                    }
                    uint8_t baro_read_ok = (bmp581_read_precise_normal(&bmp_sensor) == 0);
                    osMutexRelease(I2C1_MutexHandle);

                    /* --- Freeze watchdog -------------------------------------
                       Ground log BARO_ISSUE.csv (2026-07-10): pressure AND
                       temperature froze at t=89 s for the remaining 244 s of
                       the session while all other telemetry kept flowing.
                       Failed reads leave bmppress untouched, and a healthy
                       sensor never returns bit-identical pressure for 3 s
                       (0.3 Pa RMS noise even through the IIR filter) — either
                       way, 60 stuck samples at 20 Hz mean the sensor or the
                       bus is wedged: unlock the bus and re-init the BMP581
                       (a brown-out also silently drops its continuous mode). */
                    static double   watchdog_prev_press = 0.0;
                    static uint16_t baro_stuck_samples  = 0;
                    if (baro_read_ok && bmppress != watchdog_prev_press) {
                        watchdog_prev_press = bmppress;
                        baro_stuck_samples  = 0;
                        baro_last_data_ms   = current_ms;
                    } else if (++baro_stuck_samples >= 60) {
                        baro_stuck_samples = 0;  /* retry every 3 s while stuck */
                        if (osMutexAcquire(I2C1_MutexHandle, osWaitForever) == osOK) {
                            I2C1_Bus_Recover();
                            bmp581_init_precise_normal(&bmp_sensor);
                            osMutexRelease(I2C1_MutexHandle);
                        }
                    }

                    if (baro_read_ok) {

                        /* Prime the derivative statics from the first real reading so
                           the first call does not produce a spurious altitude kick.
                           EVENT_BARO_READY only fires after STATE_READY is entered, which
                           can be tens of seconds after boot. Initialising prev_time and
                           prev_temp at 0 would produce a large fictitious dt and delta-T
                           on that first sample. */
                        static uint32_t prev_time = 0;
                        static uint8_t first_sample = 1;
                        float dt_sec = (current_ms - prev_time) / 1000.0f;
                        if (dt_sec <= 0.0f) dt_sec = 0.05f;
                        prev_time = current_ms;

                        static float prev_temp = 0.0f;
                        latest_baro.temperature = (float)bmptemp;

                        float delta_T = latest_baro.temperature - (float)reference_temp_C;
                        float temp_rate_of_change = first_sample ? 0.0f
                                : (latest_baro.temperature - prev_temp) / dt_sec;
                        prev_temp = latest_baro.temperature;
                        first_sample = 0;

                        float raw_altitude = 44330.0f * (1.0f - pow(
                                (float)(bmppress / reference_pressure_Pa), (1.0f / 5.255f)));

                        /* THIS V3 BOARD ONLY (healthy sensors keep Kp = 0, cf. V1):
                           this BMP581's pressure port is (quasi-)sealed, so the
                           trapped cavity gas makes raw pressure track die
                           temperature. Measured on the BARO_ISSUE.csv ground log
                           (2026-07-10): altitude = -43.57 m/K, linear over a
                           153 m / 3.6 K excursion with 0.69 m RMS residual.
                           Kp cancels that slope against the calibration
                           reference temperature. Re-check the slope after any
                           rework of the sensor. */
                        float Kp = -43.57f;
                        float Kd = 0.0f;
                        float thermal_correction = (Kp * delta_T) + (Kd * temp_rate_of_change);

                        /* Clamp sized for the sealed-cavity correction: full
                           self-heating from a cold boot spans ~20 K ≈ 870 m.
                           Anything beyond is a runaway temperature read, not a
                           plausible correction. */
                        if (thermal_correction >  900.0f) thermal_correction =  900.0f;
                        if (thermal_correction < -900.0f) thermal_correction = -900.0f;

                        latest_baro.height = raw_altitude - thermal_correction;
                        current_height = latest_baro.height;

                        sprintf(latest_baro.timestamp, "%lu", current_ms);
                    }
                    break;
                }

                /* --- LiDAR: DMA ping-pong, first half -------------------------*/
                case EVENT_LIDAR_HALF_CPLT:
                {
                    Process_Lidar_Buffer_Chunk(&lidar_dma_buf[0],
                                               DMA_BUFFER_SIZE / 2, current_ms);
                    break;
                }

                /* --- LiDAR: DMA ping-pong, second half ------------------------*/
                case EVENT_LIDAR_FULL_CPLT:
                {
                    Process_Lidar_Buffer_Chunk(&lidar_dma_buf[DMA_BUFFER_SIZE / 2],
                                               DMA_BUFFER_SIZE / 2, current_ms);
                    break;
                }

                /* --- IMU: trigger non-blocking DMA burst read (100 Hz) --------*/
                case EVENT_IMU_READY:
                {
                    /* 34 bytes from BNO055_GYRO_DATA_X_LSB (0x14); result is
                       decoded in HAL_I2C_MemRxCpltCallback. */
                    if (IMU_RequestData_DMA() == 0 && debug) {
                        extern UART_HandleTypeDef huart1;
                        char sensor_msg[] = "[-] DMA I2C Start Failed!\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 10);
                    }
                    break;
                }

                /* --- GNSS: copy latest parsed fix into the global struct (1 Hz) */
                case EVENT_GNSS_READY:
                {
                    latest_gnss.latitude   = parsed_gnss.latitude;
                    latest_gnss.longitude  = parsed_gnss.longitude;
                    latest_gnss.satellites = parsed_gnss.satellites;
                    latest_gnss.altitude   = parsed_gnss.altitude;
                    break;
                }

                /* --- Battery: ADC single conversion (1 Hz) --------------------*/
                case EVENT_BATTERY_READY:
                {
                    HAL_ADC_Start(&hadc1);
                    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                        uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
                        latest_battery.voltage =
                                ((float)raw_adc / 4095.0f) * 3.3f * 3.0f * 1.025f;
                    }
                    HAL_ADC_Stop(&hadc1);
                    break;
                }
            }
        }
    }
  /* USER CODE END startTaskSensors */
}

/* USER CODE BEGIN Header_startTaskSDCard */
/**
* @brief Function implementing the TaskSDCard thread.
* @param argument: Not used
* @retval None
*/
uint8_t is_sd_inserted(void) {
    return (HAL_GPIO_ReadPin(SD_GPIO_DETECT_GPIO_Port, SD_GPIO_DETECT_Pin) == GPIO_PIN_RESET);
}
/* USER CODE END Header_startTaskSDCard */
void startTaskSDCard(void *argument)
{
  /* USER CODE BEGIN startTaskSDCard */
    extern GNSS_Data_t latest_gnss;
    extern Barometer_Data_t latest_baro;
    extern IMU_Data_t latest_imu;
    extern CanSatState_t currentState;

    /* =========================================================================
     * SD CARD INITIALIZATION
     * ========================================================================= */
    if (debug) {
        if (is_sd_inserted()) { char m[] = "[*] SD Card Inserted\r\n";     HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100); }
        else                  { char m[] = "[*] SD Card NOT Inserted\r\n"; HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100); }
    }

    char SD_carriage[] = "\r";

    extern char data_filename[];
    extern char lidar_filename[];
    extern uint8_t flight_session;

    if (is_sd_inserted()) {
        Mount_SD("/");

        /* Find the next unused session number by scanning for existing DATA_xxx.CSV files */
        flight_session = 1;
        while (flight_session < 999) {
            sprintf(data_filename, "DATA_%03d.CSV", flight_session);
            FILINFO check_fno;
            if (f_stat(data_filename, &check_fno) != FR_OK) {
                break;
            }
            flight_session++;
        }

        sprintf(data_filename,  "DATA_%03d.CSV", flight_session);
        sprintf(lidar_filename, "LIDA_%03d.CSV", flight_session);

        Create_File(data_filename);
        Update_File(data_filename,
            "tx_timestamp_ms,accel_x,accel_y,accel_z,"
            "gyro_x,gyro_y,gyro_z,roll,pitch,yaw,"
            "temperature,altitude,latitude,longitude,"
            "satellites,flags_raw,battery_voltage,gnss_alt\r\n");
        osDelay(10);

        Create_File(lidar_filename);
        Update_File(lidar_filename,
            "tx_timestamp_ms,distance,roll,pitch,yaw,"
            "quat_w,quat_x,quat_y,quat_z,"
            "accel_x,accel_y,accel_z,"
            "latitude,longitude,altitude,flags_raw,gnss_alt\r\n");

        if (debug) {
            char boot_msg[64];
            sprintf(boot_msg, "[SD] Session %d: %s / %s\r\n",
                    flight_session, data_filename, lidar_filename);
            HAL_UART_Transmit(&huart1, (uint8_t*)boot_msg, strlen(boot_msg), 100);
        }
    }

    if (debug) {
        char SD_end_msg[] = "[SD] Ready for logging\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)SD_end_msg, strlen(SD_end_msg), 100);
    }

    TelemetryPacket_t pkt;
    LidarPacket_t fast_pkt;
    char csv_buffer[256];
    /* lidar_buffer grown from 128: quat_w/x/y/z at 4 decimal places pushes
       the worst-case line past 128 bytes. */
    char lidar_buffer[192];

    /* =========================================================================
     * MAIN LOGGING LOOP
     * ========================================================================= */
   for(;;)
    {
       static uint8_t sd_was_inserted = 0;

       /* Detect SD card removal and close the log files cleanly */
       if (sd_was_inserted && !is_sd_inserted()) {
           if (lidar_file_open) {
               f_sync(&fil_lidar);
               f_close(&fil_lidar);
               lidar_file_open = 0;
           }
           if (data_file_open) {
               f_sync(&fil_data);
               f_close(&fil_data);
               data_file_open = 0;
           }
           f_mount(NULL, "/", 0);
           sd_was_inserted = 0;
       }

       if (is_sd_inserted()) {
           sd_was_inserted = 1;
       }

       /* --- Deferred end-of-flight close ----------------------------------
          LOG_CLOSE_AFTER_RECOVERY_MS after entering RECOVERY, sync and close
          both log files and latch logging_stopped so neither write path
          reopens them (packets keep flowing in RECOVERY). From that point the
          card can be pulled or the power cut with no risk of a torn FAT /
          directory write corrupting the flight data. */
       static uint32_t recovery_entry_ms = 0;
       static uint8_t  logging_stopped   = 0;
       if (!logging_stopped) {
           if (currentState == STATE_RECOVERY) {
               if (recovery_entry_ms == 0) {
                   recovery_entry_ms = HAL_GetTick();
               } else if (HAL_GetTick() - recovery_entry_ms >= LOG_CLOSE_AFTER_RECOVERY_MS) {
                   if (lidar_file_open) {
                       f_sync(&fil_lidar);
                       f_close(&fil_lidar);
                       lidar_file_open = 0;
                   }
                   if (data_file_open) {
                       f_sync(&fil_data);
                       f_close(&fil_data);
                       data_file_open = 0;
                   }
                   logging_stopped = 1;
               }
           } else {
               recovery_entry_ms = 0;
           }
       }

       /* --- High-frequency LIDAR logging (up to 50 Hz) --------------------*/
       if (uxQueueMessagesWaiting(qSDCard_LIDAR) > 0 && is_sd_inserted()) {

           /* Keep the LIDAR file open for the entire flight to avoid the cost
              of f_open/f_close on every burst at 50 Hz. */
           if (currentState >= STATE_READY && !lidar_file_open && !logging_stopped && is_sd_inserted()) {
               if (f_open(&fil_lidar, lidar_filename, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
                   f_lseek(&fil_lidar, f_size(&fil_lidar));
                   lidar_file_open = 1;
               } else {
                   sd_open_errors++;
               }
           }

           /* Logging deliberately continues through STATE_RECOVERY: the LW20
              keeps streaming and the parser keeps enqueuing regardless, so
              stopping the writes saves almost nothing (the old close-here
              logic even re-opened/re-closed the file every loop iteration,
              keeping the SD just as busy). f_sync after each burst already
              makes the file safe against power loss. */
           if (lidar_file_open && uxQueueMessagesWaiting(qSDCard_LIDAR) > 0) {

               UINT bytes_written;

               while (xQueueReceive(qSDCard_LIDAR, &fast_pkt, 0) == pdTRUE) {

                   uint8_t lidar_flags = 0;
                   if (currentState >= STATE_READY)     lidar_flags |= 0x01;
                   if (currentState >= STATE_ASCENSION) lidar_flags |= 0x02;
                   if (currentState >= STATE_DROP)      lidar_flags |= 0x04;
                   if (currentState >= STATE_RECOVERY)  lidar_flags |= 0x08;

                   char tmp[16];
                   int offset = 0;

                   offset += sprintf(lidar_buffer + offset, "%s,", fast_pkt.lidar.timestamp);

                   float_to_str(fast_pkt.lidar.distance, 2, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.roll, 1, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.pitch, 1, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.yaw, 1, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.quat_w, 4, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.quat_x, 4, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.quat_y, 4, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.quat_z, 4, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.accelX, 2, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.accelY, 2, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.accelZ, 2, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.latitude, 6, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.longitude, 6, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   float_to_str(fast_pkt.height, 2, tmp);
                   offset += sprintf(lidar_buffer + offset, "%s,", tmp);

                   offset += sprintf(lidar_buffer + offset, "%d,", lidar_flags);

                   /* GNSS altitude (MSL, 10 Hz) appended last so existing
                      column indices in the analysis tools keep working. */
                   float_to_str(fast_pkt.gnss_altitude, 2, tmp);
                   sprintf(lidar_buffer + offset, "%s\r\n", tmp);

                   if (f_write(&fil_lidar, lidar_buffer, strlen(lidar_buffer), &bytes_written) != FR_OK
                       || bytes_written < strlen(lidar_buffer)) {
                       /* Close so the next burst re-opens through the directory
                          (FA_OPEN_ALWAYS) instead of pushing into a dead handle. */
                       sd_write_errors++;
                       f_close(&fil_lidar);
                       lidar_file_open = 0;
                       break;
                   }
               }

               if (lidar_file_open && f_sync(&fil_lidar) != FR_OK) {
                   sd_write_errors++;
                   f_close(&fil_lidar);
                   lidar_file_open = 0;
               }
           }
       }

       /* --- Low-frequency telemetry logging (DATA.CSV) --------------------*/
       if (xQueueReceive(qSDCard, &pkt, 10) == pdTRUE) {
           if (is_sd_inserted()) {

               char* time_val = (strlen(pkt.baro.timestamp) > 0) ? pkt.baro.timestamp : "0";

               uint8_t flags = 0;
               if (currentState >= STATE_READY)     flags |= 0x01;
               if (currentState >= STATE_ASCENSION) flags |= 0x02;
               if (currentState >= STATE_DROP)      flags |= 0x04;
               if (currentState >= STATE_RECOVERY)  flags |= 0x08;

               char tmp[16];
               int offset = 0;

               offset += sprintf(csv_buffer + offset, "%s,", time_val);

               float_to_str(pkt.imu.accelX, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.imu.accelY, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.imu.accelZ, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);

               float_to_str(pkt.imu.gyroX, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.imu.gyroY, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.imu.gyroZ, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);

               float_to_str(pkt.imu.roll, 1, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.imu.pitch, 1, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.imu.yaw, 1, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);

               float_to_str(pkt.baro.temperature, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.baro.height, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);

               float_to_str(pkt.gnss.latitude, 6, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);
               float_to_str(pkt.gnss.longitude, 6, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);

               offset += sprintf(csv_buffer + offset, "%d,%d,", pkt.gnss.satellites, flags);

               float_to_str(pkt.bat.voltage, 2, tmp);
               offset += sprintf(csv_buffer + offset, "%s,", tmp);

               /* GNSS altitude (MSL, metres) appended last so existing column
                  indices in the analysis tools keep working. */
               float_to_str(pkt.gnss.altitude, 2, tmp);
               sprintf(csv_buffer + offset, "%s\r\n", tmp);

               /* Keep the DATA file open for the entire flight, like the
                  LIDAR file: Update_File's f_open/f_close per line was slow
                  enough (with the LiDAR sharing the SD bus) to back up
                  qSDCard by seconds and desync timestamps from flags. */
               if (!data_file_open && !logging_stopped) {
                   if (f_open(&fil_data, data_filename, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
                       f_lseek(&fil_data, f_size(&fil_data));
                       data_file_open = 1;
                   } else {
                       sd_open_errors++;
                   }
               }
               if (data_file_open) {
                   UINT data_bytes_written;
                   if (f_write(&fil_data, csv_buffer, strlen(csv_buffer), &data_bytes_written) != FR_OK
                       || data_bytes_written < strlen(csv_buffer)
                       || f_sync(&fil_data) != FR_OK) {
                       sd_write_errors++;
                       f_close(&fil_data);
                       data_file_open = 0;
                   }
               }
               if (debug) HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);
           }
       }
    }

  /* USER CODE END startTaskSDCard */
}

/* USER CODE BEGIN Header_startTaskLoRa */
/**
* @brief Function implementing the TaskLoRa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskLoRa */
void startTaskLoRa(void *argument)
{
  /* USER CODE BEGIN startTaskLoRa */
    extern UART_HandleTypeDef huart1;
    extern ADC_HandleTypeDef hadc1;
    char payload_str[256];
    TelemetryPacket_t pkt;

    extern uint32_t calibration_duration_ms;

    /* =========================================================================
     * LORA INITIALIZATION
     * ========================================================================= */
    SX1276_Init();

    /* Broadcast calibration metadata so the ground station knows the reference
       pressure was acquired and how long it took. */
    SX1276_SendCalibrationPacket(calibration_duration_ms);

    /* =========================================================================
     * TELEMETRY TRANSMIT LOOP
     * ========================================================================= */
    for(;;)
    {
        if (xQueueReceive(qLoRa, &pkt, portMAX_DELAY) == pdTRUE) {

            char* time_val = (strlen(pkt.baro.timestamp) > 0) ? pkt.baro.timestamp : "0";

            /* Encode FSM state as a bitmask for the ground station */
            uint8_t flags = 0;
            if (currentState >= STATE_READY)     flags |= 0x01;  /* Bit 0: GO_FOR_LAUNCH */
            if (currentState >= STATE_ASCENSION) flags |= 0x02;  /* Bit 1: ASCENSION     */
            if (currentState >= STATE_DROP)      flags |= 0x04;  /* Bit 2: DROP          */
            if (currentState >= STATE_RECOVERY)  flags |= 0x08;  /* Bit 3: RECOVERY      */

            char tmp[16];
            int offset = 0;

            float_to_str(pkt.imu.accelX, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.accelY, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.accelZ, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            float_to_str(pkt.imu.gyroX, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.gyroY, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.gyroZ, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            float_to_str(pkt.imu.roll, 1, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.pitch, 1, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.yaw, 1, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            float_to_str(pkt.baro.temperature, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.baro.height, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            float_to_str(pkt.gnss.latitude, 6, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.gnss.longitude, 6, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            offset += sprintf(payload_str + offset, "%d,%d,%s,",
                              pkt.gnss.satellites, flags, time_val);

            float_to_str(pkt.bat.voltage, 2, tmp);
            sprintf(payload_str + offset, "%s\r\n", tmp);

            SX1276_SendPacket((uint8_t*)payload_str, strlen(payload_str));
            osDelay(10);
        }
    }
  /* USER CODE END startTaskLoRa */
}

/* USER CODE BEGIN Header_startTaskHMI */
/**
* @brief Function implementing the TaskHMIHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskHMI */
void startTaskHMI(void *argument)
{
  /* USER CODE BEGIN startTaskHMI */

    /* =========================================================================
     * HMI INITIALIZATION
     * ========================================================================= */
    /* SSD1306 and the MCU start their own power-up sequencing at the same
       instant on a cold boot; the display's internal regulator/reset can
       need a moment before it will ACK on I2C. Retry instead of one shot
       + permanent suspend. */
    osDelay(50);
    uint8_t ssd1306_ready = 0;
    for (uint8_t attempt = 0; attempt < 5 && !ssd1306_ready; attempt++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 3, 100) == HAL_OK) {
            ssd1306_ready = 1;
        } else {
            osDelay(50);
        }
    }
    if (!ssd1306_ready) {
        vTaskSuspend(NULL);
    }

    HMI_State_t hmi_state = {
            .current_page = HMI_PAGE_OVERVIEW,
            .cursor_position = 0,
            .lora_enabled = 0,
            .format_sd_requested = 0,
            .recalib_requested = 0
    };

    ssd1306_Init();

    /* Boot logo, centered (64x64 logo on a 128x64 screen -> x=32, y=0),
       shown briefly before the existing text splash. */
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(32, 0, vortex_logo_bitmap, VORTEX_LOGO_WIDTH, VORTEX_LOGO_HEIGHT, White);
    HMI_safe_update_screen();
    osDelay(1500);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("CanSat HMI", Font_11x18, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("Initializing...", Font_7x10, White);
    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("SBC-OLED01", Font_7x10, White);
    HMI_safe_update_screen();
    osDelay(1500);

    uint8_t event;

    /* =========================================================================
     * HMI EVENT LOOP  —  active only during STATE_CONFIG
     * ========================================================================= */
    for(;;)
    {
        extern CanSatState_t currentState;

        if (currentState == STATE_CONFIG) {

            /* ========== AUTO-CALIBRATION ON CONFIG ENTRY ========== */
            if (!is_calibrated) {
                /* Runs once on entering config, sharing the same display calls
                   and calibration routine as the manual recalib menu below.
                   Kept in TaskHMI, not TaskFSM: the SSD1306 framebuffer isn't
                   mutex-protected (only the I2C1 transmit step is), so only
                   the task that owns the display should draw to it during
                   this ~8s blocking call. */
                extern BMP_t   bmp_sensor;
                extern double  reference_pressure_Pa;
                extern double  reference_temp_C;
                extern UART_HandleTypeDef huart1;

                HMI_display_recalib(&hmi_state);

                if (debug) {
                    char m[] = "\r\n[*] Calibrating Barometer (Do not move)...\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), 100);
                }

                uint32_t start_time = HAL_GetTick();
                if (BMP581_CalibrateGroundPressure(&reference_pressure_Pa, &reference_temp_C,
                                                    &bmp_sensor, HMI_display_recalib_progress) == HAL_OK) {
                    calibration_duration_ms = HAL_GetTick() - start_time;
                } else {
                    reference_pressure_Pa = 101325.0;
                    reference_temp_C = 25.0;
                    calibration_duration_ms = 0;
                }

                HMI_display_recalib_progress(100);
                osDelay(300);
                HMI_display_recalib_done((float)reference_pressure_Pa, (float)reference_temp_C);
                osDelay(3000);  /* let the operator read the result before falling back to overview */

                is_calibrated = 1;
                hmi_state.current_page = HMI_PAGE_OVERVIEW;
            }

            /* Refresh HMI_display_data from the live globals every iteration.
               baro/imu/lidar "ready" reuse existing globals (no new sensor
               activity); GNSS updates during CONFIG too via the 1Hz tick in
               startTaskFSM. */
            extern Battery_Data_t latest_battery;
            extern GNSS_Data_t    latest_gnss;
            extern LIDAR_Data_t   latest_lidar;
            extern uint8_t        is_calibrated;
            extern uint8_t        imu_is_connected;

            HMI_display_data.battery_voltage = latest_battery.voltage;
            HMI_display_data.battery_percent = calculate_battery_percent(latest_battery.voltage);
            HMI_display_data.baro_ready      = is_calibrated;
            HMI_display_data.imu_ready       = imu_is_connected;
            HMI_display_data.gnss_ready      = (latest_gnss.satellites > 0);
            HMI_display_data.gnss_satellites = latest_gnss.satellites;
            /* "Ready" means distance lines are flowing right now (the CONFIG
               stream watchdog in startTaskFSM keeps retrying), not just "seen
               once since boot" — a dead stream falls back to N/A instead of
               freezing the last distance on screen. */
            HMI_display_data.lidar_ready     = (lidar_last_data_ms != 0) &&
                                               (HAL_GetTick() - lidar_last_data_ms < 2000);
            HMI_display_data.lidar_distance  = latest_lidar.distance;

            /* ========== HANDLE RECALIBRATION REQUEST ========== */
            if (hmi_state.current_page == HMI_PAGE_RECALIB) {
                extern BMP_t bmp_sensor;
                extern double reference_pressure_Pa;
                extern double reference_temp_C;
                extern uint8_t is_calibrated;

                HMI_display_recalib(&hmi_state);

                if (BMP581_CalibrateGroundPressure(&reference_pressure_Pa,
                                                    &reference_temp_C, &bmp_sensor,
                                                    HMI_display_recalib_progress) == HAL_OK) {
                    is_calibrated = 1;
                }

                HMI_display_recalib_progress(100);
                osDelay(300);
                HMI_display_recalib_done((float)reference_pressure_Pa, (float)reference_temp_C);

                uint8_t dummy_event;
                xQueueReceive(qHMI_Events, &dummy_event, portMAX_DELAY);
                hmi_state.current_page = HMI_PAGE_MENU;
                hmi_state.cursor_position = 0;
                continue;
            }

            /* ========== HANDLE FORMAT SD REQUEST ========== */
            if (hmi_state.format_sd_requested) {
                uint8_t full_format = (hmi_state.format_sd_requested == 1);
                hmi_state.format_sd_requested = 0;

                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 10);
                ssd1306_WriteString(full_format ? "Formatting..." : "Erasing...", Font_11x18, White);
                ssd1306_SetCursor(0, 36);
                ssd1306_WriteString("Do not remove SD", Font_6x8, White);
                HMI_safe_update_screen();

                /* Close any open log file first: after Format_SD wipes the
                   FAT, a write through a stale FIL handle would corrupt the
                   fresh filesystem. Files can be open here after a READY ->
                   CONFIG re-entry; by the time the user has navigated the
                   menu and confirmed, TaskSDCard has long drained its queues
                   (no new packets are enqueued during CONFIG) so it is not
                   mid-write. */
                extern FIL fil_lidar, fil_data;
                extern uint8_t lidar_file_open, data_file_open;
                if (lidar_file_open) { f_close(&fil_lidar); lidar_file_open = 0; }
                if (data_file_open)  { f_close(&fil_data);  data_file_open  = 0; }

                FRESULT format_res = full_format ? Format_SD() : Quick_Erase_SD();

                if (format_res == FR_OK) {
                    /* Reset to session 1 and recreate empty CSV files */
                    extern uint8_t flight_session;
                    extern char data_filename[];
                    extern char lidar_filename[];
                    flight_session = 1;

                    sprintf(data_filename,  "DATA_%03d.CSV", flight_session);
                    sprintf(lidar_filename, "LIDA_%03d.CSV", flight_session);

                    Create_File(data_filename);
                    Update_File(data_filename,
                        "tx_timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
                        "roll,pitch,yaw,temperature,altitude,latitude,longitude,"
                        "satellites,flags_raw,battery_voltage,gnss_alt\r\n");
                    osDelay(10);
                    Create_File(lidar_filename);
                    Update_File(lidar_filename,
                        "tx_timestamp_ms,distance,roll,pitch,yaw,"
                        "quat_w,quat_x,quat_y,quat_z,accel_x,accel_y,accel_z,"
                        "latitude,longitude,altitude,flags_raw,gnss_alt\r\n");

                    ssd1306_Fill(Black);
                    ssd1306_SetCursor(10, 10);
                    ssd1306_WriteString(full_format ? "Format OK!" : "Erase OK!", Font_11x18, White);
                    ssd1306_SetCursor(0, 36);
                    ssd1306_WriteString("Files recreated", Font_6x8, White);
                } else {
                    /* A failed operation was previously reported as "Format
                       OK!", hiding an unusable card until the flight data
                       came back empty. A failed QUICK erase usually means a
                       corrupted FAT: retry with the full format. */
                    ssd1306_Fill(Black);
                    ssd1306_SetCursor(10, 10);
                    ssd1306_WriteString(full_format ? "Format FAIL" : "Erase FAIL", Font_11x18, White);
                    ssd1306_SetCursor(0, 36);
                    ssd1306_WriteString(full_format ? "Check/replace SD" : "Try full format", Font_6x8, White);
                }
                HMI_safe_update_screen();
                osDelay(2000);

                hmi_state.current_page = HMI_PAGE_MENU;
                hmi_state.cursor_position = 0;
                continue;
            }

            /* ========== NORMAL BUTTON HANDLING ========== */
            if (xQueueReceive(qHMI_Events, &event, pdMS_TO_TICKS(500)) == pdTRUE) {
                HMI_handle_button(&hmi_state);
            }

            /* ========== DISPLAY CURRENT PAGE ========== */
            switch (hmi_state.current_page) {
                case HMI_PAGE_OVERVIEW:   HMI_display_overview(&hmi_state);  break;
                case HMI_PAGE_MENU:       HMI_display_menu(&hmi_state);      break;
                case HMI_PAGE_LORA:       HMI_display_lora(&hmi_state);      break;
                case HMI_PAGE_SENSORS:    HMI_display_sensors(&hmi_state);   break;
                case HMI_PAGE_BATTERY:    HMI_display_battery(&hmi_state);   break;
                case HMI_PAGE_FORMAT_SD:  HMI_display_format_sd(&hmi_state); break;
                case HMI_PAGE_RECALIB:    break;  /* Handled above */
            }

            if (osMutexAcquire(I2C1_MutexHandle, osWaitForever) == osOK) {
                HMI_safe_update_screen();
                osMutexRelease(I2C1_MutexHandle);
            }

        } else {
            /* HMI is only active in CONFIG mode. Show a "screen is off on
               purpose" handoff page before suspending, so a frozen menu isn't
               mistaken for a crash. vTaskSuspend(NULL) never returns (nothing
               calls vTaskResume() on this task), so this only ever runs once,
               on the CONFIG -> READY transition. */
            HMI_display_flight_mode();
            vTaskSuspend(NULL);
            osDelay(100);
        }

        osDelay(100);
    }
  /* USER CODE END startTaskHMI */
}

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
    FatFsCnt++;
    if(FatFsCnt >= 10)
    {
      FatFsCnt = 0;
      SDTimer_Handler();
    }
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
