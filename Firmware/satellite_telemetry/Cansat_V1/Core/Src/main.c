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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "math.h"

// Devices
#include "bmp581.h"
#include "lidar.h"
#include "cansat_core.h"
#include "imu.h"
#include "sx1276.h"
#include "gnss_reader.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SX_NSS_LOW()   HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_RESET)
#define SX_NSS_HIGH()  HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_SET)
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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for TaskFSM */
osThreadId_t TaskFSMHandle;
const osThreadAttr_t TaskFSM_attributes = {
  .name = "TaskFSM",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for TaskSensors */
osThreadId_t TaskSensorsHandle;
const osThreadAttr_t TaskSensors_attributes = {
  .name = "TaskSensors",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for TaskSDCard */
osThreadId_t TaskSDCardHandle;
const osThreadAttr_t TaskSDCard_attributes = {
  .name = "TaskSDCard",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for TaskLoRa */
osThreadId_t TaskLoRaHandle;
const osThreadAttr_t TaskLoRa_attributes = {
  .name = "TaskLoRa",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* USER CODE BEGIN PV */
uint8_t TX_to_Baro [] = "A" ;
uint8_t RX_from_Baro [] = "A" ;

// --- GLOBAL VARIABLES (Accessible by all tasks) ---
uint8_t configFlag = 1; // Triggered by external switch/button
CanSatState_t currentState = STATE_STANDBY;
float current_height = 0.0f;
uint8_t imu_is_connected = 0;
uint32_t calibration_duration_ms = 0;

// Global Sensor Data
GNSS_Data_t latest_gnss;
Barometer_Data_t latest_baro;
IMU_Data_t latest_imu;
LIDAR_Data_t latest_lidar = { .distance = 99.0f };
BMP_t bmp_sensor;

// Calibration Variables
double reference_pressure_Pa = 101325.0;
double reference_temp_C = 25.0;
double PRESS_TEMP_COEF = -8.5; // Datasheet: Pressure Temperature-induced offset: +-0.5 Pa/K

// Queue Handles
QueueHandle_t qSensorEvents;
QueueHandle_t qSDCard;
QueueHandle_t qLoRa;
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
void startTaskFSM(void *argument);
void startTaskSensors(void *argument);
void startTaskSDCard(void *argument);
void startTaskLoRa(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // --- 1. POWER ON LORA AND SENSORS ---
    HAL_GPIO_WritePin(GPIOB, VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_SET);

    // --- 2. FIX SPI CHIP SELECT (NSS) ---
    // NSS must idle HIGH. If it starts LOW, the SPI bus crashes.
    HAL_GPIO_WritePin(GPIOB, LORA_NSS_Pin, GPIO_PIN_SET);

    char startup_msg[] = "\r\n[i] System Booting... Testing Hardware\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);

    // Give the LoRa module a full second to power up and stabilize
    HAL_Delay(1000);



  // --- BATTERY VOLTAGE CHECK (ADC1) ---
    char adc_msg[128];
    sprintf(adc_msg, "\r\n[*] Testing Power (ADC1)...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)adc_msg, strlen(adc_msg), 100);

    // Start ADC Conversion
    HAL_ADC_Start(&hadc1);

    // Wait up to 100ms for the conversion to finish
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);

        // Calculate voltage directly at the STM32 pin (12-bit ADC, 3.3V ref)
        float pin_voltage = ((float)raw_adc / 4095.0f) * 3.3f;

        // --- VOLTAGE DIVIDER RATIO ---
        // IMPORTANT: Adjust this multiplier to match your physical PCB resistors!
        // Example: A 10k/10k divider cuts voltage in half, so the ratio is 2.0f.
        // Example: A 20k/10k divider cuts voltage to a third, so the ratio is 3.0f.
        float voltage_divider_ratio = 3.0f; // <-- CHANGE THIS TO MATCH YOUR HARDWARE

        float true_battery_voltage = pin_voltage * voltage_divider_ratio*1.025f;

        sprintf(adc_msg, "    -> SUCCESS! Battery: %.2f V (Raw ADC: %lu)\r\n", true_battery_voltage, raw_adc);
        HAL_UART_Transmit(&huart1, (uint8_t*)adc_msg, strlen(adc_msg), 100);
    } else {
        sprintf(adc_msg, "    -> FAILED! ADC Conversion Timeout.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)adc_msg, strlen(adc_msg), 100);
    }

    // Stop the ADC to save power
    HAL_ADC_Stop(&hadc1);
    // ------------------------------------
  // --- I2C SCANNER ---
    char scan_msg[64];
    sprintf(scan_msg, "\r\n[*] Scanning I2C Bus 1...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);

    for(uint8_t i = 1; i < 128; i++) {
        // Shift the address left by 1 for the HAL library
        if(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5) == HAL_OK) {
            sprintf(scan_msg, "    -> Found device at address: 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
        }
    }

    sprintf(scan_msg, "\r\n[*] Scanning I2C Bus 3...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
    for(uint8_t i = 1; i < 128; i++) {
        // Shift the address left by 1 for the HAL library
        if(HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i<<1), 3, 5) == HAL_OK) {
            sprintf(scan_msg, "    -> Found device at address: 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
        }
    }
    sprintf(scan_msg, "[*] Scan Complete.\r\n\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
    // -----------------------------


    // --- UART3 DIAGNOSTIC PING ---
        char diag_msg[128];
        // 1. Print the STM32's configured Baud Rate
        sprintf(diag_msg, "    -> STM32 UART3 BaudRate: %lu\r\n", huart3.Init.BaudRate);
        HAL_UART_Transmit(&huart1, (uint8_t*)diag_msg, strlen(diag_msg), 100);

        // 2. Clear any old data out of the STM32 buffer
        __HAL_UART_FLUSH_DRREGISTER(&huart3);

        // 3. Send a Spacebar (' ') to attempt to wake up the LiDAR menu
        uint8_t test_tx = ' ';
        HAL_UART_Transmit(&huart3, &test_tx, 1, 100);

        // 4. Wait up to 500ms for the LiDAR to reply
        uint8_t test_rx[1] = {0};
        HAL_StatusTypeDef uart_status = HAL_UART_Receive(&huart3, test_rx, 1, 500);

        if (uart_status == HAL_OK) {
            // We received a byte! Let's see if it is a readable ASCII character.
            if (test_rx[0] >= 32 && test_rx[0] <= 126) {
                sprintf(diag_msg, "    -> SUCCESS! Received valid ASCII: '%c' (0x%02X)\r\n\n", test_rx[0], test_rx[0]);
            } else {
                sprintf(diag_msg, "    -> WARNING! Received garbage byte: 0x%02X (Baud rate mismatch?)\r\n\n", test_rx[0]);
            }
            HAL_UART_Transmit(&huart1, (uint8_t*)diag_msg, strlen(diag_msg), 100);
        } else {
            sprintf(diag_msg, "    -> FAILED! No response from LiDAR. (Check TX/RX wiring)\r\n\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)diag_msg, strlen(diag_msg), 100);
        }
        // -----------------------------


  // Initialize Barometer
  if (bmp581_init_precise_normal(&bmp_sensor) == 0) {
	  char baro_msg[] = "[+] BMP581 Initialized Successfully\r\n";
	  HAL_UART_Transmit(&huart1, (uint8_t*)baro_msg, strlen(baro_msg), HAL_MAX_DELAY);
  } else {
	  char baro_msg[] = "[-] BMP581 Initialization FAILED\r\n";
	  HAL_UART_Transmit(&huart1, (uint8_t*)baro_msg, strlen(baro_msg), HAL_MAX_DELAY);
  }

  // Initialize IMU
  if (IMU_Init(&hi2c3) == 1) {
        imu_is_connected = 1; // Mark as successful
        char imu_msg[] = "[+] BNO055 Initialized Successfully\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)imu_msg, strlen(imu_msg), HAL_MAX_DELAY);
    } else {
        imu_is_connected = 0; // Mark as failed
        char imu_msg[] = "[-] BNO055 Initialization FAILED\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)imu_msg, strlen(imu_msg), HAL_MAX_DELAY);
    }

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



	// Initialize background GNSS listening on USART2
	char gnss_init_msg[] = "[*] Arming GNSS Interrupt...\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)gnss_init_msg, strlen(gnss_init_msg), 100);
	GNSS_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    // MUST CREATE THE QUEUES BEFORE STARTING THE TASKS
    qSensorEvents = xQueueCreate(10, sizeof(SensorEvent_t));
    qSDCard       = xQueueCreate(20, sizeof(TelemetryPacket_t));
    qLoRa         = xQueueCreate(5,  sizeof(TelemetryPacket_t));
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_GPIO_TogglePin(PW_Enable_GPIO_Port, PW_Enable_Pin);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_3;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GPIO_OUT_Pin|LORA_NSS_Pin|LORA_RST_Pin|VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BARO_EXTI_Pin */
  GPIO_InitStruct.Pin = BARO_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BARO_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GPIO_OUT_Pin LORA_NSS_Pin LORA_RST_Pin VPOWER_EN_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = LED_GPIO_OUT_Pin|LORA_NSS_Pin|LORA_RST_Pin|VPOWER_EN_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Route UART3 interrupts directly to the LiDAR driver
    if (huart->Instance == USART3) {
        Lidar_RxCallback(huart);
    }
    // Route UART2 interrupts directly to the GNSS driver
    if (huart->Instance == USART1) {
        GNSS_UART_RxCpltCallback(huart);
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

    // Assuming huart1 is PC serial port
    extern UART_HandleTypeDef huart1;
    char debug_msg[128];

    // --- SAFE LIDAR INIT ---
    Lidar_Init(&huart3);
    sprintf(debug_msg, "[+] LiDAR Interrupt Armed (RTOS Safe)\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    // Tracker to prevent UART spam (only print when state actually changes)
    CanSatState_t last_printed_state = (CanSatState_t)-1;

    // JUST FOR FIRST TESTS
    configFlag = 1;

  /* Infinite loop */
  for(;;)
  {
    // DEBUG PRINT LOGIC ---
    if (currentState != last_printed_state) {
        switch(currentState) {
            case STATE_STANDBY:   sprintf(debug_msg, "\r\n[FSM] State: STANDBY\r\n"); break;
            case STATE_CONFIG:    sprintf(debug_msg, "\r\n[FSM] State: CONFIG\r\n"); break;
            case STATE_READY:
                sprintf(debug_msg, "\r\n[FSM] State: READY\r\n");
                // START THE LIDAR DATA STREAM WHEN WE ENTER READY
                Lidar_RequestStream();
                break;
            case STATE_ASCENSION: sprintf(debug_msg, "\r\n[FSM] State: ASCENSION\r\n"); break;
            case STATE_DROP:      sprintf(debug_msg, "\r\n[FSM] State: DROP\r\n"); break;
            case STATE_RECOVERY:  sprintf(debug_msg, "\r\n[FSM] State: RECOVERY\r\n"); break;
            case STATE_OFF:       sprintf(debug_msg, "\r\n[FSM] State: OFF\r\n"); break;
        }
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
        last_printed_state = currentState;
    }

    // STATE TRANSITION LOGIC ---
    switch(currentState) {
        // --------------------------------------------------------------------------------------------
        case STATE_STANDBY:
            if (configFlag == 1) {
                currentState = STATE_CONFIG;
            }
            break;

        // --------------------------------------------------------------------------------------------
            // --------------------------------------------------------------------------------------------
	case STATE_CONFIG:
		{
			if (imu_is_connected) {
				sprintf(debug_msg, "[+] IMU Hardcoded Profile Loaded.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
			} else {
				sprintf(debug_msg, "[-] IMU Not Detected.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
			}

			//Lidar_DirectDebug(&huart1);

			sprintf(debug_msg, "[*] LIDAR configurations:.\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

			sprintf(debug_msg, "\r\n[*] Calibrating Barometer...\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

			// --- DYNAMIC CALIBRATION TRACKING ---
			uint32_t start_time = HAL_GetTick(); // Capture start time

			// Barometer calibration
			if (BMP581_CalibrateGroundPressure(&reference_pressure_Pa, &reference_temp_C, &bmp_sensor) == HAL_OK) {

				// Calculate exact millisecond duration
				calibration_duration_ms = HAL_GetTick() - start_time;
				float elapsed_sec = (float)calibration_duration_ms / 1000.0f;

				sprintf(debug_msg, "[+] Calibration completed: %.2f Pa | %.2f C (Elapsed time %.1f s)\r\n",
						reference_pressure_Pa, reference_temp_C, elapsed_sec);
			} else {
				sprintf(debug_msg, "[-] Calibration Failed! Using defaults.\r\n");
				reference_pressure_Pa = 101325.0;
				reference_temp_C = 25.0;
				calibration_duration_ms = 0; // Send 0 to GUI if it fails
			}
			HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

			// --- GNSS SATELLITE LOCK ---
			GNSS_CalibrateSatellite();



			// ---------------------------

			currentState = STATE_READY;
			break;
		}
        // --------------------------------------------------------------------------------------------
        case STATE_READY:
		{
			// 1. Create and send a ticket for the Barometer
			SensorEvent_t baro_ticket = EVENT_BARO_READY;
			xQueueSend(qSensorEvents, &baro_ticket, 0);


			// 2. Create and send a ticket for the IMU
			SensorEvent_t imu_ticket = EVENT_IMU_READY;
			xQueueSend(qSensorEvents, &imu_ticket, 0);

			// --- LORA TEST TRIGGER ---
			// Send a test packet every 100 milliseconds (0.1 seconds)
			static uint32_t last_lora_tx = 0;
			if (HAL_GetTick() - last_lora_tx > TIME_BETWEEN_PACKET_LORA_mS) {
				last_lora_tx = HAL_GetTick();

				TelemetryPacket_t test_pkt;

				// CRITICAL FIX: Wipe the garbage memory out of the struct!
				memset(&test_pkt, 0, sizeof(TelemetryPacket_t));

				// Pack it with our global variables and some fake GNSS data
				sprintf(test_pkt.baro.timestamp, "%lu", HAL_GetTick());
				test_pkt.gnss.latitude = 49.039506f; // Fake Lat
				test_pkt.gnss.longitude = 2.072491f; // Fake Lon
				test_pkt.baro.height = current_height;
				test_pkt.baro.temperature = latest_baro.temperature;
				test_pkt.imu.pitch = latest_imu.pitch;
				test_pkt.imu.roll = latest_imu.roll;
				test_pkt.imu.head = latest_imu.head;
				test_pkt.imu.accelX = latest_imu.accelX;
				test_pkt.imu.accelY = latest_imu.accelY;
				test_pkt.imu.accelZ = latest_imu.accelZ;
				test_pkt.imu.gyroX = latest_imu.gyroX;
				test_pkt.imu.gyroY = latest_imu.gyroY;
				test_pkt.imu.gyroZ = latest_imu.gyroZ;
				test_pkt.gnss.latitude = latest_gnss.latitude;
				test_pkt.gnss.longitude = latest_gnss.longitude;
				test_pkt.gnss.satellites = latest_gnss.satellites;

				// Drop it in the mailbox! TaskLoRa will wake up instantly.
				xQueueSend(qLoRa, &test_pkt, 0);
			}

			// --- GNSS UPDATE TRIGGER ---
			// Tell the Sensor Task to fetch the latest background GPS data once per second
			static uint32_t last_gnss_fetch = 0;
			if (HAL_GetTick() - last_gnss_fetch > 1000) {
				last_gnss_fetch = HAL_GetTick();
				SensorEvent_t gnss_ticket = EVENT_GNSS_READY;
				xQueueSend(qSensorEvents, &gnss_ticket, 0);
			}
			// -------------------------

			// 2. LAUNCH DETECTION LOGIC
			if (configFlag == 0) {
				currentState = STATE_CONFIG; // Return to config if flag drops
			}
			// If inside box AND altitude is high enough
			else if (latest_lidar.distance < THRESH_LIDAR_IN_BOX_M && current_height > THRESH_ALTITUDE_LAUNCH_M) {
				currentState = STATE_ASCENSION;
			}
			break;
		}
		// --------------------------------------------------------------------------------------------
		case STATE_ASCENSION:
			// Wait for Drop condition (LiDAR shoots past 1 meter, meaning we left the box!)
			if (latest_lidar.distance > THRESH_LIDAR_DEPLOYED_M && current_height > THRESH_ALTITUDE_LAUNCH_M) {
				currentState = STATE_DROP;
			}
			break;

		// --------------------------------------------------------------------------------------------
		case STATE_DROP:
			// Check if we hit the ground
			if (current_height < THRESH_ALTITUDE_LANDING_M) {
				currentState = STATE_RECOVERY;
			}
			break;

        // --------------------------------------------------------------------------------------------
        case STATE_RECOVERY:

            // Deploy buzzer, stop measurements
            break;

        // --------------------------------------------------------------------------------------------
        case STATE_OFF:
            break;
    }

    // Evaluate FSM at 20Hz (every 50ms)
    HAL_GPIO_TogglePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin);
    osDelay(50);
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
    TelemetryPacket_t pkt;
    extern UART_HandleTypeDef huart1;
    char sensor_msg[256];

    /* Infinite loop */
    for(;;)
    {
        // Wait infinitely until an event arrives in the queue
        if (xQueueReceive(qSensorEvents, &current_event, portMAX_DELAY) == pdTRUE) {

            // Grab the global timestamp the exact moment the event is processed
            uint32_t current_ms = HAL_GetTick();

            switch(current_event) {

                // ---------------------------------------------------------
                case EVENT_BARO_READY:
                {
                    extern double bmptemp;
                    extern double bmppress;

                    if (bmp581_read_precise_normal(&bmp_sensor) == 0) {

                        // --- 1. TIME CALCULATION FOR DERIVATIVE ---
                        static uint32_t prev_time = 0;
                        float dt_sec = (current_ms - prev_time) / 1000.0f;
                        if (dt_sec <= 0.0f) dt_sec = 0.05f; // Prevent divide by zero on first loop
                        prev_time = current_ms;

                        // --- 2. TEMPERATURE DIFFERENCE (Delta T) ---
                        static float prev_temp = 0.0f;
                        latest_baro.temperature = (float)bmptemp;

                        float delta_T = latest_baro.temperature - (float)reference_temp_C;
                        float temp_rate_of_change = (latest_baro.temperature - prev_temp) / dt_sec;
                        prev_temp = latest_baro.temperature;

                        // --- 3. RAW ALTITUDE ---
                        float raw_altitude = 44330.0f * (1.0f - pow((float)(bmppress / reference_pressure_Pa), (1.0f / 5.255f)));

                        // --- 4. THE FEED-FORWARD CONTROLLER ---
                        float Kp = 0.07f;  // Meters of drift per 1°C difference
                        float Kd = 0.05f;  // Reaction strength to sudden thermal spikes
                        float thermal_correction = (Kp * delta_T) + (Kd * temp_rate_of_change);

                        // --- 5. THE SUMMING JUNCTION ---
                        float comp_altitude = raw_altitude - thermal_correction;

                        // Save the final compensated value
                        latest_baro.height = comp_altitude;
                        current_height = latest_baro.height;

                        // --- 6. APPLY TIMESTAMP TO STRUCT ---
                        sprintf(latest_baro.timestamp, "%lu", current_ms);

                        // --- DEBUG PRINT ---
//                        sprintf(sensor_msg, "[BAR | %s] Height: %.2fm | Temperature: %.2fC\r\n",
//                                latest_baro.timestamp, latest_baro.height, latest_baro.temperature);
//                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
//
//                    } else {
//                        sprintf(sensor_msg, "[BAR | ERROR] Error reading data!\r\n");
//                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                    }
                    break;
                }

                // ---------------------------------------------------------
                case EVENT_LIDAR_READY:
                {
                    extern volatile uint8_t lidar_stream_mode;
                    extern char lidar_rx_buffer[];

                    if (lidar_stream_mode == 0) {
                        // MENU MODE: We are in CONFIG. Echo the LiDAR menu to the PC.
                        sprintf(sensor_msg, "%s\r\n", lidar_rx_buffer);
                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                    } else {

                        // STREAM MODE: We are flying. Parse the string into floats!
                        float angle_dummy, dist_val;

                        // Parse format: "0.00  15.23 ..."
                        if (sscanf(lidar_rx_buffer, "%f %f", &angle_dummy, &dist_val) >= 2) {
                            latest_lidar.distance = dist_val;

                            // --- APPLY TIMESTAMP TO STRUCT ---
                            sprintf(latest_lidar.timestamp, "%lu", current_ms);

                            // --- DEBUG PRINT: Print Lidar Distance ---
                            // Decimate to ~5Hz so it doesn't flood the terminal
                            static uint8_t print_counter = 0;
                            if (++print_counter >= 10) {
                                sprintf(sensor_msg, "[LID | %s] Distance: %5.2f m | In Box? %s\r\n",
                                        latest_lidar.timestamp,
                                        latest_lidar.distance,
                                        (latest_lidar.distance < THRESH_LIDAR_IN_BOX_M) ? "YES" : "NO ");
                                HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                                print_counter = 0;
                            }
                        }

                        // Data Logging
                        if (currentState == STATE_DROP) {
                            pkt.lidar = latest_lidar;
                            pkt.imu = latest_imu;
                            xQueueSend(qSDCard, &pkt, 0);
                        }
                    }
                    break;
                }

                // ---------------------------------------------------------
                case EVENT_IMU_READY:
                {
                    if (getOrientationIMU(&latest_imu) == 1) {
                        // Apply Timestamp
                        sprintf(latest_imu.timestamp, "%lu", current_ms);

//                        sprintf(sensor_msg, "[IMU | %s] H:%6.1f | P:%6.1f | R:%6.1f || ax:%5.2f | ay:%5.2f | az:%5.2f\r\n",
//                                latest_imu.timestamp,
//                                latest_imu.head, latest_imu.pitch, latest_imu.roll,
//                                latest_imu.accelX, latest_imu.accelY, latest_imu.accelZ);
//                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                    } else {
                        sprintf(sensor_msg, "[-] IMU I2C Read Failed!\r\n");
                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                    }
                    break;
                }

                // ---------------------------------------------------------
				case EVENT_GNSS_READY:
				{
					// Copy background parsed data into the global RTOS struct
					if (parsed_gnss.fixed) {
						latest_gnss.latitude = parsed_gnss.latitude;
						latest_gnss.longitude = parsed_gnss.longitude;
						latest_gnss.satellites = parsed_gnss.satellites;

//						sprintf(sensor_msg, "[GNSS | %lu] Lat: %.6f | Lon: %.6f | Sats: %d\r\n",
//								current_ms, latest_gnss.latitude, latest_gnss.longitude, parsed_gnss.satellites);
					} else {
//						sprintf(sensor_msg, "[GNSS | %lu] Searching for satellites... (Sats: %d)\r\n",
//								current_ms, parsed_gnss.satellites);
					}
//					HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);

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
/* USER CODE END Header_startTaskSDCard */
void startTaskSDCard(void *argument)
{
  /* USER CODE BEGIN startTaskSDCard */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
    extern ADC_HandleTypeDef hadc1; // Bring in the ADC handle for battery reading
    //char lora_msg[300];
    char payload_str[256];
    TelemetryPacket_t pkt;

    // We must declare this so the LoRa task can see it
    extern uint32_t calibration_duration_ms;

    // 1. Initialize the LoRa Module
    SX1276_Init();

    // Explicitly lock in the Python Sync Word just in case
    SX1276_WriteRegister(REG_SYNC_WORD, 0x12);

    // --- WAIT FOR FSM CALIBRATION AND BROADCAST GNSS STATUS ---
        // Broadcast the full 17-value packet so the Python GUI populates the map and satellite counts!
        while (currentState < STATE_READY) {

            // Read Live Battery
            float vbat = 0.0f;
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                vbat = ((float)HAL_ADC_GetValue(&hadc1) / 4095.0f) * 3.3f * 3.0f * 1.025f;
            }
            HAL_ADC_Stop(&hadc1);

            // Build the standard packet, putting "0" for the flight flags and injecting live GPS
            sprintf(payload_str, "0.00,0.00,0.00,0.00,0.00,0.00,0.0,0.0,0.0,%.2f,%.2f,%.6f,%.6f,0,%lu,%.2f,%d\r\n",
                    latest_baro.temperature, current_height,
                    parsed_gnss.latitude, parsed_gnss.longitude,
                    HAL_GetTick(), vbat, parsed_gnss.satellites);

            SX1276_SendPacket((uint8_t*)payload_str, strlen(payload_str));

            osDelay(1000); // 1 Hz refresh rate during standby
        }

        // --- SEND DYNAMIC CALIBRATION PACKET ---
        // The FSM has locked 4+ satellites and entered STATE_READY. Fire the CAL packet!
        SX1276_SendCalibrationPacket(calibration_duration_ms);

    /* Infinite loop */
    for(;;)
    {
        // Wait infinitely for a fully-populated packet to arrive from the Sensor Task
        if (xQueueReceive(qLoRa, &pkt, portMAX_DELAY) == pdTRUE) {

            // --- READ LIVE BATTERY VOLTAGE (ADC1) ---
            float vbat = 0.0f;
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
                vbat = ((float)raw_adc / 4095.0f) * 3.3f * 3.0f * 1.025f;
            }
            HAL_ADC_Stop(&hadc1);
            // ----------------------------------------

            // Ensure timestamp isn't empty (send "0" if it is)
            char* time_val = (strlen(pkt.baro.timestamp) > 0) ? pkt.baro.timestamp : "0";

            // --- TRANSLATE FSM STATE TO PYTHON FLAGS ---
            uint8_t flags = 0;
            if (currentState >= STATE_READY)     flags |= 0x01; // Bit 0: GO_FOR_LAUNCH
            if (currentState >= STATE_ASCENSION) flags |= 0x02; // Bit 1: ASCENSION
            if (currentState >= STATE_DROP)      flags |= 0x04; // Bit 2: DROP
            if (currentState >= STATE_RECOVERY)  flags |= 0x08; // Bit 3: RECOVERY

            // --- FORMAT 15-VALUE PYTHON PROTOCOL V2.2 + VBAT ---
            // --- FORMAT 17-VALUE PYTHON PROTOCOL V2.3 ---
			sprintf(payload_str, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.6f,%.6f,%d,%d,%s,%.2f\r\n",
					pkt.imu.accelX, pkt.imu.accelY, pkt.imu.accelZ,
					pkt.imu.gyroX, pkt.imu.gyroY, pkt.imu.gyroZ,
					pkt.imu.roll, pkt.imu.pitch, pkt.imu.head,
					pkt.baro.temperature, pkt.baro.height,
					pkt.gnss.latitude, pkt.gnss.longitude,
					pkt.gnss.satellites, flags, time_val, vbat); // <--- PERFECTLY ALIGNED!

            // Send the packet over the air
            SX1276_SendPacket((uint8_t*)payload_str, strlen(payload_str));

            // Give the RTOS some breathing room between heavy RF transmissions
            osDelay(10);
        }
    }
  /* USER CODE END startTaskLoRa */
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
