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
#include "math.h"

#include "File_Handling_RTOS.h"
#include "File_Handling_RTOS.h"

// Devices
#include "bmp581.h"
#include "lidar.h"
#include "cansat_core.h"
#include "imu.h"
#include "sx1276.h"
#include "gnss_reader.h"
#include "hmi.h"
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
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for TaskLoRa */
osThreadId_t TaskLoRaHandle;
const osThreadAttr_t TaskLoRa_attributes = {
  .name = "TaskLoRa",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for TaskHMIHandle */
osThreadId_t TaskHMIHandleHandle;
const osThreadAttr_t TaskHMIHandle_attributes = {
  .name = "TaskHMIHandle",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 512 * 4
};
/* USER CODE BEGIN PV */
// Flight session number (auto-incremented at boot based on existing files)
uint8_t flight_session = 1;
char data_filename[16];    // e.g. "DATA_001.CSV"
char lidar_filename[16];   // e.g. "LIDAR_001.CSV"
uint8_t is_calibrated = 0;

uint8_t TX_to_Baro [] = "A" ;
uint8_t RX_from_Baro [] = "A" ;

extern volatile uint8_t FatFsCnt;
extern void SDTimer_Handler(void);
FIL fil_lidar;
uint8_t lidar_file_open = 0;

// --- GLOBAL VARIABLES (Accessible by all tasks) ---
volatile uint8_t configFlag = 1; // Triggered by external switch/button
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
Battery_Data_t latest_battery;
extern HMI_Display_Data_t HMI_display_data;
// Lidar variables to handle the DMA and broken DMA parts
char lidar_cut_char[32] = {0};
uint8_t cut_char_len = 0;
extern uint8_t lidar_dma_buf[];

//IMU Variables for the DMA
uint8_t imu_dma_rx_buf[34];
// Calibration Variables
double reference_pressure_Pa = 101325.0;
double reference_temp_C = 25.0;
double PRESS_TEMP_COEF = -8.5; // Datasheet: Pressure Temperature-induced offset: +-0.5 Pa/K

// Queue Handles
QueueHandle_t qSensorEvents;
QueueHandle_t qSDCard;
QueueHandle_t qLoRa;
QueueHandle_t qSDCard_LIDAR;  // High-frequency LIDAR-only queue (50 Hz)
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
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
uint8_t debug = 1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//__disable_irq(); //to avoid crash but it also disables interrupts for uart
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART1_FORCE_RESET();

    // 2. On attend quelques millisecondes que les condensateurs internes se vident
    HAL_Delay(10);

    // 3. On relâche le Reset pour les laisser s'allumer proprement
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

  // --- 1. POWER ON LORA AND SENSORS ---
    //HAL_GPIO_WritePin(VPOWER_EN_GPIO_OUT_GPIO_Port, VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_SET);

    // --- 2. FIX SPI CHIP SELECT (NSS) ---
    //NSS must idle HIGH. If it starts LOW, the SPI bus crashes.
    HAL_GPIO_WritePin(GPIOB, LORA_NSS_Pin, GPIO_PIN_SET);

    char startup_msg[] = "\r\n[i] System Booting... Testing Hardware\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);

    // Give the LoRa module a full second to power up and stabilize
    //HAL_Delay(1000);



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

        // Séparation en partie entière et partie décimale (2 chiffres après la virgule)
        int part_ent = (int)true_battery_voltage;
        int part_dec = (int)((true_battery_voltage - part_ent) * 100);

        // On utilise %d pour les entiers et %02d pour forcer l'affichage du zéro (ex: 8.05V)
        sprintf(adc_msg, "    -> SUCCESS! Battery: %d.%02d V (Raw ADC: %lu)\r\n", part_ent, part_dec, raw_adc);
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




	// Initialize background GNSS listening on USART1
	char gnss_init_msg[] = "[*] Arming GNSS Interrupt...\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)gnss_init_msg, strlen(gnss_init_msg), 100);

	//================================================================================For debugging purposes i put GNSS initialisation in comment
	if (debug==0){
		GNSS_Init();
	}

	char startup2_msg[] = "\r\n[i] System Booting... Testing Hardware\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)startup2_msg, strlen(startup2_msg), HAL_MAX_DELAY);
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
    qSensorEvents = xQueueCreate(20, sizeof(SensorEvent_t));
    qSDCard       = xQueueCreate(5, sizeof(TelemetryPacket_t));
    qLoRa         = xQueueCreate(5,  sizeof(TelemetryPacket_t));
    qSDCard_LIDAR = xQueueCreate(100, sizeof(LidarPacket_t));  // Larger buffer for 50Hz
    qHMI_Events = xQueueCreate(10, sizeof(uint8_t));
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_GPIO_CS_GPIO_Port, SD_GPIO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GPIO_OUT_Pin|HMILED_GPIO_Pin|LORA_NSS_Pin|LORA_RST_Pin
                          |VPOWER_EN_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HMIBTN_EXTI3_Pin */
  GPIO_InitStruct.Pin = HMIBTN_EXTI3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HMIBTN_EXTI3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BARO_EXTI_Pin */
  GPIO_InitStruct.Pin = BARO_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BARO_EXTI_GPIO_Port, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ========== CALLBACK EXTI BOUTON PA3 ==========

static uint32_t last_button_press = 0;
#define DEBOUNCE_DELAY_MS 200
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3) {
        uint32_t now = HAL_GetTick();

        if (now - last_button_press < DEBOUNCE_DELAY_MS) {
            return;
        }
        last_button_press = now;
// Protections to not write if we are not ready to recieve
        if (qHMI_Events != NULL) {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    uint8_t event = 1;
                    xQueueSendFromISR(qHMI_Events, &event, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3) {
        // Process raw DMA buffer into latest_imu fields
        IMU_ProcessData_DMA(&latest_imu);
        // Timestamp AFTER processing — this is the actual measurement instant
        latest_imu.timestamp_ms = HAL_GetTick();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) { // PA4 — BMP581 DRDY
        SensorEvent_t ev = EVENT_BARO_READY;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(qSensorEvents, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Convertit un float en string sans sprintf (évite heap overflow)
 * @param value Float à convertir
 * @param decimals Nombre de décimales (1, 2, ou 6)
 * @param buffer Buffer de sortie (min 16 bytes)
 *
 * Exemple: float_to_str(7.42, 2, buf) → "7.42"
 *          float_to_str(-123.5, 1, buf) → "-123.5"
 */
void float_to_str(float value, uint8_t decimals, char* buffer)
{
    int idx = 0;

    // Gestion signe négatif
    if (value < 0) {
        buffer[idx++] = '-';
        value = -value;
    }

    // Partie entière
    int int_part = (int)value;

    // Convertir partie entière en string (manuellement)
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

        // Inverser (car on a construit à l'envers)
        for (int i = temp_idx - 1; i >= 0; i--) {
            buffer[idx++] = temp[i];
        }
    }

    // Point décimal
    if (decimals > 0) {
        buffer[idx++] = '.';

        // Partie décimale
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
/*
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

*/
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

    // --- SAFE LIDAR INIT ---
    Lidar_Init(&huart3);
    sprintf(debug_msg, "[+] LiDAR Interrupt Armed (RTOS Safe)\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    CanSatState_t last_printed_state = (CanSatState_t)-1;

    uint32_t tick_10ms = 0;

  /* Infinite loop */
  for(;;)
  {
      // ====================================================================
      // 1. BLOC HAUTE FRÉQUENCE (Exécuté toutes les 10 ms -> 100 Hz)
      // ====================================================================
      if (currentState >= STATE_READY && currentState < STATE_OFF) {

          // GNSS et Batterie (Protégés à 1 Hz par leur propre chronomètre)
          static uint32_t last_gnss_fetch = 0;
          if (HAL_GetTick() - last_gnss_fetch > 1000) {
              last_gnss_fetch = HAL_GetTick();
              SensorEvent_t gnss_ticket = EVENT_GNSS_READY;
              xQueueSend(qSensorEvents, &gnss_ticket, 0);

              SensorEvent_t bat_ticket = EVENT_BATTERY_READY;
              xQueueSend(qSensorEvents, &bat_ticket, 0);
          }

          // Baromètre et IMU demandés à 100 Hz

          // not needed because we got baro EXTI
          //SensorEvent_t baro_ticket = EVENT_BARO_READY;
          //xQueueSend(qSensorEvents, &baro_ticket, 0);

          SensorEvent_t imu_ticket = EVENT_IMU_READY;
          xQueueSend(qSensorEvents, &imu_ticket, 0);
      }

      // ====================================================================
      // 2. BLOC BASSE FRÉQUENCE (Exécuté 1 fois sur 5 -> 20 Hz)
      // ====================================================================
      if (tick_10ms % 5 == 0)
      {
          // --- A. LOGIQUE D'AFFICHAGE DEBUG ---
          if (currentState != last_printed_state) {
              switch(currentState) {
                  case STATE_STANDBY:   sprintf(debug_msg, "\r\n[FSM] State: STANDBY\r\n"); break;
                  case STATE_CONFIG:    sprintf(debug_msg, "\r\n[FSM] State: CONFIG\r\n"); break;
                  case STATE_READY:
                      sprintf(debug_msg, "\r\n[FSM] State: READY\r\n");
                      Lidar_RequestStream();
                      break;
                  case STATE_ASCENSION: sprintf(debug_msg, "\r\n[FSM] State: ASCENSION\r\n"); break;
                  case STATE_DROP:      sprintf(debug_msg, "\r\n[FSM] State: DROP\r\n"); break;
                  case STATE_RECOVERY:  sprintf(debug_msg, "\r\n[FSM] State: RECOVERY\r\n"); break;
                  case STATE_OFF:       sprintf(debug_msg, "\r\n[FSM] State: OFF\r\n"); break;
              }
              HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
              last_printed_state = currentState;
          } // <=== L'ACCOLADE MANQUANTE ÉTAIT ICI !

          // --- B. ENVOI DE LA TÉLÉMÉTRIE (LoRa & SD lente) ---
          if (currentState >= STATE_READY && currentState < STATE_OFF) {
              static uint32_t last_lora_tx = 0;
              if (HAL_GetTick() - last_lora_tx > TIME_BETWEEN_PACKET_LORA_mS) {
                  last_lora_tx = HAL_GetTick();

                  TelemetryPacket_t telemetry_pkt;
                  memset(&telemetry_pkt, 0, sizeof(TelemetryPacket_t));

                  sprintf(telemetry_pkt.baro.timestamp, "%lu", HAL_GetTick());
                  telemetry_pkt.baro.height = current_height;
                  telemetry_pkt.baro.temperature = latest_baro.temperature;
                  telemetry_pkt.imu.pitch = latest_imu.pitch;
                  telemetry_pkt.imu.roll = latest_imu.roll;
                  telemetry_pkt.imu.yaw = latest_imu.yaw;
                  telemetry_pkt.imu.accelX = latest_imu.accelX;
                  telemetry_pkt.imu.accelY = latest_imu.accelY;
                  telemetry_pkt.imu.accelZ = latest_imu.accelZ;
                  telemetry_pkt.imu.gyroX = latest_imu.gyroX;
                  telemetry_pkt.imu.gyroY = latest_imu.gyroY;
                  telemetry_pkt.imu.gyroZ = latest_imu.gyroZ;
                  telemetry_pkt.gnss.latitude = latest_gnss.latitude;
                  telemetry_pkt.gnss.longitude = latest_gnss.longitude;
                  telemetry_pkt.gnss.satellites = latest_gnss.satellites;
                  telemetry_pkt.bat.voltage = latest_battery.voltage;

                  xQueueSend(qLoRa, &telemetry_pkt, 0);
                  xQueueSend(qSDCard, &telemetry_pkt, 0);
              }
          }

          // --- C. MACHINE À ÉTATS (Transitions de vol) ---
          switch(currentState) {
              case STATE_STANDBY:
                  if (configFlag == 1) currentState = STATE_CONFIG;
                  break;

              case STATE_CONFIG:
              {

                  if (!is_calibrated) {
                      char config_msg[128];

                      sprintf(config_msg, "\r\n[*] Calibrating Barometer (Do not move)...\r\n");
                      HAL_UART_Transmit(&huart1, (uint8_t*)config_msg, strlen(config_msg), 100);

                      uint32_t start_time = HAL_GetTick();

                      if (BMP581_CalibrateGroundPressure(&reference_pressure_Pa, &reference_temp_C, &bmp_sensor) == HAL_OK) {
                          calibration_duration_ms = HAL_GetTick() - start_time;
                          sprintf(config_msg, "[+] Calibration completed: %.2f Pa | %.2f C\r\n", reference_pressure_Pa, reference_temp_C);
                      } else {
                          sprintf(config_msg, "[-] Calibration Failed! Using defaults.\r\n");
                          reference_pressure_Pa = 101325.0;
                          reference_temp_C = 25.0;
                          calibration_duration_ms = 0;
                      }

                      HAL_UART_Transmit(&huart1, (uint8_t*)config_msg, strlen(config_msg), 100);
                      is_calibrated = 1;
                  }

                  if (configFlag == 0) {
                      is_calibrated = 0;  // Reset pour le prochain cycle si on revient en CONFIG
                      currentState = STATE_READY;
                  }
                  break;
              }

              case STATE_READY:
                  if (configFlag == 1) {
                      currentState = STATE_CONFIG;
                  } else if (latest_lidar.distance < THRESH_LIDAR_IN_BOX_M && current_height > THRESH_ALTITUDE_LAUNCH_M) {
                      currentState = STATE_ASCENSION;
                  }
                  break;

              case STATE_ASCENSION:
                  if (latest_lidar.distance > THRESH_LIDAR_DEPLOYED_M && current_height > THRESH_ALTITUDE_LAUNCH_M) {
                      currentState = STATE_DROP;
                  }
                  break;

              case STATE_DROP:
                  if (current_height < THRESH_ALTITUDE_LANDING_M) {
                      currentState = STATE_RECOVERY;
                  }
                  break;

              case STATE_RECOVERY:
                  break;

              case STATE_OFF:
                  break;
          }

          // Clignotement de la LED exécuté à 20 Hz
          HAL_GPIO_TogglePin(LED_GPIO_OUT_GPIO_Port, LED_GPIO_OUT_Pin);

      } // <=== FIN DU BLOC if (tick_10ms % 5 == 0)

      // Horloge maître et pause absolue de 10 ms
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
    //FastPacket_t fast_pkt;
    extern UART_HandleTypeDef huart1;
    //char sensor_msg[256];

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
                    //uint8_t dummy_status;
                    //HAL_I2C_Mem_Read(&hi2c1, BMP581_READ_ADDR, 0x27, 1, &dummy_status, 1, 10);
                    break;
                }

                // ---------------------------------------------------------

                // The Lidar Has 2 event states because of the ping pong buffer from the DMA
                // ---------------------------------------------------------
                case EVENT_LIDAR_HALF_CPLT:
                {
                    // Le DMA a rempli la première moitié : de l'index 0 à 99
                    // On passe l'adresse du début du buffer, et on lui dit de lire 100 cases
                    Process_Lidar_Buffer_Chunk(&lidar_dma_buf[0], DMA_BUFFER_SIZE / 2, current_ms);
                    break;
                }

                // ---------------------------------------------------------
                case EVENT_LIDAR_FULL_CPLT:
                {
                    // Le DMA a rempli la deuxième moitié : de l'index 100 à 199
                    // On passe l'adresse du milieu du buffer, et on lui dit de lire 100 cases
                    Process_Lidar_Buffer_Chunk(&lidar_dma_buf[DMA_BUFFER_SIZE / 2], DMA_BUFFER_SIZE / 2, current_ms);
                    break;
                }
                /* Before the DMA update :
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
                        if (currentState == STATE_READY) {//State Ready for debugging
                            fast_pkt.lidar = latest_lidar;
                            fast_pkt.imu = latest_imu;
                            fast_pkt.baro = latest_baro;
                            fast_pkt.gnss = latest_gnss;
                        }
                        // Send LIDAR data to SD card queue at ~50Hz
                        if (currentState >= STATE_READY) {  // Only log after drop
                            xQueueSend(qSDCard_LIDAR, &fast_pkt, 0);
                        }

                    }

                    break;
                }*/


                case EVENT_IMU_READY:
                                {
                                    // Lancement du Burst Read DMA (Non-bloquant)
                                    // Taille : 34 octets. Registre de départ : BNO055_GYRO_DATA_X_LSB (0x14)
                                    if (IMU_RequestData_DMA() == 0) {
                                        // En cas de bus occupé ou erreur matérielle grave
                                        extern UART_HandleTypeDef huart1;
                                        char sensor_msg[] = "[-] DMA I2C Start Failed!\r\n";
                                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 10);
                                    }
//                                                            sprintf(sensor_msg, "[IMU | %s] H:%6.1f | P:%6.1f | R:%6.1f || ax:%5.2f | ay:%5.2f | az:%5.2f\r\n",
//                                                                    latest_imu.timestamp,
//                                                                    latest_imu.yaw, latest_imu.pitch, latest_imu.roll,
//                                                                    latest_imu.accelX, latest_imu.accelY, latest_imu.accelZ);
//                                                            HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                                    break;
                                }
                /*
                // ---------------------------------------------------------
                case EVENT_IMU_READY:
                {
                    if (getOrientationIMU(&latest_imu) == 1) {
                        // Apply Timestamp
                        sprintf(latest_imu.timestamp, "%lu", current_ms);

//                        sprintf(sensor_msg, "[IMU | %s] H:%6.1f | P:%6.1f | R:%6.1f || ax:%5.2f | ay:%5.2f | az:%5.2f\r\n",
//                                latest_imu.timestamp,
//                                latest_imu.yaw, latest_imu.pitch, latest_imu.roll,
//                                latest_imu.accelX, latest_imu.accelY, latest_imu.accelZ);
//                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                    } else {
                        sprintf(sensor_msg, "[-] IMU I2C Read Failed!\r\n");
                        HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);
                    }
                    break;
                }
*/
                // ---------------------------------------------------------
				case EVENT_GNSS_READY:
				{
					// Copy background parsed data into the global RTOS struct
					//========================================================================================Removed for debugging :
					//if (parsed_gnss.fixed) {
						latest_gnss.latitude = parsed_gnss.latitude;
						latest_gnss.longitude = parsed_gnss.longitude;
						latest_gnss.satellites = parsed_gnss.satellites;

//						sprintf(sensor_msg, "[GNSS | %lu] Lat: %.6f | Lon: %.6f | Sats: %d\r\n",
//								current_ms, latest_gnss.latitude, latest_gnss.longitude, parsed_gnss.satellites);
					//} else {
//						sprintf(sensor_msg, "[GNSS | %lu] Searching for satellites... (Sats: %d)\r\n",
//								current_ms, parsed_gnss.satellites);
					//}
//					HAL_UART_Transmit(&huart1, (uint8_t*)sensor_msg, strlen(sensor_msg), 100);

					break;
				}

				case EVENT_BATTERY_READY:
				{
		            // --- READ LIVE BATTERY VOLTAGE (ADC1) ---
		            HAL_ADC_Start(&hadc1);
		            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		                uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
		                latest_battery.voltage = ((float)raw_adc / 4095.0f) * 3.3f * 3.0f * 1.025f;
		            }
		            HAL_ADC_Stop(&hadc1);
		            // ----------------------------------------
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

	if (is_sd_inserted()) {
		char SD_init_msg[] = "[*]SD Card Inserted\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t*)SD_init_msg, strlen(SD_init_msg), 100);
	}
	else{
		char SD_init_out_msg[] = "[*]SD Card NOT Inserted\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t*)SD_init_out_msg, strlen(SD_init_out_msg), 100);

	}
	char SD_carriage[] = "\r";

	// ✅ REMPLACER PAR
	extern char data_filename[];
	extern char lidar_filename[];
	extern uint8_t flight_session;

	if (is_sd_inserted()) {
	    Mount_SD("/");

	    // Find the next available session number
	    flight_session = 1;
	    while (flight_session < 999) {
	        sprintf(data_filename, "DATA_%03d.CSV", flight_session);
	        FILINFO check_fno;
	        if (f_stat(data_filename, &check_fno) != FR_OK) {
	            break;  // This number is free
	        }
	        flight_session++;
	    }

	    sprintf(data_filename,  "DATA_%03d.CSV",  flight_session);
	    sprintf(lidar_filename, "LIDAR_%03d.CSV", flight_session);

	    // Create both files with headers
	    Create_File(data_filename);
	    Update_File(data_filename,
	        "tx_timestamp_ms,accel_x,accel_y,accel_z,"
	        "gyro_x,gyro_y,gyro_z,roll,pitch,yaw,"
	        "temperature,altitude,latitude,longitude,"
	        "satellites,flags_raw,battery_voltage\r\n");

	    Create_File(lidar_filename);
	    Update_File(lidar_filename,
	        "tx_timestamp_ms,distance,roll,pitch,yaw,"
	        "latitude,longitude,altitude,flags_raw\r\n");

	    char boot_msg[64];
	    sprintf(boot_msg, "[SD] Session %d: %s / %s\r\n",
	            flight_session, data_filename, lidar_filename);
	    HAL_UART_Transmit(&huart1, (uint8_t*)boot_msg, strlen(boot_msg), 100);
	}

	char SD_end_msg[] = "[*] SD ready for logging\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)SD_end_msg, strlen(SD_end_msg), 100);

    // Mount
    //Mount_SD("/");
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);
    //Erasing the Card for test might have to remove later :
    //Format_SD();
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);
    // Creating a telemetry file and a Cloud of points file
    //Create_File("DATA.CSV");
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);
    //Create_File("LIDAR.CSV");
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);

    // Creating the headers
    //Update_File("DATA.CSV", "tx_timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,temperature,altitude,latitude,longitude,satellites,flags_raw,battery_voltage\r\n");
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);
    //Update_File("LIDAR.CSV", "tx_timestamp_ms,distance,roll,pitch,yaw,latitude,longitude,altitude,flags_raw\r\n");
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);


    //char SD_end_msg[] = "[*] SD ready for logging\r\n";
    //HAL_UART_Transmit(&huart1, (uint8_t*)SD_end_msg, strlen(SD_end_msg), 100);

    TelemetryPacket_t pkt;
    LidarPacket_t fast_pkt;
    char csv_buffer[256];
    char lidar_buffer[128];
    /* Infinite loop */
   for(;;)
    {
	   // At the top of startTaskSDCard for loop, add SD removal detection:

	   static uint8_t sd_was_inserted = 0;

	   // Detect SD card removal: close file cleanly before card is pulled
	   if (sd_was_inserted && !is_sd_inserted()) {
	       // Card just removed - close LIDAR file safely
	       if (lidar_file_open) {
	           f_sync(&fil_lidar);   // Flush pending writes
	           f_close(&fil_lidar);  // Close properly
	           lidar_file_open = 0;
	       }
	       // Unmount filesystem cleanly
	       f_mount(NULL, "/", 0);
	       sd_was_inserted = 0;
	   }

	   // Track insertion state
	   if (is_sd_inserted()) {
	       sd_was_inserted = 1;
	   }

	   // Handle high-frequency LIDAR data (50 Hz)
	   if (uxQueueMessagesWaiting(qSDCard_LIDAR) > 0 && is_sd_inserted()) {

		   // --- LIDAR.CSV: Keep file open for the entire flight ---
		   // File is opened once when entering STATE_READY and closed on STATE_RECOVERY
		   // This avoids costly f_open/f_close on every burst at 50Hz


		   // Open the file once when we enter flight states
		   if (currentState >= STATE_READY && !lidar_file_open && is_sd_inserted()) {
			   if (f_open(&fil_lidar, lidar_filename, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
		           f_lseek(&fil_lidar, f_size(&fil_lidar));
		           lidar_file_open = 1;
		       }
		   }

		   // Close the file when flight is over
		   if (currentState == STATE_RECOVERY && lidar_file_open) {
		       f_close(&fil_lidar);
		       lidar_file_open = 0;
		   }

		   // Drain the queue and write to file
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

		           float_to_str(fast_pkt.latitude, 6, tmp);
		           offset += sprintf(lidar_buffer + offset, "%s,", tmp);

		           float_to_str(fast_pkt.longitude, 6, tmp);
		           offset += sprintf(lidar_buffer + offset, "%s,", tmp);

		           float_to_str(fast_pkt.height, 2, tmp);
		           offset += sprintf(lidar_buffer + offset, "%s,", tmp);

		           sprintf(lidar_buffer + offset, "%d\r\n", lidar_flags);

		           f_write(&fil_lidar, lidar_buffer, strlen(lidar_buffer), &bytes_written);
		       }

		       // Flush to SD card periodically to avoid data loss if power cuts
		       f_sync(&fil_lidar);
		   }
	       }
       // Waiting for a telemetry packet and checking if the SD is still inserted
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

            // Timestamp
            offset += sprintf(csv_buffer + offset, "%s,", time_val);

            // Accel X,Y,Z (%.2f)
            float_to_str(pkt.imu.accelX, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.imu.accelY, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.imu.accelZ, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);

            // Gyro X,Y,Z (%.2f)
            float_to_str(pkt.imu.gyroX, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.imu.gyroY, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.imu.gyroZ, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);

            // Roll, Pitch, Yaw (%.1f)
            float_to_str(pkt.imu.roll, 1, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.imu.pitch, 1, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.imu.yaw, 1, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);

            // Temperature, Height (%.2f)
            float_to_str(pkt.baro.temperature, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.baro.height, 2, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);

            // Latitude, Longitude (%.6f)
            float_to_str(pkt.gnss.latitude, 6, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);
            float_to_str(pkt.gnss.longitude, 6, tmp);
            offset += sprintf(csv_buffer + offset, "%s,", tmp);

            // Satellites, Flags (int)
            offset += sprintf(csv_buffer + offset, "%d,%d,", pkt.gnss.satellites, flags);

            // Battery voltage (%.2f)
            float_to_str(pkt.bat.voltage, 2, tmp);
            sprintf(csv_buffer + offset, "%s\r\n", tmp);


/*            sprintf(lidar_buffer, "%s,%.2f,%.1f,%.1f,%.1f,%.2f\r\n",
                                time_val,
                                pkt.lidar.distance,
                                pkt.imu.roll,
                                pkt.imu.pitch,
                                pkt.imu.yaw,
                                pkt.baro.height);*/


            // Safely saving on the SD Card :

            Update_File(data_filename, csv_buffer);
            HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);
         /*   Update_File("LIDAR.CSV", lidar_buffer);
            HAL_UART_Transmit(&huart1, (uint8_t*)SD_carriage, strlen(SD_carriage), 100);*/
            //vTaskDelay(1000);
        }
	   }

       // Break
       //osDelay(5);
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

    // Explicitly lock in the P.ython Sync Word just in case
    SX1276_WriteRegister(REG_SYNC_WORD, 0x12);

    // --- WAIT FOR FSM CALIBRATION AND BROADCAST GNSS STATUS ---
        // Broadcast the full 17-value packet so the Python GUI populates the map and satellite counts!
//        while (currentState < STATE_READY) {
//
//            // Read Live Battery
//            float vbat = 0.0f;
//            HAL_ADC_Start(&hadc1);
//            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
//                vbat = ((float)HAL_ADC_GetValue(&hadc1) / 4095.0f) * 3.3f * 3.0f * 1.025f;
//            }
//            HAL_ADC_Stop(&hadc1);
//
//            // Build the standard packet, putting "0" for the flight flags and injecting live GPS
//            sprintf(payload_str, "0.00,0.00,0.00,0.00,0.00,0.00,0.0,0.0,0.0,%.2f,%.2f,%.6f,%.6f,0,%lu,%.2f,%d\r\n",
//                    latest_baro.temperature, current_height,
//                    parsed_gnss.latitude, parsed_gnss.longitude,
//                    HAL_GetTick(), vbat, parsed_gnss.satellites);
//
//            SX1276_SendPacket((uint8_t*)payload_str, strlen(payload_str));
//
//            osDelay(1000); // 1 Hz refresh rate during standby
//        }

        // --- SEND DYNAMIC CALIBRATION PACKET ---
        // The FSM has locked 4+ satellites and entered STATE_READY. Fire the CAL packet!
        SX1276_SendCalibrationPacket(calibration_duration_ms);

    /* Infinite loop */
    for(;;)
    {
        // Wait infinitely for a fully-populated packet to arrive from the Sensor Task
        if (xQueueReceive(qLoRa, &pkt, portMAX_DELAY) == pdTRUE) {



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
            char tmp[16];
            int offset = 0;

            // Accel X,Y,Z (%.2f)
            float_to_str(pkt.imu.accelX, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.accelY, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.accelZ, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            // Gyro X,Y,Z (%.2f)
            float_to_str(pkt.imu.gyroX, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.gyroY, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.gyroZ, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            // Roll, Pitch, Yaw (%.1f)
            float_to_str(pkt.imu.roll, 1, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.pitch, 1, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.imu.yaw, 1, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            // Temperature, Height (%.2f)
            float_to_str(pkt.baro.temperature, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.baro.height, 2, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            // Latitude, Longitude (%.6f)
            float_to_str(pkt.gnss.latitude, 6, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);
            float_to_str(pkt.gnss.longitude, 6, tmp);
            offset += sprintf(payload_str + offset, "%s,", tmp);

            // Satellites, Flags, Timestamp
            offset += sprintf(payload_str + offset, "%d,%d,%s,", pkt.gnss.satellites, flags, time_val);

            // Battery voltage (%.2f)
            float_to_str(pkt.bat.voltage, 2, tmp);
            sprintf(payload_str + offset, "%s\r\n", tmp);

            // Send the packet over the air
            SX1276_SendPacket((uint8_t*)payload_str, strlen(payload_str));

            // Give the RTOS some breathing room between heavy RF transmissions
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
    if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 3, 100) != HAL_OK) {
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
    uint8_t hmi_failure_count = 0;

    for(;;)
    {
    	extern CanSatState_t currentState;

    	if (currentState == STATE_CONFIG) {

    		// ========== HANDLE RECALIBRATION REQUEST ==========
    		if (hmi_state.current_page == HMI_PAGE_RECALIB) {
    			extern BMP_t bmp_sensor;
    	        extern double reference_pressure_Pa;
    	        extern double reference_temp_C;
    	        extern uint8_t is_calibrated;  // The flag from BUG 1 fix

    	        // Show initial calibration screen
    	        HMI_display_recalib(&hmi_state);

    	        // Run calibration with progress updates
    	        double sum_pressure = 0.0;
    	        double sum_temp = 0.0;
    	        int valid_reads = 0;

    	        for (int i = 0; i < SAMPLES_BAROMETER_CALIBRATION; i++) {

    	        	if (bmp581_read_precise_normal(&bmp_sensor) == 0) {
    	        		extern double bmppress;
    	                extern double bmptemp;
    	                sum_pressure += bmppress;
    	                sum_temp += bmptemp;
    	                valid_reads++;
    	        	}

    	        	// Update progress bar every 5 samples
    	            if (i % 5 == 0) {
    	            	uint8_t percent = (i * 100) / SAMPLES_BAROMETER_CALIBRATION;
    	                HMI_display_recalib_progress(percent);
    	            }

    	            osDelay(60);
    	        }

    	        // Store results
    	        if (valid_reads > 0) {
    	        	reference_pressure_Pa = sum_pressure / valid_reads;
    	            reference_temp_C = sum_temp / valid_reads;
    	            is_calibrated = 1;  // Prevent STATE_CONFIG from re-running calibration
    	        }

    	        // Show result
    	        HMI_display_recalib_progress(100);
    	        osDelay(300);
    	        HMI_display_recalib_done((float)reference_pressure_Pa, (float)reference_temp_C);

    	        // Wait for button press to go back
    	        uint8_t dummy_event;
    	        xQueueReceive(qHMI_Events, &dummy_event, portMAX_DELAY);
    	        hmi_state.current_page = HMI_PAGE_MENU;
    	        hmi_state.cursor_position = 0;
    	        continue;
    		}

    	        // ========== HANDLE FORMAT SD REQUEST ==========
    	            if (hmi_state.format_sd_requested) {
    	                hmi_state.format_sd_requested = 0;

    	                // Show formatting screen
    	                ssd1306_Fill(Black);
    	                ssd1306_SetCursor(0, 10);
    	                ssd1306_WriteString("Formatting...", Font_11x18, White);
    	                ssd1306_SetCursor(0, 36);
    	                ssd1306_WriteString("Do not remove SD", Font_6x8, White);
    	                HMI_safe_update_screen();

    	                // Format SD
    	                Format_SD();

    	                // Recreate files with new session number
    	                // (session number logic handled at boot - see PART 6)
    	                // After format, reset to session 1
    	                extern uint8_t flight_session;
    	                flight_session = 1;
    	                Create_File("DATA_001.CSV");
    	                Update_File("DATA_001.CSV", "tx_timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,temperature,altitude,latitude,longitude,satellites,flags_raw,battery_voltage\r\n");
    	                Create_File("LIDAR_001.CSV");
    	                Update_File("LIDAR_001.CSV", "tx_timestamp_ms,distance,roll,pitch,yaw,altitude,latitude,longitude,flags_raw\r\n");

    	                // Confirm
    	                ssd1306_Fill(Black);
    	                ssd1306_SetCursor(10, 10);
    	                ssd1306_WriteString("Format OK!", Font_11x18, White);
    	                ssd1306_SetCursor(0, 36);
    	                ssd1306_WriteString("Files recreated", Font_6x8, White);
    	                HMI_safe_update_screen();
    	                osDelay(2000);

    	                hmi_state.current_page = HMI_PAGE_MENU;
    	                hmi_state.cursor_position = 0;
    	                continue;
    	            }

    	            // ========== NORMAL BUTTON HANDLING ==========
    	            if (xQueueReceive(qHMI_Events, &event, pdMS_TO_TICKS(500)) == pdTRUE) {
    	                HMI_handle_button(&hmi_state);
    	            }

    	            // ========== DISPLAY CURRENT PAGE ==========
    	            switch (hmi_state.current_page) {
    	                case HMI_PAGE_OVERVIEW:   HMI_display_overview(&hmi_state);  break;
    	                case HMI_PAGE_MENU:       HMI_display_menu(&hmi_state);      break;
    	                case HMI_PAGE_LORA:       HMI_display_lora(&hmi_state);      break;
    	                case HMI_PAGE_SENSORS:    HMI_display_sensors(&hmi_state);   break;
    	                case HMI_PAGE_BATTERY:    HMI_display_battery(&hmi_state);   break;
    	                case HMI_PAGE_FORMAT_SD:  HMI_display_format_sd(&hmi_state); break;
    	                case HMI_PAGE_RECALIB:    break;  // Handled above
    	            }

    	            if (HMI_safe_update_screen() == 0) {
    	                hmi_failure_count++;
    	                if (hmi_failure_count >= 3) vTaskSuspend(NULL);
    	            } else {
    	                hmi_failure_count = 0;
    	            }

    	        } else {
    	            vTaskSuspend(NULL);
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
