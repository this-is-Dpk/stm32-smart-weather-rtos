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
// Core header for STM32 peripheral access and HAL definitions
#include "main.h"
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "process.h"
#include<string.h>
#include<stdio.h>
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT_Conf.h"
// Standard library for memory allocation and string conversion utilities
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Function pointer type for process handlers
typedef void (*ProcessFunction)(void);
// Maps process codes to their respective handler functions
typedef struct {
    char process_code[5];           // e.g., "P01"
    ProcessFunction handler;        // function to execute
} ProcessMapEntry;

// Stores process code for queue communication
typedef struct {
    char code[5];  // 4 characters + null terminator
} ProcessCode_t;

// Maintains sensor data state, including historical values and metadata
typedef struct {
    int data[20];
    int index;
    int current;
    const char *name;  // optional: "Temp", "Pressure", etc.
} SensorState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Stack size definitions for FreeRTOS tasks (in words)
#define STACK_SIZE_TEMP_SENSOR     96
#define STACK_SIZE_PRESSURE_SENSOR 96
#define STACK_SIZE_HUMIDITY_SENSOR 96
#define STACK_SIZE_LOGGER          96
#define STACK_SIZE_RESPONSE        96
#define STACK_SIZE_EMERGENCY       96
#define STACK_SIZE_STACKMON        256
#define STACK_SIZE_STATUS          96

// Task priority levels (higher number = higher priority)
#define PRIORITY_EMERGENCY_TASK    3
#define PRIORITY_SENSOR_TASK       2
#define PRIORITY_LOGGER_TASK       1
#define PRIORITY_RESPONSE_TASK     1
#define PRIORITY_MONITOR_TASK      0
#define PRIORITY_STATUS      0
#define UART_RX_BUF_SIZE 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// UART handle for serial communication via USART1
UART_HandleTypeDef huart1;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// Global variables for callback flags and batch processing index
int callback =0,  batch_index, fall_callback =0, rise_callback=0;
char  tx_buffer[100];
char uart_rx_buffer[UART_RX_BUF_SIZE];
uint8_t uart_rx_index = 0;
volatile uint8_t Live_data_enable =0,Unknown_command_state = 0;
uint8_t uart_rx_byte;
// Semaphores for UART mutual exclusion and sensor data synchronization
SemaphoreHandle_t uart_transmit_mutex, temp_ready, pressure_ready, humidity_ready;
BaseType_t  task_status;
QueueHandle_t process_code_queue;
TaskHandle_t Temp_Handle = NULL, Humidity_Handle = NULL, Pressure_Handle = NULL, Data_logger_Handle = NULL;
volatile uint8_t temp_task_heartbeat = 0, pressure_task_heartbeat = 0, humidity_task_heartbeat = 0;
volatile uint8_t emergency_triggered = 0;
// Simulated server response batches for testing process handling
char *mock_response_batch[][3] = {
    { "P01", "P07", "P05" },  //  Storm + Emergency + Log → Flash red LED, Shutdown outputs, Log data
    { "P02", "P10", "P04" },  //  Heatwave Recovery → Activate fan, Recalibrate sensors, Normal mode
    { "P03", "P01", "P09" },  //  Rain + Low Pressure + Wind → Close valve, Buzzer, Deploy antenna
    { "P08", "P05", "P06" },  //  Monitoring + Logging → Show OLED, Save to SD, Prepare cloud packet
    { "P07", "P04", "P03" },  //  Emergency + Reset + Rain → Safe state, Normal mode, Valve control
    { "P09", "P02", "P10" },  //  Wind + Heat + Recalibrate → LED blink, Fan ON, Self-check
    { "P05", "P06", "P08" },  //  Log + Upload + Display → SD write, JSON prepare, OLED show
    { "P03", "P04", "P07" },  //  Rain Alert + Normal + Emergency → Valve control, Mode switch, Shutdown
    { "P01", "P09", "P02" },  //  Storm + Wind + Heatwave → Flash LED, Antenna, Activate fan
    { "P10", "P06", "P05" }   //  Maintenance + Cloud + Logging → Recalibrate, Cloud upload, SD save
};

// Lookup table mapping process codes to handler functions
const ProcessMapEntry process_map[] = {
    { "P01", do_storm_alert },
    { "P02", do_heatwave_response },
    { "P03", do_rain_protection },
    { "P04", do_normal_mode },
    { "P05", do_data_log },
    { "P06", do_cloud_upload_prep },
    { "P07", do_emergency_shutdown },
    { "P08", do_display_readings },
    { "P09", do_wind_advisory },
    { "P10", do_recalibrate },
};
const int process_map_size = sizeof(process_map) / sizeof(process_map[0]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// Function prototypes for system initialization and task handlers
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

void Pressure_sensor_BMP280(void *);
void Humidity_sensor_DHT22(void *);
void Temperature_sensor_DS18B20(void *);
void Response_Handler(void *);
void Data_logger(void *);
void Uart1_print(const  char *);
void Emergency_alert(void *);
void vApplicationMallocFailedHook(void);
static  void stack_monitor_task(void *) ;
static void my_utoa(uint32_t , char *);
void Status_Monitor(void *);
void temp_init(void *);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Simulated temperature sensor data with predefined values
SensorState_t temp_sensor = {
	.data = {25, 28, 30, 29, 32, 34, 36, 38, 40, 41, 44, 46, 47, 43, 39, 35, 30, 28, 12, 8},
    .index = 0,
    .current = 0,
    .name = "Temp : "
};

// Simulated pressure sensor data with predefined values
SensorState_t pressure_sensor = {
	.data = {98, 99, 100, 101, 97, 96, 102, 104, 103, 106, 107, 100, 98, 96, 95, 94, 93, 108, 97, 96},
    .index = 0,
    .current = 0,
    .name = "Pressure : "
};

// Simulated humidity sensor data with predefined values
SensorState_t humidity_sensor = {
	.data = {50, 55, 60, 65, 70, 75, 80, 85, 90, 88, 60, 55, 45, 40, 35, 25, 18, 15, 10, 5},
    .index = 0,
    .current = 0,
    .name = "Humidity : "
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
// Application entry point: initializes hardware and starts FreeRTOS scheduler
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  // Initialize STM32 HAL library
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  // Configure system clock using HSI oscillator
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Enable UART interrupt-driven reception for single-byte input
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  // Create queue for inter-task process code communication
  process_code_queue = xQueueCreate(10, sizeof(ProcessCode_t));

  HAL_UART_Transmit(&huart1, (uint8_t*)"Starting...\r\n", strlen("Starting...\r\n"), 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"---------------------------sema start---------------------------\r\n", strlen("---------------------------sema start---------------------------\r\n"), 1000);
  // Initialize binary semaphores for sensor task synchronization
  temp_ready = xSemaphoreCreateBinary();
    if (temp_ready == NULL)
    	{
    	HAL_UART_Transmit(&huart1, (uint8_t*)"temp ", strlen("temp "), 1000);
    	Error_Handler();
    	}
    else HAL_UART_Transmit(&huart1, (uint8_t*)"temp ready\r\n", strlen("temp ready\r\n"), 1000);
    pressure_ready = xSemaphoreCreateBinary();
    if (pressure_ready == NULL)
 	{
 	HAL_UART_Transmit(&huart1, (uint8_t*)"pressure ", strlen("pressure "), 1000);
 	Error_Handler();
 	}
    else HAL_UART_Transmit(&huart1, (uint8_t*)"pressure ready\r\n", strlen("pressure ready\r\n"), 1000);
    humidity_ready = xSemaphoreCreateBinary();
    if (humidity_ready == NULL)
 	{
 	HAL_UART_Transmit(&huart1, (uint8_t*)"humidity ", strlen("humidity "), 1000);
 	Error_Handler();
 	}
    else HAL_UART_Transmit(&huart1, (uint8_t*)"humidity ready\r\n", strlen("humidity ready\r\n"), 1000);
    uart_transmit_mutex = xSemaphoreCreateMutex();
    if (uart_transmit_mutex == NULL)
 	{
 	HAL_UART_Transmit(&huart1, (uint8_t*)"uart_transmit_mutex ", strlen("uart_transmit_mutex "), 1000);
 	Error_Handler();
 	}
    else HAL_UART_Transmit(&huart1, (uint8_t*)"uart_transmit_mutex ready\r\n", strlen("uart_transmit_mutex ready\r\n"), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)"---------------------------sema end---------------------------\r\n", strlen("---------------------------sema end---------------------------\r\n"), 1000);

    HAL_UART_Transmit(&huart1, (uint8_t*)"---------------------------process creation start---------------------------\r\n", strlen("---------------------------process creation start---------------------------\r\n"), 1000);
  // Create FreeRTOS tasks for sensor data collection and system monitoring
    task_status = xTaskCreate(Temperature_sensor_DS18B20, "Temperature", STACK_SIZE_TEMP_SENSOR, NULL, PRIORITY_SENSOR_TASK, &Temp_Handle);
    if (task_status != pdPASS) Error_Handler();
    else HAL_UART_Transmit(&huart1, (uint8_t*)"Temperature process  created\r\n", strlen("Temperature process  created\r\n"), 1000);

    task_status = xTaskCreate(Pressure_sensor_BMP280, "Pressure", STACK_SIZE_PRESSURE_SENSOR, NULL, PRIORITY_SENSOR_TASK, &Pressure_Handle);
    if (task_status != pdPASS) Error_Handler();
    else HAL_UART_Transmit(&huart1, (uint8_t*)"Pressure process  created\r\n", strlen("Pressure process  created\r\n"), 1000);

    task_status = xTaskCreate(Humidity_sensor_DHT22, "Humidity", STACK_SIZE_HUMIDITY_SENSOR, NULL, PRIORITY_SENSOR_TASK, &Humidity_Handle);
    if (task_status != pdPASS) Error_Handler();
    else HAL_UART_Transmit(&huart1, (uint8_t*)"Humidity process  created\r\n", strlen("Humidity process  created\r\n"), 1000);

    task_status = xTaskCreate(Data_logger, "Data Logger", STACK_SIZE_LOGGER, NULL, PRIORITY_LOGGER_TASK, NULL);
    if (task_status != pdPASS) Error_Handler();
    else HAL_UART_Transmit(&huart1, (uint8_t*)"Data logger process  created\r\n", strlen("Data logger process  created\r\n"), 1000);

    task_status = xTaskCreate(Response_Handler, "Response Handler", STACK_SIZE_RESPONSE, NULL, PRIORITY_RESPONSE_TASK, NULL);
	if (task_status != pdPASS) Error_Handler();
	else HAL_UART_Transmit(&huart1, (uint8_t*)"Response_Handler  process  created\r\n", strlen("Response_Handler  process  created\r\n"), 1000);

	task_status = xTaskCreate(stack_monitor_task, "Stack Monitor", STACK_SIZE_STACKMON, NULL, PRIORITY_MONITOR_TASK, NULL);
	if (task_status != pdPASS) Error_Handler();
	else HAL_UART_Transmit(&huart1, (uint8_t*)"StackMon  process  created\r\n", strlen("StackMon  process  created\r\n"), 1000);

	task_status = xTaskCreate(Emergency_alert, "Emergency", STACK_SIZE_EMERGENCY, NULL, PRIORITY_EMERGENCY_TASK, NULL);
	if (task_status != pdPASS) Error_Handler();
	else HAL_UART_Transmit(&huart1, (uint8_t*)"Emergency alert process  created\r\n", strlen("Emergency alert process  created\r\n"), 1000);

	task_status = xTaskCreate(Status_Monitor, "UART Receive text\r\n", 96, NULL, 1, NULL);
	if (task_status != pdPASS) Error_Handler();
	else HAL_UART_Transmit(&huart1, (uint8_t*)"UART Receive text process  created\r\n", strlen("UART Receive text process  created\r\n"), 1000);

 	//print_space_info(NULL);

    HAL_UART_Transmit(&huart1, (uint8_t*)"----------------------------process creation end----------------------------\r\n", strlen("----------------------------process creation end----------------------------\r\n"), 1000);

  // Configure SEGGER SystemView for real-time debugging and tracing
    SEGGER_SYSVIEW_Conf();
    vSetVaru1MaxPRIGROUPValue();
    SEGGER_SYSVIEW_Start();
    vTaskStartScheduler();
  /* USER CODE END 2 */

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
//  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"deepak\r\n", strlen("deepak\r\n"), 1000);
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// Configures system clock using HSI oscillator
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
// Initializes USART1 for serial communication at 115200 baud
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
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);   // Lower priority than EXTI (EXTI = 5)
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
// Configures GPIO pins for interrupts and LED output
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pin : PB0 - led in error handler */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Task to simulate DS18B20 temperature sensor readings
void Temperature_sensor_DS18B20(void *pvParameter)
{
	char cpy_str[10];

	while(1)
	{
		if( xSemaphoreTake( uart_transmit_mutex, portMAX_DELAY ) == pdPASS  )
			{
			itoa(temp_sensor.data[temp_sensor.index], cpy_str, 10);  // base 10
			temp_sensor.current = temp_sensor.data[temp_sensor.index];
			if (Live_data_enable)
			{
				Uart1_print(temp_sensor.name);
				if (temp_sensor.current < 10 || temp_sensor.current > 45)
				{
					Uart1_print("Out of range - ");// HAL_UART_Transmit(&huart1, (uint8_t*)"Out of range - ", strlen("Out of range - "), 1000);
				}
                Uart1_print(cpy_str);
                Uart1_print(" C\r\n");
			}
				temp_sensor.index++;
				xSemaphoreGive(uart_transmit_mutex);
				xSemaphoreGive(temp_ready);
			}
		if(temp_sensor.index == 19)
			{
				temp_sensor.index=0;
			}

		temp_task_heartbeat = 1;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// Task to simulate BMP280 pressure sensor readings
void Pressure_sensor_BMP280(void *pvParameter)
{
	char cpy_str[10];
	while (1)
	{
		if (xSemaphoreTake(uart_transmit_mutex, portMAX_DELAY) == pdPASS)
		{
			itoa(pressure_sensor.data[pressure_sensor.index], cpy_str, 10);  // base 10
			pressure_sensor.current = pressure_sensor.data[pressure_sensor.index];
			if (Live_data_enable)
			{
				Uart1_print(pressure_sensor.name);
				if (pressure_sensor.current < 95 || pressure_sensor.current > 105)
				{
					Uart1_print("Out of range - ");
				}

				Uart1_print(cpy_str);
				Uart1_print(" Pa\r\n\n");
			}
			pressure_sensor.index++;
			xSemaphoreGive(uart_transmit_mutex);
			xSemaphoreGive(pressure_ready);
		}

		if (pressure_sensor.index == 19)
		{
			pressure_sensor.index = 0;
		}

		pressure_task_heartbeat = 1;
		vTaskDelay(pdMS_TO_TICKS(1006));
	}
}

// Task to simulate DHT22 humidity sensor readings
void Humidity_sensor_DHT22(void *pvParameter)
{
	char cpy_str[10];
	while (1)
	{
		if (xSemaphoreTake(uart_transmit_mutex, portMAX_DELAY) == pdPASS)
		{
			itoa(humidity_sensor.data[humidity_sensor.index], cpy_str, 10);  // base 10
			humidity_sensor.current = humidity_sensor.data[humidity_sensor.index];
			if (Live_data_enable)
			{
				Uart1_print(humidity_sensor.name);
				if (humidity_sensor.current < 20 || humidity_sensor.current > 80)
				{
					Uart1_print("Out of range - ");
				}

				Uart1_print(cpy_str);
				Uart1_print("%\r\n");
			}
			humidity_sensor.index++;
			xSemaphoreGive(uart_transmit_mutex);
			xSemaphoreGive(humidity_ready);
		}

		if (humidity_sensor.index == 19)
		{
			humidity_sensor.index = 0;
		}

		humidity_task_heartbeat = 1;
		vTaskDelay(pdMS_TO_TICKS(1004));
	}
}

// Task to aggregate sensor data and enqueue process codes
void Data_logger(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(humidity_ready, portMAX_DELAY) == pdPASS &&
            xSemaphoreTake(temp_ready, portMAX_DELAY) == pdPASS &&
            xSemaphoreTake(pressure_ready, portMAX_DELAY) == pdPASS &&
            xSemaphoreTake(uart_transmit_mutex, portMAX_DELAY) == pdPASS)
        {
            Uart1_print("\nSending to: https://api.smartweathercloud.com/v1/alerts/process  |  Data packet : ");

            char temp_str[8], press_str[8], hum_str[8];
            char data_logger_str[50];

            itoa(temp_sensor.current, temp_str, 10);
            itoa(pressure_sensor.current, press_str, 10);
            itoa(humidity_sensor.current, hum_str, 10);

            strcpy(data_logger_str, "T : ");
            strcat(data_logger_str, temp_str);
            strcat(data_logger_str, " | P : ");
            strcat(data_logger_str, press_str);
            strcat(data_logger_str, " | H : ");
            strcat(data_logger_str, hum_str);
            strcat(data_logger_str, "\r\n");

            Uart1_print(data_logger_str);

            for (int i = 0; i < 3; i++)
            {
                ProcessCode_t item;
                strcpy(item.code, mock_response_batch[batch_index][i]);

                if (xQueueSend(process_code_queue, &item, pdMS_TO_TICKS(50)) != pdPASS)
                {
                    Uart1_print("Queue full: Failed to enqueue process code\r\n");
                }
            }

            if (batch_index == 9)
            {
                batch_index = 0;
            }
            else
            {
                batch_index++;
            }
        }
        else
        {
            Uart1_print("data logger failed\r\n");
        }

        xSemaphoreGive(uart_transmit_mutex);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Task to process server response codes from queue
void Response_Handler(void *pvParameter)
{
    ProcessCode_t received_item;

    while (1)
    {
        // Block until first code is received
        if (xQueueReceive(process_code_queue, &received_item, portMAX_DELAY) == pdTRUE)
        {
            Uart1_print("\nWaiting for response from server .......\r\n\r\n");
            Uart1_print("-------------------------Server response process-------------------------\r\n");

            for (int item_count = 0; item_count < 3; item_count++)
            {
                // First item is already received
                if (item_count > 0)
                {
                    if (xQueueReceive(process_code_queue, &received_item, pdMS_TO_TICKS(100)) != pdTRUE)
                    {
                        Uart1_print("Warning: Missing process code in batch!\r\n");
                        break;  // No more to process
                    }
                }

                Uart1_print("Processing: ");
                Uart1_print(received_item.code);
                Uart1_print("\r\n");

                int found = 0;
                for (int i = 0; i < process_map_size; i++)
                {
                    if (strcmp(received_item.code, process_map[i].process_code) == 0)
                    {
                        process_map[i].handler();  // Valid process code
                        found = 1;
                        break;
                    }
                }

                if (!found)
                {
                    Uart1_print("Unknown process code\r\n");
                }

                vTaskDelay(pdMS_TO_TICKS(30));
            }

            Uart1_print("\r\n");
        }
    }
}

// Task to handle emergency button press events
void Emergency_alert(void *pvParameter)
{
    while (1)
    {
        if (emergency_triggered &&
            xSemaphoreTake(uart_transmit_mutex, portMAX_DELAY) == pdPASS)
        {
        	emergency_triggered = 0;
        	vTaskDelay(pdMS_TO_TICKS(300));
            Uart1_print("\r\n================== EMERGENCY BUTTON PRESSED ==================\r\n");
            Uart1_print("Sending emergency alert to cloud endpoint:\r\n");
            Uart1_print("https://api.smartweathercloud.com/v1/alerts/process\r\n");

            const char *payload = "Payload: { \"event\": \"EMERGENCY_BUTTON_PRESSED\" }\r\n";
            Uart1_print(payload);

            Uart1_print("Server Response: 200 OK - Emergency acknowledged\r\n");
            Uart1_print("==============================================================\r\n\r\n");

            xSemaphoreGive(uart_transmit_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Utility function for thread-safe UART message transmission
void Uart1_print(const char *msg)
{
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
        //xSemaphoreGive(uart_transmit_mutex);
}

// FreeRTOS hook for stack overflow detection
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    Uart1_print("Stack Overflow! Task: ");
    Uart1_print(pcTaskName);
    Uart1_print("\r\n");

    while (1);  // Halt system on stack overflow
}

// External interrupt callback for emergency button press
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		emergency_triggered = 1;
}

// Task to handle process codes from queue (unused in current implementation)
void ProcessHandlerTask(void *pvParameters)
{
    char *received_code;

    while (1)
    {
        if (xQueueReceive(process_code_queue, &received_code, portMAX_DELAY) == pdTRUE)
        {
            int found = 0;
            for (int i = 0; i < process_map_size; i++)
            {
                if (strcmp(received_code, process_map[i].process_code) == 0)
                {
                    process_map[i].handler();
                    found = 1;
                    break;
                }
            }

            if (!found)
            {
                Uart1_print(" Unknown process code\n");
            }
        }
    }
}

// Task to monitor and report invalid UART commands
void Status_Monitor(void *pvParameters)
{
	while (1)
	{
		if (Unknown_command_state == 1)
		{
			Uart1_print("Invalid command\r\n");
			Unknown_command_state = 0;
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// UART receive complete callback for command processing
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_byte == 'L')  // User pressed Enter
        {
            Live_data_enable = 1;
        }
        else if(uart_rx_byte == 'S')
        {
        	Live_data_enable = 0;

        }
        else  if (uart_rx_byte != '\r' && uart_rx_byte != '\n')
        {
        	Unknown_command_state =1;
        }

        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

// Custom unsigned integer to ASCII conversion utility
static void my_utoa(uint32_t val, char *buf) {
    char temp[11];
    int i = 0, j = 0;

    if (val == 0) {
        buf[0] = '0'; buf[1] = '\0';
        return;
    }

    while (val > 0) {
        temp[i++] = (val % 10) + '0';
        val /= 10;
    }

    while (i > 0) {
        buf[j++] = temp[--i];
    }
    buf[j] = '\0';
}

// Task to monitor stack usage and task heartbeats
static void stack_monitor_task(void *params)
{
    char msg[64];
    char numBuf[12];
    UBaseType_t stackLeft;

    while (1)
    {
        SEGGER_SYSVIEW_PrintfHost("==== Stack Monitor ====");

        stackLeft = uxTaskGetStackHighWaterMark(Temp_Handle);
        my_utoa(stackLeft * sizeof(StackType_t), numBuf);
        strcpy(msg, "Temperature_sensor_DS18B20 : ");
        strcat(msg, numBuf);
        strcat(msg, " bytes");
        SEGGER_SYSVIEW_PrintfHost(msg);

        stackLeft = uxTaskGetStackHighWaterMark(Pressure_Handle);
        my_utoa(stackLeft * sizeof(StackType_t), numBuf);
        strcpy(msg, "Pressure_sensor_BMP280 : ");
        strcat(msg, numBuf);
        strcat(msg, " bytes");
        SEGGER_SYSVIEW_PrintfHost(msg);

        stackLeft = uxTaskGetStackHighWaterMark(Humidity_Handle);
        my_utoa(stackLeft * sizeof(StackType_t), numBuf);
        strcpy(msg, "Humidity_sensor_DHT22 : ");
        strcat(msg, numBuf);
        strcat(msg, " bytes");
        SEGGER_SYSVIEW_PrintfHost(msg);

        stackLeft = uxTaskGetStackHighWaterMark(Data_logger_Handle);
        my_utoa(stackLeft * sizeof(StackType_t), numBuf);
        strcpy(msg, "Data_logger : ");
        strcat(msg, numBuf);
        strcat(msg, " bytes");
        SEGGER_SYSVIEW_PrintfHost(msg);

        size_t freeHeap = xPortGetFreeHeapSize();
        my_utoa(freeHeap, numBuf);
        strcpy(msg, "Free heap: ");
        strcat(msg, numBuf);
        SEGGER_SYSVIEW_PrintfHost(msg);

        if (temp_task_heartbeat == 0)
        {
            Uart1_print("ALERT: Temperature task not responding!\r\n");
        }
        if (pressure_task_heartbeat == 0)
        {
            Uart1_print("ALERT: Pressure task not responding!\r\n");
        }
        if (humidity_task_heartbeat == 0)
        {
            Uart1_print("ALERT: Humidity task not responding!\r\n");
        }

        // Reset for next check
        temp_task_heartbeat = 0;
        pressure_task_heartbeat = 0;
        humidity_task_heartbeat = 0;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// FreeRTOS hook for heap allocation failure
void vApplicationMallocFailedHook(void)
{
    Uart1_print("Malloc Failed! Out of heap memory.\r\n");

    // Optional: blink an LED or trap here
    __disable_irq();  // Disable all interrupts
    while (1);        // Stay here to halt system
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
// Timer interrupt callback for system tick increment
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
// Global error handler for critical system failures
void Error_Handler(void)
{
    char *err_msg = "Critical Error. Entering safe mode.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), 1000);
    // Optionally log active task, blink LED, or soft reset
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Blink LED
        HAL_Delay(500);
    }
}

#ifdef  USE_FULL_ASSERT
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
