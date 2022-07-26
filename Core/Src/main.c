/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include "BME280_FreeRTOS.h"
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

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

/* Definitions for Stab_indicat_ta */
osThreadId_t Stab_indicat_taHandle;
const osThreadAttr_t Stab_indicat_ta_attributes = {
  .name = "Stab_indicat_ta",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2C_Task */
osThreadId_t I2C_TaskHandle;
const osThreadAttr_t I2C_Task_attributes = {
  .name = "I2C_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BtnReadTask */
osThreadId_t BtnReadTaskHandle;
const osThreadAttr_t BtnReadTask_attributes = {
  .name = "BtnReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UART_DataReqTas */
osThreadId_t UART_DataReqTasHandle;
const osThreadAttr_t UART_DataReqTas_attributes = {
  .name = "UART_DataReqTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CheckConnTask */
osThreadId_t CheckConnTaskHandle;
const osThreadAttr_t CheckConnTask_attributes = {
  .name = "CheckConnTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_queue */
osMessageQueueId_t UART_queueHandle;
const osMessageQueueAttr_t UART_queue_attributes = {
  .name = "UART_queue"
};
/* Definitions for I2C_Queue */
osMessageQueueId_t I2C_QueueHandle;
const osMessageQueueAttr_t I2C_Queue_attributes = {
  .name = "I2C_Queue"
};
/* Definitions for USB_Mutex */
osMutexId_t USB_MutexHandle;
const osMutexAttr_t USB_Mutex_attributes = {
  .name = "USB_Mutex"
};
/* Definitions for UART_DataCountingSem01 */
osSemaphoreId_t UART_DataCountingSem01Handle;
const osSemaphoreAttr_t UART_DataCountingSem01_attributes = {
  .name = "UART_DataCountingSem01"
};
/* USER CODE BEGIN PV */
extern BME280_Calibrate_parametrs BME280_Cal_par;
typedef struct{
	char buff[100];
}UART_Queue_t;
typedef struct{
	uint8_t value;
	uint8_t nom_of_func;
	uint8_t * result;
	uint8_t set_parameters[3];
	float * result_float;
	void (* ptr_check_con)(uint8_t * result);
	void (* ptr_set_one_par)(uint8_t value);
	void (* ptr_set_three_par)(uint8_t par1,uint8_t par2,uint8_t par3);
	void (* ptr_void)();
	void (* ptr_read_value)(float * result);
}I2C_Queue_t;

enum {
	FUNC_VOID,
	FUNC_PTR_UINT8,
	FUNC_UINT8,
	FUNC_PTR_FLOAT,
	FUNC_THREE_UINT8
}NOM_OF_FUNC;

uint8_t conection_status;
uint8_t status_mode;

enum{
	NORMAL_WORK,
	DEBUG_WORK,
	WARNING_WORK,
	ERROR_WORK
}STATUS_OF_WORK;

struct oversamling_and_mode{
	  uint8_t o_temp;
	  uint8_t o_press;
	  uint8_t o_hum;
	  uint8_t mode;
}om;
struct stndbyTime_filter_SPI{
	uint8_t standby_time;
	uint8_t filter;
	uint8_t SPI_three_wire;
}conf_sfs;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
void StartStabIndicationTask(void *argument);
void StartUART_Task(void *argument);
void StartADC_Task(void *argument);
void StartI2C_Task(void *argument);
void StartBtnReadTask(void *argument);
void StartUART_DataReqTask(void *argument);
void StartCheckConnTask(void *argument);

/* USER CODE BEGIN PFP */
void bme_init_queues();
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
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET){
	  status_mode = DEBUG_WORK;
  }else{
	  status_mode = NORMAL_WORK;
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of USB_Mutex */
  USB_MutexHandle = osMutexNew(&USB_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UART_DataCountingSem01 */
  UART_DataCountingSem01Handle = osSemaphoreNew(10, 10, &UART_DataCountingSem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART_queue */
  UART_queueHandle = osMessageQueueNew (16, sizeof(UART_Queue_t), &UART_queue_attributes);

  /* creation of I2C_Queue */
  I2C_QueueHandle = osMessageQueueNew (8, sizeof(I2C_Queue_t), &I2C_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Stab_indicat_ta */
  Stab_indicat_taHandle = osThreadNew(StartStabIndicationTask, NULL, &Stab_indicat_ta_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(StartUART_Task, NULL, &UART_Task_attributes);

  /* creation of ADC_Task */
  ADC_TaskHandle = osThreadNew(StartADC_Task, NULL, &ADC_Task_attributes);

  /* creation of I2C_Task */
  I2C_TaskHandle = osThreadNew(StartI2C_Task, NULL, &I2C_Task_attributes);

  /* creation of BtnReadTask */
  BtnReadTaskHandle = osThreadNew(StartBtnReadTask, NULL, &BtnReadTask_attributes);

  /* creation of UART_DataReqTas */
  UART_DataReqTasHandle = osThreadNew(StartUART_DataReqTask, NULL, &UART_DataReqTas_attributes);

  /* creation of CheckConnTask */
  CheckConnTaskHandle = osThreadNew(StartCheckConnTask, NULL, &CheckConnTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  bme_init_queues();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bme_init_queues(){
	I2C_Queue_t i2c_msg;
	i2c_msg.result = &conection_status;
	i2c_msg.nom_of_func = FUNC_PTR_UINT8;
	i2c_msg.ptr_check_con = BME280_Check_Conection;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);
	UART_Queue_t uart_msg;
	sprintf(uart_msg.buff, "\r\nConnection to BME280 status: 0x%x\r\n", conection_status);
	osMessageQueuePut(UART_queueHandle, &uart_msg, 0, osWaitForever);

	i2c_msg.ptr_void = BME280_ReadCalibration;
	i2c_msg.nom_of_func = FUNC_VOID;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

	i2c_msg.nom_of_func = FUNC_UINT8;
	i2c_msg.value = BME280_OVERSAMPLING_X8;
	i2c_msg.ptr_set_one_par = BME280_SetOversamplingHum;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

	i2c_msg.nom_of_func = FUNC_THREE_UINT8;
	i2c_msg.set_parameters[0] = BME280_OVERSAMPLING_X4;
	i2c_msg.set_parameters[1] = BME280_OVERSAMPLING_X4;
	i2c_msg.set_parameters[2] = BME280_MODE_NORMAL;
	i2c_msg.ptr_set_three_par = BME280_SetOversampling;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

	i2c_msg.result = &om;
	i2c_msg.nom_of_func = FUNC_PTR_UINT8;
	i2c_msg.ptr_check_con = BME280_GetOversamplingMode;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

	i2c_msg.nom_of_func = FUNC_THREE_UINT8;
	i2c_msg.set_parameters[0] = BME280_STANDBY_TIME_10;
	i2c_msg.set_parameters[1] = BME280_FILTER_4;
	i2c_msg.set_parameters[2] = BME280_3WIRE_SPI_OFF;
	i2c_msg.ptr_set_three_par = BME280_SetConfig;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

	i2c_msg.nom_of_func = FUNC_PTR_UINT8;
	i2c_msg.result = &conf_sfs;
	i2c_msg.ptr_check_con = BME280_GetConfig;
	osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartStabIndicationTask */
/**
  * @brief  Function implementing the Stab_indicat_ta thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStabIndicationTask */
void StartStabIndicationTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	UART_Queue_t msg;
    osDelay(100);
	 sprintf(msg.buff, "\n\rPrinting calibration parameters:\n\rT1: %d\n\rT2: %d\n\rT3: %d\n\r", BME280_Cal_par.T1, BME280_Cal_par.T2, BME280_Cal_par.T3);
	 osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
	 sprintf(msg.buff, "P1: %d\n\rP2: %d\n\rP3: %d\n\rP4: %d\n\rP5: %d\n\rP6: %d\n\rP7: %d\n\rP8: %d\n\rP9: %d\n\r", BME280_Cal_par.P1, BME280_Cal_par.P2, BME280_Cal_par.P3, BME280_Cal_par.P4, BME280_Cal_par.P5, BME280_Cal_par.P6, BME280_Cal_par.P7, BME280_Cal_par.P8, BME280_Cal_par.P9);
	 osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
	 sprintf(msg.buff, "H1: %d\n\rH2: %d\n\rH3: %d\n\rH4: %d\n\rH5: %d\n\rH6: %d\n\r", BME280_Cal_par.H1, BME280_Cal_par.H2, BME280_Cal_par.H3, BME280_Cal_par.H4, BME280_Cal_par.H6);
	 osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
	 sprintf(msg.buff, "\n\rOversamlping and mode:\n\rTemp: %d\n\rPress: %d\n\rHum: %d\n\rMode: %d\n\r", om.o_temp, om.o_press, om.o_hum, om.mode);
	 osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
	 sprintf(msg.buff, "\n\rStandby time, filter and SPI 3-wire:\n\rStandby time: %d\n\rFilter: %d\n\rSPI 3-wire: %d\n\r", conf_sfs.standby_time, conf_sfs.filter, conf_sfs.SPI_three_wire);
	 osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_IWDG_Refresh(&hiwdg);
	if (status_mode == DEBUG_WORK){
		sprintf(msg.buff, "Toggle led\r\n");
		osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
	}
	switch (status_mode){
	case DEBUG_WORK: osDelay(150);
	case NORMAL_WORK: osDelay(500);break;
	case ERROR_WORK: osDelay(50);break;
	case WARNING_WORK: osDelay(250);break;
	}


  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_Task */
void StartUART_Task(void *argument)
{
  /* USER CODE BEGIN StartUART_Task */
	UART_Queue_t msg;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(UART_queueHandle, &msg, 0, osWaitForever);
    HAL_UART_Transmit(&huart1, &msg.buff, strlen(msg.buff), osWaitForever);
  }
  /* USER CODE END StartUART_Task */
}

/* USER CODE BEGIN Header_StartADC_Task */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC_Task */
void StartADC_Task(void *argument)
{
  /* USER CODE BEGIN StartADC_Task */
	UART_Queue_t msg;
	uint16_t last_adc[5];
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    uint16_t adc_value;
    adc_value = HAL_ADC_GetValue(&hadc1);
    for (uint8_t i = 0; i < 4; i++){
    	last_adc[i] = last_adc[i + 1];
    }
    last_adc[4] = adc_value;

    uint8_t warning_adc = 0;
    for (uint8_t i = 0; i < 5; i++){
    	if (last_adc[i] == 0){
    		warning_adc = 1;
    	}else{
    		warning_adc = 0;
    		break;
    	}
    }
    if (warning_adc == 1){
    	status_mode = WARNING_WORK;
    }else{
    	if (status_mode != ERROR_WORK || status_mode != DEBUG_WORK){
    		status_mode = NORMAL_WORK;
    	}
    }
    sprintf(msg.buff, "ADC value: %d\r\n", adc_value);
    osMessageQueuePut(UART_queueHandle, &msg, 0, osWaitForever);
    osDelay(1000);
  }
  /* USER CODE END StartADC_Task */
}

/* USER CODE BEGIN Header_StartI2C_Task */
/**
* @brief Function implementing the I2C_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2C_Task */
void StartI2C_Task(void *argument)
{
  /* USER CODE BEGIN StartI2C_Task */
	I2C_Queue_t msg;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(I2C_QueueHandle, &msg, 0, osWaitForever);
	  switch(msg.nom_of_func){
	  case FUNC_UINT8: msg.ptr_set_one_par(msg.value);break;
	  case FUNC_PTR_UINT8: msg.ptr_check_con(msg.result);break;
	  case FUNC_VOID: msg.ptr_void();break;
	  case FUNC_THREE_UINT8: msg.ptr_set_three_par(msg.set_parameters[0],msg.set_parameters[1],msg.set_parameters[2]);break;
	  case FUNC_PTR_FLOAT: msg.ptr_read_value(msg.result_float);break;
	  default: {
		  UART_Queue_t uart_msg;
		  sprintf(uart_msg.buff, "Error value of nom_of_func\r\n");
		  osMessageQueuePut(UART_queueHandle, &uart_msg, 0, osWaitForever);
	  }break;
	  }
  }
  /* USER CODE END StartI2C_Task */
}

/* USER CODE BEGIN Header_StartBtnReadTask */
/**
* @brief Function implementing the BtnReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtnReadTask */
void StartBtnReadTask(void *argument)
{
  /* USER CODE BEGIN StartBtnReadTask */
	UART_Queue_t msg;
  /* Infinite loop */
  for(;;)
  {
	static uint32_t timer_press;
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_RESET){

    	if (status_mode == DEBUG_WORK){
    		sprintf(msg.buff, "Button pressed\r\n");
    		osMessageQueuePut(UART_queueHandle, &msg, 0, 100);
    	}

    	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_RESET){osDelay(1);}

    	uint32_t hold_time = HAL_GetTick() - timer_press;

    	/*Virtual COM Port*/
    	sprintf(msg.buff, "Time of hold: %d\r\n", hold_time);
    	osMutexAcquire(USB_MutexHandle, 250);
    	CDC_Transmit_FS(msg.buff, strlen(msg.buff));
    	osMutexRelease(USB_MutexHandle);

    	if (hold_time > 400){
    		//osSemaphoreAcquire(, timeout)
    	}
    	if (hold_time > 20 && hold_time < 400){
    		if (status_mode == 1){
    			/*UART*/
    			sprintf(msg.buff, "Button relised\r\n");
    			osMessageQueuePut(UART_queueHandle, &msg, 0, 100);
    		}
    		osSemaphoreAcquire(UART_DataCountingSem01Handle, 100);
    	}

    }else{
    	timer_press = HAL_GetTick();
    }
    osDelay(1);
  }
  /* USER CODE END StartBtnReadTask */
}

/* USER CODE BEGIN Header_StartUART_DataReqTask */
/**
* @brief Function implementing the UART_DataReqTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_DataReqTask */
void StartUART_DataReqTask(void *argument)
{
  /* USER CODE BEGIN StartUART_DataReqTask */
	UART_Queue_t msg;	//input uart message from queue
	I2C_Queue_t i2c_msg;	//

  /* Infinite loop */
  for(;;)
  {
	  /*===============If button was press=========================*/
	if (osSemaphoreRelease(UART_DataCountingSem01Handle) == HAL_OK){
		if (status_mode == 1){
			sprintf(msg.buff, "Button action\r\n");
			osMessageQueuePut(UART_queueHandle, &msg, 0, 100);
		}
		float temperature = 1000.0f, press = 0.0f, hum = 0.0f;

		i2c_msg.nom_of_func = FUNC_PTR_FLOAT;
		i2c_msg.ptr_read_value = BME280_GetTemperature;
		i2c_msg.result_float = &temperature;
		osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);
		while(temperature == 1000.0f){osDelay(10);}

		i2c_msg.nom_of_func = FUNC_PTR_FLOAT;
		i2c_msg.ptr_read_value = BME280_GetPressure;
		i2c_msg.result_float = &press;
		osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

		i2c_msg.nom_of_func = FUNC_PTR_FLOAT;
		i2c_msg.ptr_read_value = BME280_GetHumidity;
		i2c_msg.result_float = &hum;
		osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);

		while (osMessageQueueGetCount(I2C_QueueHandle) != 0 ){osDelay(1);} // wait for end of all function

		/*=========Forming the buffer to uart queue and put in queue========*/
		sprintf(msg.buff, "\r\nTemperature: %.03f *C\r\nPressure: %.03f hPa\r\nHumidaty: %.03f %%\r\n\r\n", temperature, press/1000.0f, hum);
		osMessageQueuePut(UART_queueHandle, &msg, 0, 100);

		/*=========Mutex for USB======================*/
		osMutexAcquire(USB_MutexHandle, osWaitForever);
		CDC_Transmit_FS(msg.buff, strlen(msg.buff));
		osMutexRelease(USB_MutexHandle);
	}
    osDelay(1);
  }
  /* USER CODE END StartUART_DataReqTask */
}

/* USER CODE BEGIN Header_StartCheckConnTask */
/**
* @brief Function implementing the CheckConnTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCheckConnTask */
void StartCheckConnTask(void *argument)
{
  /* USER CODE BEGIN StartCheckConnTask */
  /* Infinite loop */

	/*Regular check connection to BME280*/
  for(;;)
  {
	  /*Forming the queue to I2C, function BME280_Check_Conection*/
	  I2C_Queue_t i2c_msg;
	  i2c_msg.result = &conection_status;
	  i2c_msg.nom_of_func = FUNC_PTR_UINT8;
	  i2c_msg.ptr_check_con = BME280_Check_Conection;
	  osMessageQueuePut(I2C_QueueHandle, &i2c_msg, 0, osWaitForever);
	  /*If connection is failed*/
	  if (conection_status != HAL_OK){

		  status_mode = ERROR_WORK;

		  UART_Queue_t uart_msg;
		  sprintf(uart_msg.buff, "\r\nError connection\r\n");
		  osMessageQueuePut(UART_queueHandle, &uart_msg, 0, osWaitForever);

		  MX_I2C1_Init();
		  osDelay(10);
	  }
	  if (conection_status == HAL_OK && status_mode != WARNING_WORK && status_mode != DEBUG_WORK){
		  status_mode = NORMAL_WORK;
	  }
	  /*Forming the queue to UART transmit*/
	  UART_Queue_t uart_msg;
	  sprintf(uart_msg.buff, "\r\nConnection to BME280 status: 0x%x\r\n", conection_status);
	  osMessageQueuePut(UART_queueHandle, &uart_msg, 0, osWaitForever);

	  /*Fast request if is connection error */
	  if (conection_status == HAL_OK){
		  osDelay(5000);
	  }else{
		  osDelay(1);
	  }
  }
  /* USER CODE END StartCheckConnTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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
