/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
	uint8_t buff[4];
	int32_t value;
} packet_buffer32_t;
typedef union {
	uint8_t buff[2];
	int16_t value;
} packet_buffer16_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 18

#define PI 3.141592653589
#define SHAFT_PULSE_PER_ROT 28.0
#define GEAR_RATIO 210.0

#define PULSE_PER_REVOLUTION (GEAR_RATIO*SHAFT_PULSE_PER_ROT)
#define WHEEL_DIAMETER 45.5 //mm
#define WHEEL_DISTANCE 160.5 //mm
#define WHEEL_CIRCUMFERENCE PI * WHEEL_DIAMETER
#define DIST_CONVERSION WHEEL_CIRCUMFERENCE/PULSE_PER_REVOLUTION
#define WHEEL_RADIUS WHEEL_DIAMETER/2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encoder_control */
osThreadId_t encoder_controlHandle;
const osThreadAttr_t encoder_control_attributes = {
  .name = "encoder_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for odometry_math */
osThreadId_t odometry_mathHandle;
const osThreadAttr_t odometry_math_attributes = {
  .name = "odometry_math",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for preempt_mission */
osThreadId_t preempt_missionHandle;
const osThreadAttr_t preempt_mission_attributes = {
  .name = "preempt_mission",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for symple_command */
osThreadId_t symple_commandHandle;
const osThreadAttr_t symple_command_attributes = {
  .name = "symple_command",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for goto_coordenate */
osThreadId_t goto_coordenateHandle;
const osThreadAttr_t goto_coordenate_attributes = {
  .name = "goto_coordenate",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for goto_angle */
osThreadId_t goto_angleHandle;
const osThreadAttr_t goto_angle_attributes = {
  .name = "goto_angle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for move */
osEventFlagsId_t moveHandle;
const osEventFlagsAttr_t move_attributes = {
  .name = "move"
};
/* Definitions for turn */
osEventFlagsId_t turnHandle;
const osEventFlagsAttr_t turn_attributes = {
  .name = "turn"
};
/* USER CODE BEGIN PV */
uint8_t RxBuffer[BUFFER_SIZE];
uint8_t TxBuffer[BUFFER_SIZE];

char ESP_mission = 'x';

int counter_R_encoder = 0;
int counter_L_encoder = 0;
int counter_R_ = 0;
int counter_L_ = 0;
int counter_error = 0;

int32_t delta_R;
int32_t delta_L;

double delta_R_encoder = 0;
double delta_L_encoder = 0;
double delta_distance = 0;
double delta_angle = 0;
double delta_x = 0;
double delta_y = 0;

double double_x_coord = 0;
double double_y_coord = 0;
double double_distance_total = 0;
double double_angle_rotation = 0;

int distance_target;
int true_distance_target;
int true_angle_rotation;
int true_angle_target;

packet_buffer16_t distance_total;
packet_buffer16_t x_coord;
packet_buffer16_t y_coord;
packet_buffer16_t angle_rotation;
packet_buffer16_t angle_target;
packet_buffer16_t x_target;
packet_buffer16_t y_target;
uint8_t velocity;
uint16_t speed;
uint16_t L_speed;
uint16_t R_speed;


int i = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void Start_encoder_control(void *argument);
void Start_odometry_math(void *argument);
void Start_preempt_mission(void *argument);
void Start_symple_command(void *argument);
void Start_goto_coordenates(void *argument);
void Start_goto_angle(void *argument);

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_SPI_TransmitReceive_DMA(&hspi1, TxBuffer, RxBuffer, BUFFER_SIZE);

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, 1);

  TIM1->CNT = 1000;
  TIM2->CNT = 1000;
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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of encoder_control */
  encoder_controlHandle = osThreadNew(Start_encoder_control, NULL, &encoder_control_attributes);

  /* creation of odometry_math */
  odometry_mathHandle = osThreadNew(Start_odometry_math, NULL, &odometry_math_attributes);

  /* creation of preempt_mission */
  preempt_missionHandle = osThreadNew(Start_preempt_mission, NULL, &preempt_mission_attributes);

  /* creation of symple_command */
  symple_commandHandle = osThreadNew(Start_symple_command, NULL, &symple_command_attributes);

  /* creation of goto_coordenate */
  goto_coordenateHandle = osThreadNew(Start_goto_coordenates, NULL, &goto_coordenate_attributes);

  /* creation of goto_angle */
  goto_angleHandle = osThreadNew(Start_goto_angle, NULL, &goto_angle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of move */
  moveHandle = osEventFlagsNew(&move_attributes);

  /* creation of turn */
  turnHandle = osEventFlagsNew(&turn_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 60-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STBY_Pin|BIN2_Pin|BIN1_Pin|AIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STBY_Pin BIN2_Pin BIN1_Pin AIN2_Pin */
  GPIO_InitStruct.Pin = STBY_Pin|BIN2_Pin|BIN1_Pin|AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AIN1_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AIN1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi){
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);



	switch (RxBuffer[4]) {
		case 'F':
			ESP_mission = 'F';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			osThreadFlagsSet(preempt_missionHandle, 0x01);
			break;
		case 'B':
			ESP_mission = 'B';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			break;
		case 'L':
			ESP_mission = 'L';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			break;
		case 'R':
			ESP_mission = 'R';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			break;
		case 'S':
			ESP_mission = 'S';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			break;
		case 'T':
			ESP_mission = 'T';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			angle_target.buff[0] = RxBuffer[6];
			angle_target.buff[1] = RxBuffer[7];

			osThreadFlagsSet(preempt_missionHandle, 0x01);
			break;
		case 'M':
			ESP_mission = 'M';
			speed = ((uint16_t) RxBuffer[5]) *10;
			L_speed = ((uint16_t) RxBuffer[5]) *10;
			R_speed = ((uint16_t) RxBuffer[5]) *10;
			x_target.buff[0] = RxBuffer[6];
			x_target.buff[1] = RxBuffer[7];
			y_target.buff[0] = RxBuffer[8];
			y_target.buff[1] = RxBuffer[9];

			osThreadFlagsSet(preempt_missionHandle, 0x01);
			break;
		case 'D':
			TxBuffer[4] = 'D';
			TxBuffer[5] = distance_total.buff[0];
			TxBuffer[6] = distance_total.buff[1];
			break;
		case 'C':
			TxBuffer[4] = 'C';
			TxBuffer[5] = x_coord.buff[0];
			TxBuffer[6] = x_coord.buff[1];
			TxBuffer[7] = y_coord.buff[0];
			TxBuffer[8] = y_coord.buff[1];
			break;
		case 'V':
			TxBuffer[4] = 'V';
			TxBuffer[5] = velocity;
			break;
		case 'P':
			TxBuffer[4] = 'P';
			TxBuffer[5] = x_coord.buff[0];
			TxBuffer[6] = x_coord.buff[1];
			TxBuffer[7] = y_coord.buff[0];
			TxBuffer[8] = y_coord.buff[1];
			TxBuffer[9] = angle_rotation.buff[0];
			TxBuffer[10] = angle_rotation.buff[1];
			break;
		default:
			//ESP_mission = 'x';
			break;
		}

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_encoder_control */
/**
* @brief Function implementing the encoder_control thread.
* @param argument: Not used
* @retval None
*/



/* USER CODE END Header_Start_encoder_control */
void Start_encoder_control(void *argument)
{
  /* USER CODE BEGIN Start_encoder_control */
//	int R_counter = 0;
//	int L_counter = 0;
//	int delta_R_counter = 0;
//	int delta_L_counter = 0;
//	int velocity_t = 0;


  /* Infinite loop */
  for(;;)
  {
	/*delta_R_counter = TIM2->CNT - R_counter;
	delta_L_counter = TIM1->CNT - L_counter;
	R_counter = TIM2->CNT;
	L_counter = TIM1->CNT;


	if(abs(delta_R_counter) > 60000){
	  if(delta_R_counter < 0){
		  delta_R_encoder = - (65535 + delta_R_counter);
	  } else{
		  delta_R_encoder = - (delta_R_counter - 65535);
	  }
	}else{
		delta_R_encoder = - delta_R_counter;
	}
	  if(abs(delta_L_counter) > 60000){
		  if(delta_L_counter < 0){
			  delta_L_encoder = - (65535 + delta_L_counter);
		  } else{
			  delta_L_encoder = - (delta_L_counter - 65535);
		  }
	  }else{
		delta_L_encoder = - delta_L_counter;
	  }

	  counter_R_encoder += delta_R_encoder;
	  counter_L_encoder += delta_L_encoder;*/

	  delta_L = (int32_t) (TIM1->CNT - 1000);
	  delta_R = (int32_t) (TIM2->CNT - 1000);
	  TIM1->CNT = 1000;
	  TIM2->CNT = 1000;

	  float d_L = delta_L*DIST_CONVERSION;
	  float d_R = delta_R*DIST_CONVERSION;

	  float d = (d_L + d_R)/2;
	  float d_theta = (d_R - d_L)/WHEEL_DISTANCE;

	  double_x_coord += d * cos(double_angle_rotation + (d_theta/2));
	  double_y_coord += d * sin(double_angle_rotation + (d_theta/2));

	  double_angle_rotation += d_theta;
	  if(double_angle_rotation > 2*PI){
		  double_angle_rotation -= 2*PI;
	  }
	  if(double_angle_rotation < -2*PI){
		  double_angle_rotation += 2*PI;
	  }

	  angle_rotation.value = (int16_t) (double_angle_rotation *180 / PI);
	  x_coord.value = (int16_t) double_x_coord;
	  y_coord.value = (int16_t) double_y_coord;

	  double_distance_total += d;
	  distance_total.value = (int16_t) double_distance_total;

	  /*delta_angle = (double) (((delta_R_encoder - delta_L_encoder) * 0.065) / 1.6);
	  double_angle_rotation += delta_angle;
	  angle_rotation.value = (int16_t) double_angle_rotation;

	  delta_distance = (double) (((delta_R_encoder + delta_L_encoder) * 0.12) / 2);
	  double_distance_total += delta_distance;
	  distance_total.value = (int16_t) double_distance_total;

	  delta_x = (double) (delta_distance * cos(double_angle_rotation * M_PI / 180));
      delta_y = (double) (delta_distance * sin(double_angle_rotation * M_PI / 180));
	  double_x_coord += delta_x;
	  double_y_coord += delta_y;
	  x_coord.value = (int16_t) double_x_coord;
	  y_coord.value = (int16_t) double_y_coord;*/

//	  velocity = (int) (delta_distance / (HAL_GetTick() - velocity_t));
//	  velocity_t = HAL_GetTick();

	osDelay(10);
  }
  /* USER CODE END Start_encoder_control */
}

/* USER CODE BEGIN Header_Start_odometry_math */
/**
* @brief Function implementing the odometry_math thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_odometry_math */
void Start_odometry_math(void *argument)
{
  /* USER CODE BEGIN Start_odometry_math */
  /* Infinite loop */
  for(;;)
  {

	  osDelay(1);
  }
  /* USER CODE END Start_odometry_math */
}

/* USER CODE BEGIN Header_Start_preempt_mission */
/**
* @brief Function implementing the preempt_mission thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_preempt_mission */
void Start_preempt_mission(void *argument)
{
  /* USER CODE BEGIN Start_preempt_mission */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
	  if(ESP_mission == 'M'){
		angle_target.value = (int) ((atan2(y_target.value,x_target.value)* 180 / M_PI) + angle_rotation.value);
		distance_target = (int) (sqrt(pow(y_target.value, 2) + pow(x_target.value,2)) + distance_total.value);

		osThreadResume(goto_coordenateHandle);
		osThreadResume(goto_angleHandle);
	  }else if(ESP_mission == 'T'){
		osThreadResume(goto_angleHandle);
	  }else if(ESP_mission == 'F'){
		angle_target.value = angle_rotation.value;
		osThreadResume(goto_angleHandle);
	  }



    osDelay(1);
  }
  /* USER CODE END Start_preempt_mission */
}

/* USER CODE BEGIN Header_Start_symple_command */
/**
* @brief Function implementing the symple_command thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_symple_command */
void Start_symple_command(void *argument)
{
  /* USER CODE BEGIN Start_symple_command */
  /* Infinite loop */
  for(;;)
  {
	switch(ESP_mission){
	case 'F':
	  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
	  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
	  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
	  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 1);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, L_speed);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, R_speed);
		break;
	case 'B':
	  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
	  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
	  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 1);
	  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, L_speed);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, R_speed);
		break;
	case 'L':
		  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
		  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
		  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
		  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 1);

	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, L_speed);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, R_speed);
		break;
	case 'R':
		  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
		  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
		  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 1);
		  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, L_speed);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, R_speed);
		break;
	case 'S':
	  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
	  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
	  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
	  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, L_speed);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, R_speed);
	  osThreadSuspend(goto_angleHandle);
		break;
	default:
		break;
	}

    osDelay(10);
  }
  /* USER CODE END Start_symple_command */
}

/* USER CODE BEGIN Header_Start_goto_coordenates */
/**
* @brief Function implementing the goto_coordenate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_goto_coordenates */
void Start_goto_coordenates(void *argument)
{
  /* USER CODE BEGIN Start_goto_coordenates */
  osThreadSuspend(goto_coordenateHandle);
  /* Infinite loop */
  for(;;)
  {
	if(ESP_mission == 'M'){
		true_distance_target = -(distance_total.value - distance_target);
		if(!(true_distance_target <= 10) || !(true_distance_target >= -10)){
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 1);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, L_speed);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, R_speed);
		  } else {
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
			osThreadSuspend(goto_coordenateHandle);
			osThreadSuspend(goto_angleHandle);
		  }

		osDelay(20);
	}

  }
  /* USER CODE END Start_goto_coordenates */
}

/* USER CODE BEGIN Header_Start_goto_angle */
/**
* @brief Function implementing the goto_angle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_goto_angle */
void Start_goto_angle(void *argument)
{
  /* USER CODE BEGIN Start_goto_angle */
  osThreadSuspend(goto_angleHandle);
  int direction_adjustment = 0;
  /* Infinite loop */
  for(;;)
  {
	if(ESP_mission == 'M'){
		true_angle_target = - (angle_rotation.value - angle_target.value);
		if(!(true_angle_target <= 1) || !(true_angle_target >= -1)){
		  if((true_angle_target) > 0){
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 1);
		  }else {
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 1);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
		  }
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ((int) L_speed  *  (0.4+(pow(abs(true_angle_target),1.5) / 360.0))));
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ((int) R_speed  *  (0.4+(pow(abs(true_angle_target),1.5) / 360.0))));
	  }
	  osDelay(20);

	}

	if(ESP_mission == 'T'){

		true_angle_rotation = angle_rotation.value - 360.0 * (int)(angle_rotation.value / 360.0);
		true_angle_target = -(true_angle_rotation - angle_target.value);
		if(!(true_angle_target <= 1) || !(true_angle_target >= -1)){
		  if((true_angle_target) < 0){
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 1);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
		  }else {
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 1);
		  }
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ((int) L_speed  *  (0.4+(pow(abs(true_angle_target),1.5) / 360.0))));
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ((int) R_speed  *  (0.4+(pow(abs(true_angle_target),1.5) / 360.0))));
	  }else{
		  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
		  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
		  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
		  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		  osThreadSuspend(goto_angleHandle);
	  }
		osDelay(20);

	}

	if(ESP_mission == 'F'){

		true_angle_target = (angle_rotation.value - angle_target.value);
		if(!(true_angle_target <= 0) || !(true_angle_target >= 0)){
			i++;
			direction_adjustment = 20 * abs(true_angle_target);
			direction_adjustment = (direction_adjustment > 0) * (direction_adjustment < (speed/2)) * direction_adjustment + (direction_adjustment >= (speed/2)) * (speed/2);
			if((true_angle_target) > 0){
				// Turn right: Increase right speed, decrease left speed
				L_speed = speed + direction_adjustment;
				R_speed = speed - direction_adjustment;
			} else {
				// Turn left: Increase left speed, decrease right speed
				R_speed = speed + direction_adjustment;
				L_speed = speed - direction_adjustment;
			}
		  }else {
		  L_speed = speed;
		  R_speed = speed;
		  }

	  osDelay(20);
	}
  }
  /* USER CODE END Start_goto_angle */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
