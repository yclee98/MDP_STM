/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include "motor.h"
#include "gyro.h"
#include "encoder.h"
#include "pid.h"
#include "moving_avg.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for syncMotorTask */
osThreadId_t syncMotorTaskHandle;
const osThreadAttr_t syncMotorTask_attributes = {
  .name = "syncMotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
//
//set to 1 once user button is pressed so that code will run
volatile int start = 0;

extern int16_t target_angle; // target angle of rotation,
//extern int16_t Kp;
//extern float Kd;
//extern float Ki;
extern int16_t rpm;         // speed in rpm number of count/sec * 60 sec  divide by 260 count per round
extern int16_t pwmMax; // Maximum PWM value = 7200 keep the maximum value too 7000
extern int16_t no_of_tick;
extern int16_t oldposC;
extern int16_t oldposD;

//Motor motorC;
//Motor motorD;
int ival = 0;

//distance calculation
double fullRotationWheel = 1580;
double circumferenceWheel = 21.3;
double distC = 0;
double distD = 0;
double goDist = 0;

uint16_t encoderCountC = 0;
uint16_t encoderCountD = 0;
uint16_t encoderCount = 0;

//gyro
double totalAngle;

//OLED row display
uint8_t OLED_row0[20],OLED_row1[20],OLED_row2[20],OLED_row3[20],OLED_row4[20],OLED_row5[20];

//encoder
encoder_instance encoderC, encoderD;

//motor pid
pid_instance motorCpid, motorDpid;
uint8_t pidEnable = 0;

//flag to indicate if car moving
volatile uint8_t isMoving = 0;
volatile int8_t direction = 0; // 1 for forward, -1 for reverse

//encoder moving average
mov_aver_intance encoderCma, encoderDma;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void syncMotor(void *argument);
void encoder(void *argument);
void oledDisplayTask(void *argument);
void StartGyroTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//	if (htim == &ENCODER_C_HTIM)
//	{
//		motorC.counter = __HAL_TIM_GET_COUNTER(htim);
//		motorC.count = (int16_t)motorC.counter;
//		motorC.position = motorC.count/4;  //x1 Encoding
//		motorC.angle = motorC.count/2; // x2 encoding
//	}
//	else if (htim == &ENCODER_D_HTIM)
//	{
//		motorD.counter = __HAL_TIM_GET_COUNTER(htim);
//		motorD.count = (int16_t)motorD.counter;
//		motorD.position = motorD.count/4;  //x1 Encoding
//		motorD.angle = motorD.count/2; // x2 encoding
//	}
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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	SerialComm_Init();
	//Motor_Init();

	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	syncMotorTaskHandle = osThreadNew(syncMotor, NULL, &syncMotorTask_attributes);
	encoderTaskHandle = osThreadNew(encoder, NULL, &encoderTask_attributes);
	oledTaskHandle = osThreadNew(oledDisplayTask, NULL, &oledTask_attributes);
	gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/**
* @}
*/
/**
* @}
*/

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|CIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIN1_Pin|DIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CIN2_Pin */
  GPIO_InitStruct.Pin = CIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CIN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CIN1_Pin */
  GPIO_InitStruct.Pin = CIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CIN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIN1_Pin DIN2_Pin */
  GPIO_InitStruct.Pin = DIN1_Pin|DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {
	if(GPIO_Pin == User_Button_Pin && start == 0){
		start = 1;
	}
}

void resetCar(){
	encoder_reset(&encoderC);
	encoder_reset(&encoderD);

	pid_reset(&motorCpid);
	pid_reset(&motorDpid);

	reset_average_filter(&encoderCma);
	reset_average_filter(&encoderDma);

	distC=0;
	distD=0;

	totalAngle=0;
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
  /* USER CODE BEGIN 5 */
	pidEnable = 1;
	for (;;)
	{
		if (start == 0)
		{
			osDelay(200);
			continue;
		}
		forward(120);
		osDelay(10);
		while(isMoving){
			osDelay(100);
		}
		osDelay(4000);
		backward(120);
		osDelay(10);
		while(isMoving){
			osDelay(100);
		}
		osDelay(4000);
	}
//	sprintf(OLED_row0, "start task");
//	target_angle = 0;
//	Kp = 20;       // 10
//	Ki = 0.001;   // 0.001
//	Kd = 700;
//
//
//	/* Infinite loop */
//	for(;;)
//	{
//		if(start == 0){
//			osDelay(1000);
//			continue;
//		}
//
//	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//
//	  //forward(150);
////	  if (ival == 0)
////	  {
////
////	  }
////	  else if (ival == 1)
////	  {
////		  Kp = 20;
////	  }
////	  else if (ival == 2)
////	  {
////		  Kd = 500;
////	  }
////	  else if (ival == 3)
////	  {
////		  Kd = 1000;
////	  }
////	  else
////	  {
////		  start = 0;
////	  }
//
//	  target_angle = 0;
//	  while (target_angle < 1700)
//	  {
//		  target_angle += 360; // rotate 720 degree
//		  motorC.start = 1;
//		  motorD.start = 1;
//		  while (motorC.start || motorD.start)
//		  {
//			  osDelay(50);
//		  }
//	  }
//	  stopMotor(&motorC);
//	  stopMotor(&motorD);
//	  target_angle = 0;
//	  ival += 1;
//	  //Ki += 0.001;
//	  //Kd -= 100;
//	  //Kp += 5;

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_syncMotor */
/**
 * @brief Function implementing the syncMotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_syncMotor */
void syncMotor(void *argument)
{
  /* USER CODE BEGIN syncMotor */
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); //motorC
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); //motorD
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //servo motor

	resetCar();

	int motorCvel, motorDvel;
	uint32_t currentTick, previousTick=0;

	for (;;)
	{
		if(!isMoving){
			osDelay(50);
			previousTick = HAL_GetTick();
			continue;
		}
		currentTick = HAL_GetTick();

		if(currentTick - previousTick > 50L){
			if(!pidEnable){
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1500);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 1500);
				sprintf(OLED_row4, "pid C 1500");
				sprintf(OLED_row5, "pid D 1500");
				osDelay(1000);
			}else if(pidEnable){
				motorCvel = encoderC.velocity;
				motorDvel = encoderD.velocity;

				apply_average_filter(&encoderCma, motorCvel);
				apply_average_filter(&encoderDma, motorDvel);

				apply_pid(&motorCpid, encoderCma.out, currentTick-previousTick);
				apply_pid(&motorDpid, encoderDma.out, currentTick-previousTick);

				if(!isMoving){
					osDelay(10);
					continue;
				}

				//forward
				if(direction == 1){
					if(motorCpid.output > 0){
						setDirection(1,1);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, motorCpid.output);
					}else{
						setDirection(0,1);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, -motorCpid.output);
					}
					if(motorDpid.output > 0){
						setDirection(1,2);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, motorDpid.output);
					}else{
						setDirection(0,2);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, -motorDpid.output);
					}
				}


				//reverse
				if(direction == -1){
					if(motorCpid.output > 0){
						setDirection(0,1);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, motorCpid.output);
					}else{
						setDirection(1,1);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, -motorCpid.output);
					}
					if(motorDpid.output > 0){
						setDirection(0,2);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, motorDpid.output);
					}else{
						setDirection(1,2);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, -motorDpid.output);
					}
				}

				sprintf(OLED_row4, "pid C %d", motorCpid.output);
				sprintf(OLED_row5, "pid D %d", motorDpid.output);


			}
			previousTick = currentTick;
		}
		//printToSerial(motorCvel, encoderCma.out, motorCpid.lastError, motorCpid.output, motorCpid.errorIntegral);
		osDelay(50);
	}

//	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
//	rpm = (int)((1000/no_of_tick) * 60/260);  // For calculating motor rpm - by multiplying it with speed value
//
//	motorC.htim = &htim4;
//	motorC.motor = 1;
//	motorC.angle = 0;
//	motorC.error = target_angle - motorC.angle;
//	motorC.error_old = 0;
//	motorC.error_area = 0;
//	motorD.htim = &htim2;
//	motorD.motor = 2;
//	motorD.angle = 0;
//	motorD.error = target_angle - motorD.angle;
//	motorD.error_old = 0;
//	motorD.error_area = 0;
//
//
//
////	Kp = 10;       // 10
////	Ki = 0.001;   // 0.001
////	Kd = 0;
//
//	motorC.millisOld = HAL_GetTick(); // get time value before starting - for PID
//	motorD.millisOld = HAL_GetTick(); // get time value before starting - for PID
//
//	int16_t pwmValC = 0;
//	int16_t pwmValD = 0;
//
//	//htim1.Instance->CCR4 = 100;
//	//osDelay(500);
//	//htim1.Instance->CCR4 = 147;
//
//	/* Infinite loop */
//	for(;;)
//	{
//		if (start==0)
//		{
//			motorC.pwmVal = 0;
//			motorD.pwmVal = 0;
//			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,motorC.pwmVal);
//			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,motorD.pwmVal);
//
//			motorC.err = 0;// for checking whether error has settle down near to zero
//			motorC.angle = 0;
//			motorC.start = 1;
//
//			motorD.err = 0;// for checking whether error has settle down near to zero
//			motorD.angle = 0;
//			motorD.start = 1;
//		}
//
//		while (start==0){ //wait for the User PB to be pressed
//			osDelay(500);
//			motorD.millisOld = HAL_GetTick(); // get time value before starting - for PID
//			motorC.millisOld = HAL_GetTick(); // get time value before starting - for PID
//		}
//
//		while(!motorD.start && !motorC.start)
//		{
//			osDelay(500);
//			motorD.millisOld = HAL_GetTick(); // get time value before starting - for PID
//			motorC.millisOld = HAL_GetTick(); // get time value before starting - for PID
//		}
//
//		if (target_angle >= 1800)
//			target_angle = 1800;
//		pwmValC = 0;
//		pwmValD = 0;
//		//target_angle = 2000;
//
//		if (motorD.start)
//		{
//			pwmValD = PID_Control(&motorD, 0); // call the PID control loop calculation
//		}
//		if (motorC.start)
//		{
//			pwmValC = PID_Control(&motorC, 1); // call the PID control loop calculation
//		}
//		motorC.pwmVal = pwmValC;          // overwrite PID control above, minimum pwmVal = 1000?
//		motorD.pwmVal = pwmValD;          // overwrite PID control above, minimum pwmVal = 1000?
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,motorC.pwmVal); // output PWM waveform to drive motor
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,motorD.pwmVal); // output PWM waveform to drive motor
//
//		if (abs(motorC.error) < 3){ // error is less than 3 deg
//			motorC.err++; // to keep track how long it has reached steady state
//			motorC.angle = -(int)(motorC.position*360/260);  //calculate the angle
//			motorC.error = target_angle - motorC.angle; // calculate the error
//		}
//		if (abs(motorD.error) < 3){ // error is less than 3 deg
//			motorD.err++; // to keep track how long it has reached steady state
//			motorD.angle = (int)(motorD.position*360/260);  //calculate the angle
//			motorD.error = target_angle - motorD.angle; // calculate the error
//		}
//
//		if (motorC.err > 1 && motorC.start) { // error has settled to within the acceptance ranges
//			resetMotor(&motorC);
//			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,motorC.pwmVal);
//		}
//		if (motorD.err > 1 && motorD.start) {
//			resetMotor(&motorD);
//			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,motorD.pwmVal);
//		}
//		while (!motorC.start && !motorD.start)
//		{
//			//continue to send the data values for display
//			//start = 0;  // wait for PB to restart
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer On
//			osDelay(500);
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer Off
//		}
//		osDelay(50);
//	}
  /* USER CODE END syncMotor */
}

/* USER CODE BEGIN Header_encoder */
/**
 * @brief Function implementing the encoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
	/* Infinite loop */
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderC
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderD

	encoder_reset(&encoderC);
	encoder_reset(&encoderD);

	int cnt1, cnt2, diffC;
	int cnt3, cnt4, diffD;

	uint32_t currentTick, previousTick=0;

	int avgDist = 0;

	cnt1 = 0;//__HAL_TIM_GET_COUNTER(&htim4);
	cnt3 = 0;//__HAL_TIM_GET_COUNTER(&htim2);
	for(;;){
		if(!isMoving){
			osDelay(50);
			previousTick = HAL_GetTick();
			continue;
		}
		currentTick = HAL_GetTick();
		if(currentTick - previousTick >= 100L){
			diffC = 0;
			diffD = 0;
			//encoderC
			cnt2 = __HAL_TIM_GET_COUNTER(&htim4);
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
			{
				if(cnt2 <= cnt1)
					diffC = cnt1 - cnt2;
				else
					diffC = (65535 - cnt2) + cnt1; //problem
			}
			else
			{
				if(cnt2 >= cnt1)
					diffC = cnt2 - cnt1;
				else
					diffC = (65535 - cnt1) + cnt2;
			}
			cnt1 = cnt2;

			//encoderD
			cnt4 = __HAL_TIM_GET_COUNTER(&htim2);
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
			{
				if(cnt4 <= cnt3)
					diffD = cnt3 - cnt4;
				else
					diffD = (65535 - cnt4) + cnt3;
			}
			else
			{
				if(cnt4 >= cnt3)
					diffD = cnt4 - cnt3;
				else
					diffD = (65535 - cnt3) + cnt4;
			}
			cnt3 = cnt4;

			distC += abs(diffC)/fullRotationWheel*circumferenceWheel;
			distD += abs(diffD)/fullRotationWheel*circumferenceWheel;

			if(diffC!=0 && diffD !=0){ //when there is movement then we see if dist greater before stop car
				avgDist = (distC+distD)/2;
				if(avgDist >= goDist){
					motorStop();
					resetCar();
					osDelay(50);
				}
			}

			encoderC.velocity = abs(diffC);
			encoderD.velocity = abs(diffD);

			sprintf(OLED_row1, "dist %d", avgDist);
			sprintf(OLED_row2, "spdC %d", encoderC.velocity);
			sprintf(OLED_row3, "spdD %d", encoderD.velocity);
			previousTick = currentTick;
			osDelay(10);
		}
		osDelay(50);
	}
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_oledDisplayTask */
/**
 * @brief Function implementing the oledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_oledDisplayTask */
void oledDisplayTask(void *argument)
{
  /* USER CODE BEGIN oledDisplayTask */
	/* Infinite loop */
	//sprintf(OLED_row1, "OLED ready");
	for(;;)
	{
		OLED_Clear();
		OLED_ShowString(10,0,OLED_row0);
		OLED_ShowString(10,10,OLED_row1);
		OLED_ShowString(10,20,OLED_row2);
		OLED_ShowString(10,30,OLED_row3);
		OLED_ShowString(10,40,OLED_row4);
		OLED_ShowString(10,50,OLED_row5);
		OLED_Refresh_Gram();
		osDelay(1000);
	}
  /* USER CODE END oledDisplayTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
 * @brief Function implementing the gyroTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */
		gyro_Init();
		uint8_t val[2] = {0, 0};
		uint32_t currentTick, previousTick=0;
		double offset = 0;//7.848882995;
		int16_t angularSpeed = 0;
		double measuredAngle = 0;
		totalAngle = 0;
		int calPWM = 0;

		previousTick = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
//	  if(!isMoving){
//	  previousTick = HAL_GetTick();
//		  osDelay(100);
//		  continue;
//	  }
	  currentTick = HAL_GetTick(); //millisecond
	  if(currentTick - previousTick >= 50L){
		  readByte(0x37, val); //read GYRO_ZOUT_H and GYRO_ZOUT_L since we pass val which is 16 bit
		   angularSpeed = (val[0] << 8) | val[1]; //(highByte * 256) + lowByte; degree/second

		   measuredAngle = ((double)(angularSpeed)+offset) * ((currentTick - previousTick) / 16400.0);
		   totalAngle += measuredAngle;
		  // printgyro(angularSpeed, angularSpeed);

		   if(totalAngle > 720)
			   totalAngle =0;
		   if(totalAngle < -720)
			   totalAngle = 0;

		   if(isMoving){
			   calPWM = (int)(147 + totalAngle*7);
				if(calPWM > 200)
				   calPWM = 200;
				if(calPWM < 100)
				   calPWM = 100;
			   htim1.Instance->CCR4 = calPWM;
		   }

		   sprintf(OLED_row0, "pwm %d", calPWM);
		   previousTick = currentTick;
	  }
	  osDelay(50);
  }
  /* USER CODE END StartGyroTask */
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
