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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Timer1.h" //Timer.h is firmware made by AUVIC for controlling the PWM output
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0

//Circular Buffer Parameters
#define CIRCULAR_BUFFER_SIZE 3000 //Size of Buffer for UART Recieved communication in Bytes

//PWM Timing Parameters
#define PrescalerRegister 84 - 1 //Divide timer clock (84MHZ) by x + 1
#define INITIALIZE_DUTY 7.5 //7.5 for 1500 us pulse width of PWM which initializes motors
#define Stop_Duty 7.5 //Same as initialize duty
#define PWM_PERIOD 20000    //Period of PWM timers
#define MOTOR_RANGE 400		//Motor power input range (400 Max)

//Time to Update Parameters
#define TimeBetweenStatusUpdates 1 //Time between status updates in seconds
#define StatusUpdateSize 11 //Size of status update message sent to jetson (in bytes)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
	//Initializing interupt handelers
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

int circleBufferRead = 0; //Variable to track where buffer is read from
int circleBufferStore = 0; //Variable to track where information is stored in bufffer
int messages = 0; //Tracks # of messages in buffer

uint8_t circleBuffer[CIRCULAR_BUFFER_SIZE] = {0}; //Creates an array of bytes of specified size for the buffer
uint8_t statusUpdate[StatusUpdateSize] = {0}; //Creates an array for status update information

// Intializing variables for traking mode of LEDs on the motor board (frquency of flashing)
int LEDTimer1 = 3;
int LEDTimer2 = 0;
int LEDTimer3 = 0;
int LEDTimer4 = 0;

// Variable for counting time between led flashes
float LEDCount = 0;

// 
float LEDState1 = 0;
float LEDState2 = 0;

float Timecount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM9_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//Sends Pulses of 1500us to intialize all motors on the sub
void Initialize_Motors(){

	//Initializing PWM parameters on the timers
	INIT_TIM_PWM(TIM5, PWM_PERIOD,PrescalerRegister);
	INIT_TIM_PWM(TIM1,PWM_PERIOD,PrescalerRegister);
	INIT_TIM_PWM(TIM11,PWM_PERIOD,PrescalerRegister);

	HAL_Delay(100);
	//start pwm 1
	Start_PWM(TIM5,2,INITIALIZE_DUTY,GPIOC,10,2);
	HAL_Delay(1);
	//start pwm 2
	Start_PWM(TIM1,4,INITIALIZE_DUTY,GPIOA,11,1);
	HAL_Delay(1);
	//start pwm 3
	Start_PWM(TIM1,1,INITIALIZE_DUTY,GPIOA,8,1);
	HAL_Delay(1);
	//start pwm 4
	Start_PWM(TIM5,1,INITIALIZE_DUTY,GPIOB,12,2);
	HAL_Delay(1);
	//start pwm 5
	Start_PWM(TIM5,3,INITIALIZE_DUTY,GPIOC,11,2);
	HAL_Delay(1);
	//start pwm 6
	Start_PWM(TIM11,1,INITIALIZE_DUTY,GPIOC,12,3);
	HAL_Delay(1);
	//start pwm 7
	Start_PWM(TIM5,4,INITIALIZE_DUTY,GPIOB,11,2);
	HAL_Delay(1);
	//start pwm 8
	Start_PWM(TIM11,1,INITIALIZE_DUTY,GPIOB,9,3);
}

//Stops all motors by sending 1500us pwm pulses
void Stop_All_Motors(){

	Set_Duty_Cycle(TIM5,2,Stop_Duty);
	Set_Duty_Cycle(TIM1,4,Stop_Duty);
	Set_Duty_Cycle(TIM1,1,Stop_Duty);
	Set_Duty_Cycle(TIM5,1,Stop_Duty);
	Set_Duty_Cycle(TIM5,3,Stop_Duty);
	Set_Duty_Cycle(TIM5,4,Stop_Duty);
	Set_Duty_Cycle(TIM11,1,Stop_Duty);// since two of the outputs on the motorboard are supplied by the same timer/channel we only need 7 statements here
}


//Sets enable pins high for all motor enables (disables motors as it is inverted)
void set_Enables_High(){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
}

//Sets enable pins low for all motor enables (enables motors as it is inverted)
void set_Enables_Low(){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

//Sets motor speed based on recieved messages
int Motor_Control(){

	int direction = 1;

	if(circleBuffer[circleBufferRead + 2] & 128){
		direction = -1;
	}

	uint8_t rawSpeed = circleBuffer[circleBufferRead + 2];
	float speed = (rawSpeed & 127)*direction;
	speed = (speed/127)*MOTOR_RANGE;
	float Duty = (1500 + speed)/PWM_PERIOD;
	Duty *= 100;

	uint8_t MotorSelect = circleBuffer[circleBufferRead + 1];


	if(MotorSelect & 1){
		Set_Duty_Cycle(TIM5,2,Duty); //Motor1 ...
		statusUpdate[1] = rawSpeed;
	}
	if(MotorSelect & 2){
		Set_Duty_Cycle(TIM1,4,Duty);
		statusUpdate[2] = rawSpeed;
	}
	if(MotorSelect & 4){
		Set_Duty_Cycle(TIM1,1,Duty);
		statusUpdate[3] = rawSpeed;
	}
	if(MotorSelect & 8){
		Set_Duty_Cycle(TIM5,1,Duty);
		statusUpdate[4] = rawSpeed;
	}
	if(MotorSelect & 16){
		Set_Duty_Cycle(TIM5,3,Duty);
		statusUpdate[5] = rawSpeed;
	}
	if(MotorSelect & 32){
		Set_Duty_Cycle(TIM11,1,Duty);
		statusUpdate[6] = rawSpeed;
	}
	if(MotorSelect & 64){
		Set_Duty_Cycle(TIM5,4,Duty);
		statusUpdate[7] = rawSpeed;
	}
	if(MotorSelect & 128){
		Set_Duty_Cycle(TIM11,1,Duty);	//Motor 8
		statusUpdate[8] = rawSpeed;
	}

}

//Turns on, flashes rhythmically, or powers off leds based on recieved message
void LED_Control(){


	int LEDSpeed = circleBuffer[circleBufferRead + 2];
	uint8_t LEDSelect = circleBuffer[circleBufferRead + 1];

	if(LEDSelect & 1){
		//LEDTimer1 = LEDSpeed; //Commented out to make LED1 uneditable
	}
	if(LEDSelect & 2){
		LEDTimer2 = LEDSpeed;
	}
	if(LEDSelect & 4){
		LEDTimer3 = LEDSpeed;
	}
	if(LEDSelect & 8){
		LEDTimer4 = LEDSpeed;
	}


}


//Toggles LED On or Off depending on LED mode and current count on timer
void LEDToggle(uint16_t pin,int Timer){


	if(Timer == 0){
		HAL_GPIO_WritePin(GPIOC, pin, 0);
	}
	else if(Timer == 1){
		
		HAL_GPIO_WritePin(GPIOC, pin, LEDState1);
	}
	else if(Timer == 2){
		HAL_GPIO_WritePin(GPIOC, pin, LEDState2);
	}
	else if(Timer == 3){
		HAL_GPIO_WritePin(GPIOC, pin, 1);
	}


}

void SendUpdate(){

	if(Timecount == TimeBetweenStatusUpdates){
		HAL_UART_Transmit(&huart1, statusUpdate, StatusUpdateSize, 0xFFFF);
	}
}

//Reads information from the buffer, changes behaviour of the board based on the information and increments index in buffer 
void readBuffer(){

	if(circleBuffer[circleBufferRead] == 'M'){

		Motor_Control();

	}
	else if(circleBuffer[circleBufferRead] == 'L'){
		LED_Control();

	}
	else if(circleBuffer[circleBufferRead] == 'P'){


	}
	else if(circleBuffer[circleBufferRead] == 'O' && circleBuffer[circleBufferRead + 1] == 'F' && circleBuffer[circleBufferRead + 2] == 'F'){
		Stop_All_Motors();
		HAL_Delay(1000);
		set_Enables_High();

	}
	else if(circleBuffer[circleBufferRead] == 'I' && circleBuffer[circleBufferRead + 1] == 'N' && circleBuffer[circleBufferRead + 2] == 'I'){
		set_Enables_Low();
		Initialize_Motors();

	}

	messages -= 1;
	circleBufferRead += 3;
	if(circleBufferRead >= CIRCULAR_BUFFER_SIZE-1){
		circleBufferRead = 0;
	}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, &circleBuffer[circleBufferStore], 3);
  statusUpdate[0] = 'S';
  HAL_TIM_Base_Start_IT(&htim9);
  set_Enables_High();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(messages > 0){
	  		  readBuffer();
	  	  }

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2500-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LED_2_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LED_2_Pin PA6 PA7
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LED_2_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	///HAL_UART_Transmit(&huart1, buffer, 1, 0xFFFF);
	messages += 1;
	circleBufferStore += 3;

	if(circleBufferStore >= CIRCULAR_BUFFER_SIZE - 1){
		circleBufferStore = 0;
	}
	
	statusUpdate[9] = (100*messages*3)/CIRCULAR_BUFFER_SIZE;
	HAL_UART_Receive_DMA(&huart1, &circleBuffer[circleBufferStore], 3);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){


	if(htim == &htim9){

		
		Timecount += 0.25;
		LEDCount += 0.25;

		if(LEDCount == 1){
			if(LEDState2 == 1){
				LEDState2 = 0;
			}
			else{
				LEDState2 = 1;
			}
		}

		if(LEDState1 == 0){
			LEDState1 = 1;
		}
		else{
			LEDState1 = 0;
		}

		LEDToggle(GPIO_PIN_0, LEDTimer1);
		LEDToggle(GPIO_PIN_1, LEDTimer2);
		LEDToggle(GPIO_PIN_2, LEDTimer3);
		LEDToggle(GPIO_PIN_3, LEDTimer4);

		SendUpdate();

		if(Timecount >= TimeBetweenStatusUpdates){
			Timecount = 0;
		}

		if(LEDCount >= 1){
			LEDCount = 0;
		}

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
