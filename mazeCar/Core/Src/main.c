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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ADXL345.h"
#include "MPU6050.h"
#include "L3G4200D.h"
#include "math.h"
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t ADC_VAL[3];
float vol1 = 0;
float vol2 = 0;
float vol3 = 0;

float front_sensor = 0;
float left_sensor = 0;
float right_sensor = 0;

char Buffer[25] = {0};
uint16_t x,y,z;
MPU6050_t MPU6050;

double roll,pitch,yaw, froll, fpitch, fyaw = 0;
uint32_t elapsedTime, currentTime, previousTime;
double dt = 0.015;

uint32_t counterTim2 = 0;
int16_t countTim2 = 0;
int16_t right_wheel = 0;

uint32_t counterTim3 = 0;
int16_t countTim3 = 0;
int16_t left_wheel = 0;

int speed =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void ADC_Select_CH0(void){
    ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}
void ADC_Select_CH1(void){
  ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}
void ADC_Select_CH2(void){
  ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void filter_gyro();

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        counterTim2 = __HAL_TIM_GET_COUNTER(htim);
        countTim2 = (int16_t)counterTim2;
        right_wheel = countTim2 / 4;
    }
    else if (htim->Instance == TIM3)
    {
        counterTim3 = __HAL_TIM_GET_COUNTER(htim);
        countTim3 = (int16_t)counterTim3;
        left_wheel = countTim3 / 4;
        // Process count for TIM3
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  printf("hello world\r\n");
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  while (MPU6050_Init(&hi2c1) == 1);
  printf("init success\r\n");
  filter_gyro();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
//  HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
//  HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
//  HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
//  HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  while(1){
//		  if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0){
//			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,SET);
//		  }
//		  if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0){
//			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,RESET);
//		  }
//	  }
//	  while(1){
//		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,RESET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,RESET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,RESET);
//		  HAL_Delay(1000);
//
//		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,RESET);
//		  HAL_Delay(1000);
//
//		  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin,SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin,RESET);
//		  HAL_Delay(1000);
//
//		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin,SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin,RESET);
//		  HAL_Delay(1000);
//	  }

//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,5000); //left
//	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,5150); // right
//	  while(1){
//
//	  }

//	  previousTime = HAL_GetTick();
//	  MPU6050_Read_Gyro(&hi2c1, &MPU6050);
//	  roll += (MPU6050.Gx - froll) *dt;
//	  pitch += (MPU6050.Gy - fpitch) *dt;
//	  yaw += (MPU6050.Gz - fyaw) *dt;
//
////	  printf("X: %d Y: %d Z: %d\r\n",(int) roll, (int)pitch, (int)yaw);
//	  currentTime = HAL_GetTick();
//	  elapsedTime = currentTime - previousTime;
//	  dt = ((double) elapsedTime) / 1000;
//	  printf("GX: %d GY: %d GZ: %d dt: %f\r\n",(int)MPU6050.Gx, (int)MPU6050.Gy, (int)MPU6050.Gz,dt);

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	ADC_VAL[0] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	HAL_Delay(20);
	vol1 = (ADC_VAL[0]*3.3)/4095;
	front_sensor = 13 * pow(vol1, -1);

	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(20);
	vol2 = (ADC_VAL[1]*3.3)/4095;
	left_sensor =  13 * pow(vol2, -1);

	ADC_Select_CH2();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(20);
	vol3 = (ADC_VAL[2]*3.3)/4095;
	right_sensor =  13 * pow(vol3, -1);
//	5000L - 5150R
//	  forward(); //670 = 18cm
//	  turn_right(); //320 = 90 degree
//	  turn_left(); //300
	float t = 5.736;
	float t1 = 5.864;
	float t2 = 6;
	float ss = 1.5;
	float ss1 = 0.2;
	if (left_sensor < t && right_sensor > t1 && front_sensor > t2
		&& left_sensor > t-ss1 && right_sensor > t1+ss1){
		turn_right_nolag();
	}else if (right_sensor < t && left_sensor > t1 && front_sensor > t2
			  && right_sensor > t-ss1 && left_sensor > t1+ss1){
		turn_left_nolag();
	}else{
		if (front_sensor < t2 && left_sensor >= t-ss && left_sensor <= t1+ss && right_sensor >= t-ss && right_sensor <= t1+ss ){
			turn_180();
		}else if ( front_sensor > t2 && left_sensor >= t-ss && left_sensor <= t1+ss && right_sensor >= t-ss && right_sensor <= t1+ss ){
			forward();
		}

		else if ( front_sensor > t2 && left_sensor >= t-ss+0.8 && left_sensor <= t1+ss+9 && right_sensor >= t ){
			if (front_sensor < t2+7.7 && left_sensor >= t-ss+0.8 && left_sensor <= t1+ss+9 && right_sensor >= t ){
				way_1_right();
			}else{
//				forward();
			}
		}else if ( front_sensor > t2 && left_sensor >= t && right_sensor >= t-ss+0.8 && right_sensor <= t1+ss+9 ){
			if (front_sensor < t2+7.7 && left_sensor >= t && right_sensor >= t-ss+0.8 && right_sensor <= t1+ss+9 ){
				way_1_left();
			}else{
//				forward();
			}
		}
//		else if(front_sensor < t2+6 && left_sensor >= t+6 && right_sensor >= t+6 ){
//			forward();
//			HAL_Delay(50);
//			turn_left90();
//			HAL_Delay(230);
//		}else if(front_sensor > t2+10 && left_sensor >= t+10 && right_sensor >= t-ss && right_sensor <= t1+ss+8 ){
//			forward();
//			HAL_Delay(100);
//			turn_left90();
//			HAL_Delay(230);
//		}else if(front_sensor > t2+10 && right_sensor >= t+10 && left_sensor >= t-ss && left_sensor <= t1+ss+8 ){
//			forward();
//			HAL_Delay(100);
//			turn_right90();
//			HAL_Delay(230);
//		}else if(front_sensor > t2+10 && right_sensor >= t+10 && left_sensor >= t+10 ){
//			forward();
//			HAL_Delay(100);
//			turn_right90();
//			HAL_Delay(230);
//		}

		else{
//			stop();
		}
	}
  }
  /* USER CODE END 3 */
}
void bam_right(){
	float t = 5.736;
	if (right_sensor < t){
		turn_right();
	}
}
void bam_left(){
	float t = 5.736;
	if (left_sensor < t){
		turn_left();
	}
}
void way_1_left(){
	forward();
	HAL_Delay(700);
	turn_left90();
	HAL_Delay(300);
	forward();
	HAL_Delay(700);
}
void way_1_right(){
	forward();
	HAL_Delay(680);
	turn_right90();
	HAL_Delay(380);
	forward();
	HAL_Delay(700);
}
void turn_180(){
	turn_right90();
	HAL_Delay(630); //180 degree
}
void forward(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,3000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,3320); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
void backward(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,5000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,5150); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);
}
void turn_right(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,1600); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1400); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);
}
void turn_left(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,1400); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1600); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
void turn_right_nolag(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,3000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,1580); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
void turn_left_nolag(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,1580); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,3000); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
void turn_right90(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,4000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,4150); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);
}
void turn_left90(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,5000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,5150); // right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
void stop(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim1.Init.Prescaler = 18-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 18-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED1_Pin|STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AI2_Pin|AI1_Pin|BI1_Pin|BI2_Pin
                          |LED5_Pin|LED6_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin STBY_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AI2_Pin AI1_Pin BI1_Pin BI2_Pin
                           LED5_Pin LED6_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = AI2_Pin|AI1_Pin|BI1_Pin|BI2_Pin
                          |LED5_Pin|LED6_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void filter_gyro(){
	printf("Start probe filter\r\n");
	HAL_Delay(500);
	for(uint8_t x = 0; x < 10; x++){
		printf("*");
	    HAL_Delay(100);
	}
	printf("*\r\n");
	for(int x = 0; x < 1000; x++){
		MPU6050_Read_Gyro(&hi2c1, &MPU6050);
		froll += MPU6050.Gx;
		fpitch += MPU6050.Gy;
		fyaw += MPU6050.Gz;
	}
	froll = froll / 1000;
	fpitch = fpitch /1000;
	fyaw = fyaw/1000;
	printf("froll: %.2f fpitch: %.2f fyaw: %.2f\r\n",froll, fpitch, fyaw);
	printf("Prove filter done!\r\n");

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
