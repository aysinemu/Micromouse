/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_CB2_Pin GPIO_PIN_1
#define ADC_CB2_GPIO_Port GPIOA
#define ADC_CB3_Pin GPIO_PIN_2
#define ADC_CB3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_0
#define AI2_GPIO_Port GPIOB
#define AI1_Pin GPIO_PIN_1
#define AI1_GPIO_Port GPIOB
#define BI1_Pin GPIO_PIN_10
#define BI1_GPIO_Port GPIOB
#define BI2_Pin GPIO_PIN_11
#define BI2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_12
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_14
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_15
#define LED6_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define UP 0
#define	DOWN 1
#define	LEFT 2
#define	RIGHT 3


#define ROW 16
#define COL 16

typedef struct{
    int row;
    int col;
    int value;
}coord;

typedef struct cell_infos{
	// variables for north,east,south,west walls
	bool walls[4];
	bool visited;
    int angle_update;
    bool dead;
}cell_info;
typedef struct wall_mazes{
	cell_info cells[16][16];
}wall_maze;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
