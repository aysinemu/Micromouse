/*
 * API.c
 *
 *  Created on: Aug 17, 2024
 *      Author: Minh Tuan
 */


#include "API.h"
#include "bno055.h"
#include "bno_config.h"
#include "MPU6050.h"
#include <math.h>

uint16_t timerLeftDefault = 4000;
uint16_t timerRightDefault = 4150;

uint16_t timerLeft = 4000;
uint16_t timerRight = 4150;


const uint16_t MAX_TIMER_LEFT_VALUE = 4100; // Upper limit for PWM values
const uint16_t MIN_TIMER_LEFT_VALUE = 3900;
const uint16_t MAX_TIMER_RIGHT_VALUE = 4250; // Upper limit for PWM values
const uint16_t MIN_TIMER_RIGHT_VALUE = 4050;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern double distanceStraight();
extern double distanceLeft();
extern double distanceRight();
extern void angle(int goc);
extern int16_t countTim3;
extern int16_t countTim2;
extern bno055_euler_t CurrentYaw();
int16_t lastTim3_Left = 0;
int16_t lastTim2_Right = 0;


double laststraight;

double left ;
double right;
double straight;

uint16_t counterWheel = 0;
int16_t rightWheel = 0;
int16_t leftWheel = 0;

void timer_Right(uint8_t timer){
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
	timerRight += timer;
    if (timerRight > MAX_TIMER_RIGHT_VALUE) {
    	timerRight = MAX_TIMER_RIGHT_VALUE;
    }
    if (timerLeft > MIN_TIMER_LEFT_VALUE) {
    	timerLeft -= timer;
    }
}
void timer_Left(uint8_t timer){
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
	timerLeft += timer;
    if (timerLeft > MAX_TIMER_LEFT_VALUE) {
    	timerLeft = MAX_TIMER_LEFT_VALUE;
    }
    if (timerRight > MIN_TIMER_RIGHT_VALUE) {
    	timerRight -= timer;
    }
}

void forwardGPIO(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
// led 3 right
// led 5 left
//-1250;
int16_t lastpositionLeft;
int16_t lastpositionRight;

void API_moveForward(){
	forwardGPIO();
	bno055_euler_t lastYaw = CurrentYaw();
	bno055_euler_t currentYaw;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRight); // right

	rightWheel = 0;
	leftWheel = 0;
	uint8_t flag = 0;

	lastpositionLeft = countTim3;
	lastpositionRight = countTim2;
	straight = distanceStraight();
	while(1){
		left = distanceLeft();
		right = distanceRight();
		straight = distanceStraight();


		if(straight < 5.5) {
			break;
		}

		else if(lastpositionRight - countTim2 > 1260 && lastpositionLeft - countTim3 > 1260){
			if(straight < 5.5 || straight > 12){
				break;
			}
		}

		if(HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) == 0){
			timer_Left(30);
			flag = 0;

		}
		else if(HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) == 0){
			timer_Right(30);
			flag = 0;
		}


		else if(left < 5){
			timer_Left(10);
			flag = 0;
		}
		else if( right < 5){
			timer_Right(10);
			flag = 0;
		}
		else if(left >= 5 && left < 12 && right >= 5 && right < 12){
			if(left < right){
				timer_Left(30);
				flag = 0;
			}
			else {
				timer_Right(30);
				flag = 0;
			}
		}

		else if(left >= 12){
			if(HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) == 0 ||( right > 5.5 && right < 12)){
				if(right > 7) {
					timer_Left(30);
				}
				else {
					timer_Left(10);
				}
				flag = 0;
			}

		}

		else if(right >= 12 ){
			if(HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) == 0 || ( left > 5.5 && left <12)){
				if(left > 7) {
					timer_Left(30);
				}
				else {
					timer_Left(10);
				}
				flag = 0;
			}
			timer_Right(10);
			flag = 0;
		}
		else{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED5_Pin, RESET);
			if(((lastTim2_Right - countTim2) -(lastTim3_Left - countTim3)) > 3){
				timer_Left(10);
				flag = 0;
			}
			else if(((lastTim3_Left - countTim3) -(lastTim2_Right - countTim2) ) > 3){
				timer_Right(10);
				flag = 0;
			}
		}
		if(flag == 0){
			flag = 1;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRight); // right
		}
	}
	while (1) {
	    currentYaw = CurrentYaw();

	    // Calculate the raw difference
	    float yawDifference = currentYaw.yaw - lastYaw.yaw;

	    // Handle wrapping around the 360-degree mark
	    if (yawDifference > 180.0) {
	        yawDifference -= 360.0;
	    } else if (yawDifference < -180.0) {
	        yawDifference += 360.0;
	    }

	    // Check if the yaw difference is greater than 5 degrees
	    if (yawDifference > 5.0) {
	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);     // left motor off
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 4150);  // right motor forward
	    }
	    // Check if the yaw difference is less than -5 degrees
	    else if (yawDifference < -5.0) {
	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4000);  // left motor forward
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);     // right motor off
	    }
	    // Within the 5-degree tolerance
	    else {
	        stop();
	        HAL_Delay(150);

	        // uint64_t total = mean x counter;

	        // counter ++;
	        // total = total + ((lastcounterencoderight - counttim2) - (lastcounterencoderleft- counttim3)/2)
	        // mean = total/counter;


	        return;
	    }
	}
}

void backwardRight(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,4150); // right

}
void backwardLeft(){
	HAL_GPIO_WritePin(AI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, BI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, AI1_Pin,SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,4000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,0); // right
}
void API_turnRight(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);

	bno055_euler_t lastYaw = CurrentYaw();
	bno055_euler_t currentYaw;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,4000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,4150); // right

    float target_yaw = lastYaw.yaw + 90;

    // Handle angle wrapping
    if (target_yaw >= 360.0) {
        target_yaw -= 360.0;
    } else if (target_yaw < 0.0) {
        target_yaw += 360.0;
    }

    float mean = 0;
    while (1) {
        currentYaw = CurrentYaw();

        // Calculate the yaw difference and handle wrapping
        float yawDifference = currentYaw.yaw - target_yaw;

        if (yawDifference > 180.0) {
            yawDifference -= 360.0;
        } else if (yawDifference < -180.0) {
            yawDifference += 360.0;
        }

        mean = fabs(yawDifference) ;
        if (mean < 50 && mean > 29) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 3150); // right
        }
        else if (mean > 12 && mean < 30) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2650); // right
        }
        else if (mean > 2 && mean < 13) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2450); // right
        }
        if (mean < 3) {
            stop();
            return;
        }
    }
}
void API_turnLeft(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,4000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,4150); // right
	bno055_euler_t lastYaw = CurrentYaw();
	bno055_euler_t currentYaw;
    float target_yaw = lastYaw.yaw - 90;
    float mean = 0;
    // Handle angle wrapping
    if (target_yaw >= 360.0) {
        target_yaw -= 360.0;
    } else if (target_yaw < 0.0) {
        target_yaw += 360.0;
    }

    while (1) {
        currentYaw = CurrentYaw();

        float yawDifference = currentYaw.yaw - target_yaw;

        if (yawDifference > 180.0) {
            yawDifference -= 360.0;
        } else if (yawDifference < -180.0) {
            yawDifference += 360.0;
        }

        mean = fabs(yawDifference) ;
        if (mean < 50 && mean > 29) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 3150); // right
        }
        else if (mean > 12 && mean < 30) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2650); // right
        }
        else if (mean > 2 && mean < 13) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2200); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2350); // right
        }
        if (mean < 3) {
            stop();
            return;
        }
    }
}
void API_turnRight180(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);

	bno055_euler_t lastYaw = CurrentYaw();
	bno055_euler_t currentYaw;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,4000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,4150); // right

    float target_yaw = lastYaw.yaw + 180;

    // Handle angle wrapping
    if (target_yaw >= 360.0) {
        target_yaw -= 360.0;
    } else if (target_yaw < 0.0) {
        target_yaw += 360.0;
    }
    float mean = 0;
    while (1) {
        currentYaw = CurrentYaw();

        // Calculate the yaw difference and handle wrapping
        float yawDifference = currentYaw.yaw - target_yaw;

        if (yawDifference > 180.0) {
            yawDifference -= 360.0;
        } else if (yawDifference < -180.0) {
            yawDifference += 360.0;
        }

        mean = fabs(yawDifference) ;
        if (mean < 50 && mean > 29) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 3150); // right
        }
        else if (mean > 12 && mean < 30) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2650); // right
        }
        else if (mean > 2 && mean < 13) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2450); // right
        }
        if (mean < 3) {
            stop();
            return;
        }
    }
}
void API_turnLeft180(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,4000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,4150); // right
	bno055_euler_t lastYaw = CurrentYaw();
	bno055_euler_t currentYaw;
    float target_yaw = lastYaw.yaw - 180;

    // Handle angle wrapping
    if (target_yaw >= 360.0) {
        target_yaw -= 360.0;
    } else if (target_yaw < 0.0) {
        target_yaw += 360.0;
    }

    float mean = 0;
    while (1) {
        currentYaw = CurrentYaw();

        // Calculate the yaw difference and handle wrapping
        float yawDifference = currentYaw.yaw - target_yaw;

        if (yawDifference > 180.0) {
            yawDifference -= 360.0;
        } else if (yawDifference < -180.0) {
            yawDifference += 360.0;
        }

        mean = fabs(yawDifference) ;
        if (mean < 50 && mean > 29) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3000); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 3150); // right
        }
        else if (mean > 12 && mean < 30) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2500); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2650); // right
        }
        else if (mean > 1 && mean < 13) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300); // left
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2450); // right
        }
        if (mean < 2) {
            stop();
            return;
        }
    }
}
void stop(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}

