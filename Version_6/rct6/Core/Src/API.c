/*
 * API.c
 *
 *  Created on: Aug 17, 2024
 *      Author: Minh Tuan
 */


#include "API.h"
#include "MPU6050.h"
uint16_t timerLeftDefault = 3000;
uint16_t timerRightDefault = 3150;

uint16_t timerLeft = 3000;
uint16_t timerRight = 3150;


const uint16_t MAX_TIMER_LEFT_VALUE = 3100; // Upper limit for PWM values
const uint16_t MIN_TIMER_LEFT_VALUE = 2900;
const uint16_t MAX_TIMER_RIGHT_VALUE = 3250; // Upper limit for PWM values
const uint16_t MIN_TIMER_RIGHT_VALUE = 3050;

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
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
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
	HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
	timerLeft += 5;
    if (timerLeft > MAX_TIMER_LEFT_VALUE) {
    	timerLeft = MAX_TIMER_LEFT_VALUE;
    }
    if (timerRight > MIN_TIMER_RIGHT_VALUE) {
    	timerRight -= 5;
    }
}

void forwardGPIO(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}
//-1250;
int16_t lastpositionLeft;
int16_t lastpositionRight;
void API_moveForward(){
	forwardGPIO();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRight); // right

	rightWheel = 0;
	leftWheel = 0;
	uint8_t flag = 0;

	lastpositionLeft = countTim3;
	lastpositionRight = countTim2;

	while(1){
		left = distanceLeft();
		right = distanceRight();
		straight = distanceStraight();

		if(lastpositionRight - countTim2 > 1250 && lastpositionLeft - countTim3 > 1250){
			stop();
			return;
		}
		if(straight < 5) {
			stop();
			return;
		}

		if(HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) == 0){
			timer_Left(10);
			HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		}
		else if(HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) == 0){
			timer_Right(10);
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		}


		if(left < 5.2){
			timer_Left(5);
			flag = 0;
		}
		else if( right < 5.2){
			timer_Right(5);
			flag = 0;
		}
		else if(left > 5.5 && left < 12 && right > 5.5 && right < 12){
			if(left < right){
				timer_Left(20);
				flag = 0;
			}
			else {
				timer_Right(20);
				flag = 0;
			}
		}

		else if(left > 12 && right > 5.6 && right < 12 && HAL_GPIO_ReadPin(Sensor_Right_GPIO_Port, Sensor_Right_Pin) != 0){
			timer_Left(5);
			flag = 0;
		}
		else if(right > 12 && left > 5.6 && left < 12 && HAL_GPIO_ReadPin(Sensor_Left_GPIO_Port, Sensor_Left_Pin) != 0){
			timer_Right(5);
			flag = 0;
		}
		else{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
			if(((countTim2 - lastTim2_Right) -(countTim3 - lastTim3_Left)) > 3){
				timer_Left(5);
				flag = 0;
			}
			else if(((countTim3 - lastTim3_Left) - (countTim2 - lastTim2_Right) ) > 3){
				timer_Right(5);
				flag = 0;
			}
		}
		if(flag == 0){
			flag = 1;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRight); // right
		}
	}
}
// led 3 right
// led 5 left
//void API_moveForward(){
//
//	forwardGPIO();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRight); // right
//
//	counterWheel = 0;
//	rightWheel = 0;
//	leftWheel = 0;
//
//	uint8_t flag = 0;
//	while(1){
//
//		lastTim3_Left = countTim3;
//		lastTim2_Right = countTim2;
//		laststraight = straight;
//		left = distanceLeft();
//		right = distanceRight();
//		straight = distanceStraight();
//	    static int last_left_encoder_count = 0;
//	    static int last_right_encoder_count = 0;
//	    int current_left_encoder_count = countTim3;
//	    int current_right_encoder_count = countTim2;
//	    int lap = 0;
//		if (current_left_encoder_count == last_left_encoder_count && current_right_encoder_count == last_right_encoder_count) {
//			if (left - right > 2.5){
//				backwardLeft();
//				HAL_Delay(30);
//				forwardGPIO();
//				lap++;
//			}else {
//				backwardRight();
//				HAL_Delay(30);
//				forwardGPIO();
//				lap++;
//			}
//    	}
//		else{
//			last_left_encoder_count = current_left_encoder_count;
//        	last_right_encoder_count = current_right_encoder_count;
//
//			if(straight < 5) {
//				stop();
//				return;
//			}
//			if(left < 5.2){
//
//				timer_Left(5);
//				flag = 0;
//			}
//			else if( right < 5.2){
//				timer_Right(5);
//				flag = 0;
//			}
//			else if(left > 5.6 && left < 12 && right > 5.6 && right < 12){
//				if(left < right){
//					timer_Left(20);
//					flag = 0;
//				}
//				else {
//					timer_Right(20);
//					flag = 0;
//				}
//			}
//
//			else if(left > 12 && right > 5.6 && right < 12){
//				timer_Left(5);
//				flag = 0;
//
//			}
//			else if(right > 12 && left > 5.6 && left < 12){
//				timer_Right(5);
//				flag = 0;
//			}
//			else{
//				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, RESET);
//				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
//				if(((countTim2 - lastTim2_Right) -(countTim3 - lastTim3_Left)) > 3){
//					timer_Left(5);
//					flag = 0;
//				}
//				else if(((countTim3 - lastTim3_Left) - (countTim2 - lastTim2_Right) ) > 3){
//					timer_Right(5);
//					flag = 0;
//				}
//			}
//			if(flag == 0){
//				flag = 1;
//				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
//				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRight); // right
//			}
//		}
//	}
//}


void backwardRight(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,3120); // right

}
void backwardLeft(){
	HAL_GPIO_WritePin(AI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, BI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, AI1_Pin,SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,3000); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,0); // right
}
void API_turnRight(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeft); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRightDefault); // right
	angle(-83);
	stop();
}
void API_turnLeft(){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,timerLeftDefault); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,timerRightDefault); // right
	angle(83);
	stop();
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

