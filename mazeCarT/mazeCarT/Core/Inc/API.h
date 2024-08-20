/*
 * API.h
 *
 *  Created on: Aug 17, 2024
 *      Author: Minh Tuan
 */

#ifndef INC_API_H_
#define INC_API_H_

#include "main.h"


void forward();
void turn_right();
void turn_left();
void stop();
void timer_Right(uint8_t timer);
void timer_Left(uint8_t timer);
void forwardGPIO();
void backwardRight();
void backwardLeft();
#endif /* INC_API_H_ */
