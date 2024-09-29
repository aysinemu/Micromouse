/*
 * encoder.c
 *
 *  Created on: Aug 3, 2024
 *      Author: Minh Tuan
 */



#include "encoder.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx.h"


#define ENCODER_COUNT 1300 // number of encoder steps per wheel revolution
#define MAX_ENCODERS 2     // maximum amount of encoders

/*reset encoder count is halt of max uint16. this helps to prevent faulty vlaue on overflow*/
const int reset_encoder_count = 65535 / 2;
uint8_t used_encoders = 0;

int32_t current_millis = 0, previous_millis = 0;

static float calculate_speed(int count, int interval);
static void init_decoder_TIM2(void);
static void init_decoder_TIM3(void);

uint8_t init_encoder(encoder_t * enc, uint8_t number_of_encoder)
{
    used_encoders = number_of_encoder+1;


    if (number_of_encoder+1 > MAX_ENCODERS)
        return 1;

    if (number_of_encoder == 0)
    {
        init_decoder_TIM2();
        enc->timer = (volatile uint32_t *) TIM2->CNT;
    }
    else
    {
        init_decoder_TIM3();
        enc->timer = (volatile uint32_t *) TIM3->CNT;
    }
   enc->current_count = 0;
    enc->position = 0;
    enc->speed= 0; // units per minute
     printf("used encoders %d \n", used_encoders);
    return 0;
}

static float calculate_speed(int count, int interval)
{
    // Calculate speed in units per minute
    //
    float speed = (count / (float)ENCODER_COUNT) * (60000.0 / interval);
    return speed;
}

// ...

void update_encoder(encoder_t *enc)
{
    /*
    Update general values for both encoders
    interval is the time passed since the last update. its measured in milliseconds

    */

    int interval, count_difference;
    current_millis = millis();
    interval = current_millis - previous_millis;

        enc->current_count = enc->timer;
        count_difference = enc->current_count - reset_encoder_count;
    /*poston sums driven distance */
        enc->position += (count_difference);
        enc->speed = calculate_speed(count_difference, interval);
    /*reset encoder count to prevent overflow*/
        enc->timer = reset_encoder_count;




    previous_millis = current_millis;
}

float get_speed_enc(encoder_t *enc)
{
    return enc->speed;
}

int32_t get_position_enc(encoder_t *enc)
{
    return enc->position;
}

void reset_position_enc(encoder_t *enc)
{
    enc->position = 0;
}

void print_info_enc(encoder_t *enc)
{
    // Print debug information
//     printf("Left: %ld, speed: %0.3f Pos: %ld \t Right: %ld, speed: %0.3f Pos: %ld  \n", left->current_count, left->speed, left->position, right->current_count, right->speed, right->position);
}



