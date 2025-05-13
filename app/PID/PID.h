#ifndef __PID_H
#define __PID_H

#define MOTOR_REDUCTION_RATIO 63  //Motor reduction ratio
#define PULSE_PRE_ROUND 20 //The number of pulses per turn of the encoder
#define RADIUS_OF_TIRE 34 //Tire radius, in millimeters
#define CIRCLES_OF_TIRE RADIUS_OF_TIRE * 2 * 3.14
#define ROUND_TIME 50.0f

#include "main.h"
#include "encoder.h"
#include "A4950.h"

void PID_Control_Left(float target_speed_left);
void PID_Control_Right(float target_speed_right);
float getLeftSpeed(Encoder_HandleTypeDef *enc);
float getRightSpeed(Encoder_HandleTypeDef *enc);

#endif