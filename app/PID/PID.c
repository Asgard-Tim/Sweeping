#include "PID.h"

volatile int32_t leftCountGlobal  = 0;
volatile int32_t rightCountGlobal = 0;
volatile int32_t leftCount  = 0;
volatile int32_t rightCount = 0;
float leftSpeed = 0.0f;
float rightSpeed = 0.0f;

float time = ROUND_TIME / 1000.0f;

float kp_left = 0.1f;
float ki_left = 0.1f;
float kd_left = 0.001f;
float previous_error_left = 0.0f;
float error_left = 0.0f;
float integral_left = 0.0f;
float derivative_left = 0.0f;
float pid_output_left = 0.0f;

float kp_right = 0.1f;
float ki_right = 0.1f;
float kd_right = 0.001f;
float previous_error_right = 0.0f;
float error_right = 0.0f;
float integral_right = 0.0f;
float derivative_right = 0.0f;
float pid_output_right = 0.0f;

void PID_Control_Left(float target_speed_left){
		error_left = target_speed_left - leftSpeed;
		
		integral_left += error_left;
		if (integral_left > 99.0f)
			integral_left = 99.0f;
		else if (integral_left < - 99.0f)
			integral_left = -99.0f;
		
		derivative_left = error_left - previous_error_left;
		
		pid_output_left = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
		
		if (pid_output_left > 99.0f)
			pid_output_left = 99.0f;
		else if (pid_output_left < - 99.0f)
			pid_output_left = -99.0f;
		
		// 驱动电机
		A4950_SetLeft(pid_output_left * 7);
		
		previous_error_left = error_left;
}

void PID_Control_Right(float target_speed_right){
		error_right = target_speed_right - rightSpeed;
		
		integral_right += error_right;
		if (integral_right > 99.0f)
			integral_right = 99.0f;
		else if (integral_right < - 99.0f)
			integral_right = -99.0f;
		
		derivative_right = error_right - previous_error_right;
		
		pid_output_right = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
		
		if (pid_output_right > 99.0f)
			pid_output_right = 99.0f;
		else if (pid_output_right < - 99.0f)
			pid_output_right = -99.0f;
		
		// 驱动电机
		A4950_SetRight(pid_output_right * 7);
		
		previous_error_right = error_right;
}

float getLeftSpeed(Encoder_HandleTypeDef *enc){
		leftCount = leftCountGlobal;
		leftCountGlobal  = Encoder_GetCount(enc);
		leftCount -= leftCountGlobal;
		leftSpeed = leftCount / time / PULSE_PRE_ROUND / MOTOR_REDUCTION_RATIO * CIRCLES_OF_TIRE / 10; // cm/s
		return leftSpeed;
}

float getRightSpeed(Encoder_HandleTypeDef *enc){
		rightCount = rightCountGlobal;
		rightCountGlobal = - Encoder_GetCount(enc);
		rightCount -= rightCountGlobal;
		rightSpeed = rightCount / time / PULSE_PRE_ROUND / MOTOR_REDUCTION_RATIO * CIRCLES_OF_TIRE / 10; // cm/s
		return rightSpeed;
}