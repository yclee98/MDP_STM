#include "motor.h"

extern TIM_HandleTypeDef htim1;

void Motor_Init(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void ServoCenter(){
	htim1.Instance->CCR4 =150;
}

