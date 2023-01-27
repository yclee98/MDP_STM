#include "motor.h"

extern TIM_HandleTypeDef htim1;

extern uint16_t pwmVal; // Speed of the Robot
extern uint16_t maxPwmVal; // Max Speed of the Robot
extern uint16_t minPwmVal; // Min Speed of the Robot
extern uint16_t pwmValC;	// Speed of wheel C
extern uint16_t pwmValD;	// Speed of wheel D

extern int speedDiff;

void Motor_Init(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void ServoCenter(){
	htim1.Instance->CCR1 =150;
}

void setSpeed(uint16_t speed)
{
	if (speed > maxPwmVal)
	{
		return;
	}

	if (speed < minPwmVal)
	{
		return;
	}

	pwmVal = speed;
}

void setDirection(bool isForward)
{
	if (isForward){// move forward
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- forward
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- forward
		HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	}
	else { // reverse
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- reverse
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- reverse
		HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
	}
}

void forward()
{

	htim1.Instance->CCR1 = 75;
	setDirection(1);
	setSpeed(2000);

	osDelay(5000);

	setSpeed(0);
}

void backward()
{
	htim1.Instance->CCR1 = 75;

	setDirection(0);
	setSpeed(2000);
	osDelay(5000);

	setSpeed(0);
}

void turnLeft()
{
	htim1.Instance->CCR1 = 110;
	setDirection(1);
	setSpeed(2000);

	osDelay(5000);

	setSpeed(0);
}

void turnRight()
{
	htim1.Instance->CCR1 = 50;
	setDirection(1);
	setSpeed(2000);

	osDelay(5000);

	setSpeed(0);
}
