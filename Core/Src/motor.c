#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;

extern uint16_t pwmVal; // Speed of the Robot
extern uint16_t maxPwmVal; // Max Speed of the Robot
extern uint16_t minPwmVal; // Min Speed of the Robot
extern uint16_t pwmValC;	// Speed of wheel C
extern uint16_t pwmValD;	// Speed of wheel D

extern int speedDiff;
extern double totalAngle;
extern uint8_t OLED_row5[20], OLED_row4[20];;

void Motor_Init(){
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

}

void setSpeed(uint16_t speed)
{
	if(speed == 0){

	}
	else if (speed > maxPwmVal)
	{
		speed=maxPwmVal;
	}
	else if (speed < minPwmVal)
	{
		speed=minPwmVal;
	}

	pwmVal = speed;
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwmVal);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwmVal);
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

void forward(int var)
{
	int calPWM = 0;

	htim1.Instance->CCR4 = var;//145;
	osDelay(50);
	setDirection(1);
	osDelay(200);
	totalAngle = 0;

	setSpeed(1200);

	uint32_t tick = HAL_GetTick();

	while(HAL_GetTick() - tick < 7000L){
		sprintf(OLED_row5, "cal %d", calPWM);
		sprintf(OLED_row4, "ta %d", (int)totalAngle);
		osDelay(50);
		calPWM = (int)(150 + totalAngle*5);
		if(calPWM > 200)
		   calPWM = 200;
	    if(calPWM < 100)
		   calPWM = 100;
	   htim1.Instance->CCR4 = calPWM;
	   osDelay(50);
	}
	setSpeed(0);


}

void backward()
{
	htim1.Instance->CCR4 = 150;

	setDirection(0);
	setSpeed(2000);
	osDelay(5000);

	setSpeed(0);
}

void test()
{
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- forward
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- forward
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
}

void testrun()
{
	test();

	setSpeed(2000);

		osDelay(5000);

		setSpeed(0);
}

void turnLeft()
{
	htim1.Instance->CCR4 = 85;
	setDirection(1);
	setSpeed(2000);

	osDelay(5000);

	setSpeed(0);
}

void turnRight()
{
	htim1.Instance->CCR4 = 250;
	setDirection(1);
	setSpeed(2000);

	osDelay(5000);

	setSpeed(0);
}
