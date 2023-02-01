#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern int16_t oldposC;    // // see SysTick_Handler in stm32f4xx_it.c
extern int16_t oldposD;    // // see SysTick_Handler in stm32f4xx_it.c

//Motor
int16_t no_of_tick = 50; // number of tick used in SysTick to calculate speed, in msec
int16_t target_angle = 0; // target angle of rotation,
int16_t Kp = 0;
float Kd = 0;
float Ki = 0;
int16_t rpm = 0;         // speed in rpm number of count/sec * 60 sec  divide by 260 count per round
int16_t pwmMax = 3000; // Maximum PWM value = 7200 keep the maximum value too 7000

//gyro
extern double totalAngle;
extern uint8_t OLED_row5[20], OLED_row4[20];;

void Motor_Init(){
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

int16_t PID_Control(Motor *motor, int flipped){
	//Control Loop
	if (abs(motor->error)>2){ //more than 2 degree difference
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer
		motor->angle = (int)(motor->position*360/265);  // supposed to be 260 tick per revolution?
		if (flipped)
			motor->angle = -motor->angle;
		motor->error = target_angle - motor->angle;

		if (motor->error > 0)
			setDirection(1); //forward
		else
			setDirection(0); //reverse direction

		motor->millisNow = HAL_GetTick();
		motor->dt = (motor->millisNow - motor->millisOld); // time elapsed in millisecond
		motor->millisOld = motor->millisNow; // store the current time for next round

		motor->error_area = motor->error_area + motor->error*motor->dt; // area under error for Ki

		motor->error_change = motor->error - motor->error_old; // change in error
		motor->error_old = motor->error; //store the error for next round
		motor->error_rate = motor->error_change/motor->dt; // for Kd

		motor->pwmVal = (int)(motor->error*Kp + motor->error_area*Ki + motor->error_rate*Kd);  // PID

		//pwmVal = 2000;   // overwrite PID above, minimum pwmVal = 1000

		if (motor->pwmVal > pwmMax)  // Clamp the PWM to its maximum value
			motor->pwmVal = pwmMax;

		return(motor->pwmVal);
		//__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal); // output the valie
	} // if loop
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
}

void backward()
{
	setDirection(0);
}

void turnLeft()
{
	htim1.Instance->CCR4 = 85;
	setDirection(1);
}

void turnRight()
{
	htim1.Instance->CCR4 = 250;
	setDirection(1);
}
