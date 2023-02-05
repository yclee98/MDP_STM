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
int16_t pwmMax = 5000; // Maximum PWM value = 7200 keep the maximum value too 7000
int16_t pwmMin = 600;

extern double goDist;

extern uint8_t isMoving;
extern uint8_t direction;

void Motor_Init(){

}

void resetMotor(Motor *motor)
{
	motor->err = 0;
	motor->angle = 0;
	//motor->error_area -= motor->error*motor->dt;
	motor->htim->Instance->CNT = 0;
	motor->reset = 1;
	motor->pwmVal = 0;
	motor->start = 0;
}

void stopMotor(Motor *motor)
{
	motor->angle = 0;
	motor->error = target_angle - motor->angle;
	motor->error_old = 0;
	motor->error_area = 0;
	motor->error_rate = 0;
	motor->err = 0;
	motor->htim->Instance->CNT = 0;
	motor->reset = 0;
	motor->start = 0;
	motor->pwmVal = 0;
	motor->position = 0;
}

int16_t PID_Control(Motor *motor, int flipped){
	//Control Loop
	int16_t pwmVal = 0;
	if (abs(motor->error)>2){ //more than 2 degree difference
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer
		motor->angle = (int)(motor->position*360/265);  // supposed to be 260 tick per revolution?
		if (flipped)
			motor->angle = -motor->angle;
		motor->error = target_angle - motor->angle;

		if (motor->error > 0)
			setDirection(1, motor->motor); //forward
		else
			setDirection(0, motor->motor); //reverse direction

		motor->millisNow = HAL_GetTick();
		if (!motor->reset)
			motor->dt = (motor->millisNow - motor->millisOld); // time elapsed in millisecond
		motor->millisOld = motor->millisNow; // store the current time for next round

		if (!motor->reset)
		{
			motor->error_area = motor->error_area + motor->error*motor->dt; // area under error for Ki
			motor->error_change = motor->error - motor->error_old; // change in error
			motor->error_old = motor->error; //store the error for next round
			motor->error_rate = motor->error_change/motor->dt; // for Kd
		}

		pwmVal = (int)(motor->error*Kp + motor->error_area*Ki + motor->error_rate*Kd);  // PID

		//pwmVal = 2000;   // overwrite PID above, minimum pwmVal = 1000

		if (pwmVal > pwmMax)  // Clamp the PWM to its maximum value
			pwmVal = pwmMax;

		motor->reset = 0;
		return(pwmVal);
		//__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,pwmVal); // output the valie
	} // if loop
}

//isForward 1,0
//motor 1=c,2=d, 0=both
void setDirection(bool isForward, int motor)
{
	if (isForward){// move forward
		if (motor == 1) // C
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- forward
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		}
		else if (motor == 2) // D
		{
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- forward
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- forward
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- forward
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
		}

	}
	else { // reverse
		if (motor == 1) // C
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- reverse
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		}
		else if (motor == 2) // D
		{
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- reverse
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel D- reverse
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel D- reverse
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
		}
	}
}

void forward(double var)
{
	goDist = var;
	htim1.Instance->CCR4 = 147;
	osDelay(50);
	setDirection(1, 0);
	isMoving = 1;
	direction = 1;
}

void backward(double var)
{
	goDist = var;
	htim1.Instance->CCR4 = 147;
	osDelay(50);
	setDirection(0, 0);
	isMoving = 1;
	direction = -1;
}


void motorStop(){
	isMoving = 0;
	osDelay(50);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	osDelay(50);
}

void testMotorSpeed(){
	goDist = 200;
	setDirection(1,0);
	osDelay(50);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 1500);
	osDelay(50);
}



void turnLeft()
{
	htim1.Instance->CCR4 = 85;
	setDirection(1, 0);
}

void turnRight()
{
	htim1.Instance->CCR4 = 250;
	setDirection(1, 0);
}
