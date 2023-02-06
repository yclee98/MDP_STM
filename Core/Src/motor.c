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

extern encoder_instance encoderC, encoderD;
extern uint8_t OLED_row1[20];

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

//forward = 1, reverse = 0
//c=1, d=2, both=0
void setDirection(int dir, int motor)
{
	if (dir){// move forward
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
	else{ // reverse
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
	encoderC.direction = dir;
	encoderD.direction = dir;
}

//1= forward, 0= reverse
void forward(int dir, double dist)
{
//	float dist_error = 0.94; //120 cm exceeded by 7cm
//	dist = (int)(dist*dist_error);
	__disable_irq();
	resetCar();
	osDelay(10);
	goDist = dist;
	htim1.Instance->CCR4 = SERVO_CENTER;
	osDelay(500);
	setDirection(dir, 0);
	osDelay(100);
	isMoving = 1;
	__enable_irq();

	int avgDist = 0;
	static int count=0;
	//while(encoderC.distance < dist && encoderD.distance < dist){
	while(avgDist < dist){
		avgDist = (encoderC.distance+encoderD.distance)/2;
		sprintf(OLED_row1, "dist %d %d", avgDist, count);
		osDelay(10);
	}
	count++;

	motorStop();

	osDelay(50);
}

//void backward(double var)
//{
//	goDist = var;
//	htim1.Instance->CCR4 = SERVO_CENTER;
//	osDelay(50);
//	setDirection(0, 0);
////	direction = -1;
//	isMoving = 1;
//	osDelay(10);
//
//	int avgDist = 0;
//
//	while(avgDist < var){
//		avgDist = (encoderC.distance+encoderD.distance)/2;
//		osDelay(100);
//	}
//
//	motorStop();
//	resetCar();
//	osDelay(50);
//}

void motorStop(){
	__disable_irq();
	isMoving = 0;
	osDelay(50);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	osDelay(100);
	htim1.Instance->CCR4 = SERVO_CENTER;//return wheel straight
	__enable_irq();
	osDelay(100);
}

void testMotorSpeed(){
	isMoving = 1;
		osDelay(50);
	goDist = 200;
	setDirection(1,0);
	osDelay(50);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 2000);
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
