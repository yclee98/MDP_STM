#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim10;

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

//extern double goDist;
extern uint8_t isMoving;
extern uint16_t SERVO_CENTER;
extern int16_t servoMultiplier;
extern double totalAngle;
extern uint8_t pidEnable;

extern int targetAngle;
extern int targetDistance;
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


void motorStart(){

	isMoving = 1;

	if(pidEnable == 0){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 2000);
	}

	//stop calculation interrupt
	HAL_TIM_Base_Start_IT(&htim10);

	//for pid/encoder interrupt
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

}

void motorStop(){
	resetCar();

	HAL_TIM_Base_Stop_IT(&htim7);
	HAL_TIM_Base_Stop_IT(&htim6);
	HAL_TIM_Base_Stop_IT(&htim10);
	isMoving = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
	htim1.Instance->CCR4 = SERVO_CENTER;//return wheel straight
	osDelay(50);
}

//1= forward, 0= reverse
void forward(int dir, double dist)
{
	htim1.Instance->CCR4 = SERVO_CENTER;
	setDirection(dir, 0);
	setSpeed(15);
	targetDistance = dist;
	osDelay(1000);

	if (dist != 0){
		motorStart();
	}

	int calPWM = SERVO_CENTER;

	//servo control
	while(isMoving){
		if(encoderC.direction == 1) //forward
			calPWM = (int)(SERVO_CENTER + totalAngle*servoMultiplier);
		else if(encoderC.direction == 0)//reverse
			calPWM = (int)(SERVO_CENTER - totalAngle*servoMultiplier);
		else
			calPWM = SERVO_CENTER;
		if(calPWM > 200)
			calPWM = 200;
		if(calPWM < 100)
			calPWM = 100;
		htim1.Instance->CCR4 = calPWM;
		osDelay(50);
	}
	osDelay(50);
}


void testMotorSpeed(){
	isMoving = 1;
	osDelay(50);
	setDirection(1,0);
	osDelay(50);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 2000);
	osDelay(50);
}

int addAngle(int angle){
	angle += totalAngle;

	if(angle >= 360)
		angle -=360;
	if(angle <= -360)
		angle +=360;

	return angle;
}

void turnLeft(int dir, int angle)
{
//	angle = addAngle(angle);
	setSpeed(5);
	htim1.Instance->CCR4 = 114;
	setDirection(dir, 0);
	osDelay(100);
	motorStart();
	targetAngle = angle;

	while(isMoving)
		osDelay(100);

	if (!dir)
		totalAngle +=angle;
	else
		totalAngle -= angle;

}

void turnRight(int dir, int angle)
{
//	angle = addAngle(-angle);

	setSpeed(5);
	htim1.Instance->CCR4 = 190;
	setDirection(dir, 0);
	osDelay(100);
	motorStart();
	targetAngle = angle;

	while(isMoving)
		osDelay(100);

	if (dir)
		totalAngle +=angle;
	else
		totalAngle -= angle;
}
