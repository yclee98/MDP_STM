#include "motor.h"

extern TIM_HandleTypeDef htim1; //servo
extern TIM_HandleTypeDef htim8; //motor


extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim10;

extern encoder_instance encoderC, encoderD;
extern pid_instance motorCpid, motorDpid;

extern uint16_t SERVO_CENTER;
extern int16_t servoMultiplier;

extern uint8_t pidEnable;
extern uint8_t isMoving;
extern uint8_t isAngle;

extern double totalAngle;
extern int targetAngle;
extern int targetDistance;

extern uint8_t OLED_row1[20], OLED_row4[20];

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

void setMotorCPWM(){
	if(encoderC.direction){ //forward
		if(motorCpid.output > 0){
			setDirection(1,1);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, motorCpid.output);
		}else{
			setDirection(0,1);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, -motorCpid.output);
		}
	}else{ //reverse
		if(motorCpid.output > 0){
			setDirection(0,1);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, motorCpid.output);
		}else{
			setDirection(1,1);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, -motorCpid.output);
		}
	}
}

void setMotorDPWM(){
	if(encoderD.direction){ //forward
		if(motorDpid.output > 0){
			setDirection(1,2);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, motorDpid.output);
		}else{
			setDirection(0,2);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, -motorDpid.output);
		}
	}else{ //reverse
		if(motorDpid.output > 0){
			setDirection(0,2);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, motorDpid.output);
		}else{
			setDirection(1,2);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, -motorDpid.output);
		}
	}
}

void motorStart(){
	isMoving = 1;

	if(pidEnable == 0){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 2000);
	}

	//for checking when to stop interrupt
	HAL_TIM_Base_Start_IT(&htim10);

	//for pid/encoder interrupt
	HAL_TIM_Base_Start_IT(&htim6); //motorD
	HAL_TIM_Base_Start_IT(&htim7); //motorC

}

void motorStop(){
	isMoving = 0;
	__disable_irq();

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);

	HAL_TIM_Base_Stop_IT(&htim7);
	HAL_TIM_Base_Stop_IT(&htim6);
	HAL_TIM_Base_Stop_IT(&htim10);
	__enable_irq();

	resetCar();

	htim1.Instance->CCR4 = SERVO_CENTER;//return wheel straight
	osDelay(50);
}

pid_instance gyroPID;

//1= forward, 0= reverse
void forward(int dir, double dist)
{
	if(dist <=0 && dist > 300)
		return;
	htim1.Instance->CCR4 = SERVO_CENTER;
	setDirection(dir, 0);
	setTarget(&motorCpid, 25); // MAX 25 for Accuracy
	setTarget(&motorDpid, 25); // MAX 40 for speed but horrible accuracy
	targetDistance = dist;
	osDelay(1000);

	pid_reset(&gyroPID);

	if (dist != 0){
		motorStart();
	}else{
		return;
	}

	int calPWM = SERVO_CENTER;

	//servo control
	while(isMoving){
//		if(dir == 1) //forward
//			calPWM = (int)(SERVO_CENTER + totalAngle*servoMultiplier);
//		else if(encoderC.direction == 0)//reverse
//			calPWM = (int)(SERVO_CENTER - totalAngle*servoMultiplier);
//		else
//			calPWM = SERVO_CENTER;

		apply_pid1(&gyroPID, totalAngle);

		if(dir == 1){
			calPWM = (int)(SERVO_CENTER - gyroPID.output);
		}
		else if(dir == 0)//reverse
			calPWM = (int)(SERVO_CENTER + gyroPID.output);

		if(calPWM > 249)
			calPWM = 249;
		if(calPWM < 99)
			calPWM = 99;
//		printVelocity(calPWM, 0);

		htim1.Instance->CCR4 = calPWM;

		sprintf(OLED_row4, "servo %d", calPWM);

		osDelay(50);
	}


	osDelay(50);
}

int addAngle(double angle){
	angle += totalAngle;

	if(angle >= 360)
		angle -=360;
	if(angle <= -360)
		angle +=360;

	return angle;
}

void turnLeft(int dir, double angle)
{
	if(angle > 360 || angle <= 0)
		return;
	setTarget(&motorDpid, 10.0);
	setTarget(&motorCpid, 10.0*0.4846855213416525);
	htim1.Instance->CCR4 = 99;
	setDirection(dir, 0);
	osDelay(500);

	isAngle = 1;
	if(dir)
		targetAngle = angle;
	else
		targetAngle = -angle;

	motorStart();

	while(isMoving)
		osDelay(100);

	if (!dir)
		totalAngle +=angle;
	else
		totalAngle -= angle;

	isAngle = 0;
}

void turnRight(int dir, double angle)
{
	if(angle > 360 || angle < 0)
			return;

	setTarget(&motorDpid, 10.0*0.505463828125);
	setTarget(&motorCpid, 10.0);

	htim1.Instance->CCR4 = 249;
	setDirection(dir, 0);
	osDelay(500);
	isAngle = 1;

	if(dir)
		targetAngle = -angle;
	else
		targetAngle = angle;

	motorStart();

	while(isMoving)
		osDelay(100);

	if (dir)
		totalAngle +=angle;
	else
		totalAngle -= angle;

	isAngle = 0;
}
