#include "motor.h"

extern TIM_HandleTypeDef htim1; //servo
extern TIM_HandleTypeDef htim8; //motor

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim10;

extern encoder_instance encoderC, encoderD;
extern pid_instance motorCpid, motorDpid;
extern pid_instance gyroPID;

extern uint16_t SERVO_CENTER;
extern int TURNING_MAX_SPEED;

extern uint8_t pidEnable;
extern uint8_t isMoving;
extern uint8_t isAngle;

extern double totalAngle;
extern double targetAngle;
extern double targetDistance;
extern int STRAIGHT_MAX_SPEED;

extern uint8_t OLED_row1[20], OLED_row4[20];

//forward = 1, reverse = 0
//c=1, d=2, both=0
void setDirection(int dir, int motor)
{
	if (dir){// move forward
		if (motor == 1) // C
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		}
		else if (motor == 2) // D
		{
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
		}
	}
	else{ // reverse
		if (motor == 1) // C
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		}
		else if (motor == 2) // D
		{
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
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
	__disable_irq();
	isMoving = 1;
//	totalAngle = 0.0;

	if(pidEnable == 0){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 5000);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 5000);
	}

	//for checking when to stop interrupt
	HAL_TIM_Base_Start_IT(&htim10);

	//for pid/encoder interrupt
	HAL_TIM_Base_Start_IT(&htim6); //motorD
	HAL_TIM_Base_Start_IT(&htim7); //motorC


	__enable_irq();
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

extern int MINSPEED;
extern int AVGSPEED;
extern int MAXSPEED;

//1= forward, 0= reverse
void forward(int dir, double dist)
{
	if(dist <=0 && dist > 300)
		return;


	htim1.Instance->CCR4 = SERVO_CENTER;
	setDirection(dir, 0);
//	setTarget(&motorCpid, 25); // MAX 25 for Accuracy
//	setTarget(&motorDpid, 25); // MAX 40 for speed but horrible accuracy
	targetDistance = dist;

	if (dist <= 30)
	{
		STRAIGHT_MAX_SPEED = MINSPEED;
	}
	else if (dist <= 60)
	{
		STRAIGHT_MAX_SPEED = AVGSPEED;
	}
	else
	{
		STRAIGHT_MAX_SPEED = AVGSPEED;
	}

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

		apply_pid_servo(&gyroPID, totalAngle);

		if(dir == 1){
			calPWM = (int)(SERVO_CENTER - gyroPID.output);
		}
		else if(dir == 0)//reverse
			calPWM = (int)(SERVO_CENTER+1 + gyroPID.output);

		if(calPWM > 249)
			calPWM = 249;
		if(calPWM < 99)
			calPWM = 99;
		//printVelocity(calPWM, 147);

		htim1.Instance->CCR4 = calPWM;

		//sprintf(OLED_row4, "servo %d", calPWM);

		osDelay(50);
	}
	osDelay(50);
}

void turnLeft(int dir, double angle) //radius = 24.5
{
	if(angle > 360 || angle <= 0)
		return;

	setTarget(&motorCpid, TURNING_MAX_SPEED);
	setTarget(&motorDpid, TURNING_MAX_SPEED*0.509257627); //4846855213416525

	htim1.Instance->CCR4 = 99;
	setDirection(dir, 0);
	osDelay(100);

	isAngle = 1;

	if(dir)
		targetAngle = angle;
	else
		targetAngle = -angle;

	motorStart();

	while(isMoving)
		osDelay(50);

	if (!dir)
		totalAngle +=angle;
	else
		totalAngle -= angle;

//	totalAngle = 0.0;

	isAngle = 0;
}

void turnRight(int dir, double angle) //radius = 24.3,25.45
{
	if(angle > 360 || angle < 0)
			return;

	setTarget(&motorCpid, TURNING_MAX_SPEED*0.596); //0.505463828125
	setTarget(&motorDpid, TURNING_MAX_SPEED);


	htim1.Instance->CCR4 = 249;
	setDirection(dir, 0);
	osDelay(100);

	isAngle = 1;

	if(dir)
		targetAngle = -angle;
	else
		targetAngle = angle;

	motorStart();

	while(isMoving)
		osDelay(50);

	if (dir)
		totalAngle +=angle;
	else
		totalAngle -= angle;

//	totalAngle = 0.0;

	isAngle = 0;
}

extern double ultrasonicDistance;

double sensorDistance(double targetDist){
	double forwardDist = 0;
	double uDist = 0;
	int buffer = 2;
	double distTravelled = 0;
	int checks = 2;
	for (;;)
	{
		//HCSR04_Read(); // Call Sensor
		uDist = getUltrasonicDistance();
		if (uDist <= 200 && uDist != -1) // Valid distance
		{
			if (uDist >= targetDist - buffer && uDist <= targetDist + buffer)
			{
				checks -= 1;
				if (checks <= 0)
					break;
			}
			else if (uDist <= targetDist)
			{
				checks = 2;
				forwardDist = targetDist-uDist;
				if(forwardDist >= buffer){
					forward(0, forwardDist);
					distTravelled-=forwardDist;
				}

			}
			else
			{
				checks = 2;
				forwardDist = uDist-targetDist;
				if(forwardDist >= buffer){
					forward(1, forwardDist);
					distTravelled+=forwardDist;
				}

			}
		}
		else
		{
			checks = 2;
			forward(1, 30);
			distTravelled+=30;
		}
	}
	return distTravelled+targetDist;
}
