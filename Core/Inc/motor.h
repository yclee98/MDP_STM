
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
struct motor
{
	//Motor
	uint32_t counter;    // Timer C counter
	int16_t count;       // Convert counter to signed value
	int16_t speed;        // speed in term of number of edges detected per Systick
	int start;             // use to start stop the motor
	int16_t pwmVal;       //pwm value to control motor speed
	int err; // status for checking return
	int16_t position;    // position of the motor (1 rotation = 260 count)
	int16_t angle;        // angle of rotation, in degree resolution = 360/260
	int16_t position_target; // target position
	int16_t error;           // error between target and actual
	int32_t error_area;  // area under error - to calculate I for PI implementation
	float error_rate; // to calculate D for PID control
	int32_t error_old, error_change;
	int32_t millisOld, millisNow, dt; // to calculate I and D for PID control
	int motor; // Indicate motor
	int reset; // Indicate if motor has been reset
	TIM_HandleTypeDef *htim; // Encoder Timer
} typedef Motor;

void Motor_Init();
void ServoCenter();
void stopMotor(Motor *motor);
void resetMotor(Motor *motor);
void setDirection(int dir, int motor);
void forward(int dir, double dist);
void motorStop();
void motorStart();
void turnLeft(int dir,int angle);
void turnRight(int dir,int angle);

void testMotorSpeed();

#endif /* INC_MOTOR_H_ */
