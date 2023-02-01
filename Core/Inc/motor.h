
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/*
 * center = 150
 * extreme right = 250
 * extreme left = 85
 */
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
	int32_t error_old, error_change;
	float error_rate; // to calculate D for PID control
	int32_t millisOld, millisNow, dt; // to calculate I and D for PID control
} typedef Motor;

void Motor_Init();
void ServoCenter();
void setSpeed(uint16_t speed);
void setDirection(bool isForward);
void forward();
void backward();
void turnLeft();
void turnRight();

#endif /* INC_MOTOR_H_ */
