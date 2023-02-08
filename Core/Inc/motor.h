
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"





void setDirection(int dir, int motor);
void motorStart();
void motorStop();
void forward(int dir, double dist);
void turnLeft(int dir,int angle);
void turnRight(int dir,int angle);

void testMotorSpeed();

#endif /* INC_MOTOR_H_ */
