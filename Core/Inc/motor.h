
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <stdbool.h>
/*
 * center = 150
 * extreme right = 250
 * extreme left = 85
 */

void Motor_Init();
void ServoCenter();
void setSpeed(uint16_t speed);
void setDirection(bool isForward);
void forward();
void backward();
void turnLeft();
void turnRight();

#endif /* INC_MOTOR_H_ */
