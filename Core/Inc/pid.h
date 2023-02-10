
#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct{
	int16_t lastError;
	int32_t errorIntegral;
	int16_t output;
	int16_t target;
}pid_instance;

void setTarget(pid_instance *m, int16_t target);
//void setSpeed(int16_t speed);
void setPID(float p, float i, float d);
void apply_pid(pid_instance *m, int16_t measuredVelocity);
void pid_reset(pid_instance *m);

void apply_pid1(pid_instance *m, int16_t measuredGyro);

#endif /* INC_PID_H_ */
