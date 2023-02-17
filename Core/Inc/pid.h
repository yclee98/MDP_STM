
#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct{
	double lastError;
	double errorIntegral;
	int16_t output;
	double target;
}pid_instance;

void setTarget(pid_instance *m, double target);
//void setSpeed(int16_t speed);
void setPID(float p, float i, float d);
void apply_pid(pid_instance *m, int16_t measuredVelocity);
void pid_reset(pid_instance *m);

void apply_pid1(pid_instance *m, double measuredGyro);

#endif /* INC_PID_H_ */
