
#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct{
	int16_t lastError;
	int32_t errorIntegral;
	int16_t output; //the pid calculated value to set on the motor
}pid_instance;

void setPID(float p, float i, float d);
void apply_pid(pid_instance *m, int16_t measuredVelocity, uint32_t deltaTime);
void pid_reset(pid_instance *m);

#endif /* INC_PID_H_ */
