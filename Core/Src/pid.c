#include "pid.h"

#define PID_MAX  4000
#define PID_MIN 700

int16_t MOTOR_VELOCITY_REF = 400; //abt 226vel

float kp = 5;
float ki = 0.8;
float kd = 0.2;

void setPID(float p, float i, float d){
	kp = p;
	ki = i;
	kd = d;
}

void apply_pid(pid_instance *m, int16_t measuredVelocity, uint32_t deltaTime){
	int32_t inputError =MOTOR_VELOCITY_REF - measuredVelocity;
	m->errorIntegral += inputError / deltaTime;

	m->output =
			kp * inputError +
			ki * (m->errorIntegral)+
			kd * (inputError-m->lastError)*deltaTime;

	if(m->output >= PID_MAX)
		m->output = PID_MAX;
	else if(m->output <= -PID_MAX)
		m->output = -PID_MAX;

	m->lastError = inputError;
}

void pid_reset(pid_instance *m){
	m->lastError = 0;
	m->errorIntegral = 0;
	m->output = 0;
}
