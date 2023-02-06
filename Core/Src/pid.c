#include "pid.h"

#define PID_MAX  4000
#define PID_MIN 700

//500 for 100milisec 4 0.8 0.2

int16_t MOTOR_VELOCITY_REF = 1000;

float kp = 3;
float ki = 0.5;
float kd = 0.1;

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
