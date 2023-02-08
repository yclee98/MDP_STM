#include "pid.h"

#define PID_MAX 4000
#define PID_MIN 1100

#define SAMPLING_RATE 5000 //10ms, 5ms
#define INTEGRAL_GAIN_MAX  2000000

//need to adjust servomultiplier when changing speed
extern int16_t MOTOR_VELOCITY_REF;

float kp = 50;
float ki = 0.0005;
float kd = 200;

void setPID(float p, float i, float d){
	kp = p;
	ki = i;
	kd = d;
}

void setSpeed(int16_t speed){
	MOTOR_VELOCITY_REF = speed;
}


void apply_pid1(pid_instance *m, int16_t measuredVelocity){
	int32_t inputError = MOTOR_VELOCITY_REF - measuredVelocity;
	m->errorIntegral += inputError * SAMPLING_RATE;

	int32_t errorChange = inputError - m->lastError;
	m->lastError = inputError;

	int32_t errorRate = errorChange / SAMPLING_RATE;

	m->output =
			kp * inputError +
			ki * m->errorIntegral +
			kd * errorRate;

	if(m->output >= PID_MAX)
		m->output = PID_MAX;
	else if(m->output <= -PID_MAX)
		m->output = -PID_MAX;


	osDelay(50);
}


	//	int32_t inputError = MOTOR_VELOCITY_REF - measuredVelocity;
//	m->errorIntegral += inputError;
//
//	if(m->errorIntegral > INTEGRAL_GAIN_MAX)
//		m->errorIntegral = INTEGRAL_GAIN_MAX;
//	else if(m->errorIntegral < -INTEGRAL_GAIN_MAX)
//		m->errorIntegral = -INTEGRAL_GAIN_MAX;
//
//	m->output =
//			kp * inputError +
//			ki * (m->errorIntegral)/SAMPLING_RATE +
//			kd * (inputError-m->lastError)* SAMPLING_RATE;
//
//	if(m->output >= PID_MAX)
//		m->output = PID_MAX;
//	else if(m->output <= -PID_MAX)
//		m->output = -PID_MAX;
//
//	m->lastError = inputError;
//	osDelay(50);
//}

void apply_pid(pid_instance *m, int16_t measuredVelocity, uint32_t deltaTime){
	int32_t inputError =MOTOR_VELOCITY_REF - measuredVelocity;
	m->errorIntegral += inputError / deltaTime;

	m->output =
			kp * inputError +
			ki * (m->errorIntegral)+
			kd * (inputError-m->lastError)*deltaTime;

	if(m->output >= PID_MAX)
		m->output = PID_MAX;
	//to ensure car wont go below min pid which can cause car to slow down alot
	else if(m->output >= 0 && m->output < PID_MIN)
		m->output = PID_MIN;
	else if(m->output < -PID_MAX){ //if need to change direction
		m->output = -PID_MAX;
	}


	m->lastError = inputError;
}

void pid_reset(pid_instance *m){
	m->lastError = 0;
	m->errorIntegral = 0;
	m->output = 0;
}
