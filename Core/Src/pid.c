#include "pid.h"

#define PID_MAX 7000

#define SAMPLING_RATE 5000 //5ms

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

//void setSpeed(int16_t speed){
//	MOTOR_VELOCITY_REF = speed;
//}

void setTarget(pid_instance *m, int16_t target){
	m->target = target;
}


void apply_pid(pid_instance *m, int16_t measuredVelocity){
//	int32_t inputError = MOTOR_VELOCITY_REF - measuredVelocity;
	int32_t inputError = m->target - measuredVelocity;

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

float kp1 = 0.5;
float ki1 = 0.000001;
float kd1 = 500;

void apply_pid1(pid_instance *m, int16_t measuredGyro){

	int32_t inputError = 0 - measuredGyro;
	m->errorIntegral += inputError * SAMPLING_RATE;

	int32_t errorChange = inputError - m->lastError;
	m->lastError = inputError;

	int32_t errorRate = errorChange / SAMPLING_RATE;

	m->output =
			kp1 * inputError +
			ki1 * m->errorIntegral +
			kd1 * errorRate;

//	if(m->output >= 200)
//		m->output = 200;
//	else if(m->output <= 100)
//		m->output = 100;

	osDelay(50);
}


void pid_reset(pid_instance *m){
	m->lastError = 0;
	m->errorIntegral = 0;
	m->output = 0;
	m->target = 0;
}





//void apply_pid1(pid_instance *m, int16_t measuredVelocity)
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
