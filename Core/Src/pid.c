#include "pid.h"

#define PID_MAX 7000

#define SAMPLING_RATE 200.0 //5ms

//need to adjust servomultiplier when changing speed
extern int16_t MOTOR_VELOCITY_REF;

extern uint8_t OLED_row5[20];

//void setSpeed(int16_t speed){
//	MOTOR_VELOCITY_REF = speed;
//}

void setTarget(pid_instance *m, double target){
	m->target = target;
}


float kp = 120;
float ki = 0.0010;
float kd = 0;
float motorSamplingRate = 200.0; //1/200=5ms == 0.005s

void apply_pid(pid_instance *m, int16_t measuredVelocity){
//	int32_t inputError = MOTOR_VELOCITY_REF - measuredVelocity;
	double inputError = m->target - measuredVelocity;

	m->errorIntegral += inputError * motorSamplingRate;

	double errorChange = inputError - m->lastError;
	m->lastError = inputError;

	double errorRate = errorChange / motorSamplingRate;

	m->output =
			kp * inputError +
			ki * m->errorIntegral +
			kd * errorRate;

	if(m->output >= PID_MAX)
		m->output = PID_MAX;
	else if(m->output <= -PID_MAX)
		m->output = -PID_MAX;

//	sprintf(OLED_row5, "err %d", (int)m->errorIntegral);

}

void setPID(float p, float i, float d){
	kp = p;
	ki = i;
	kd = d;
}

float kp1 = 2; //0.5
double ki1 = 0.0001; //0.000001
float kd1 = 0; //500
float gyroSamplingRate = 200.0;

void apply_pid1(pid_instance *m, double measuredGyro){

	int32_t inputError = 0 - measuredGyro;
	m->errorIntegral += inputError * gyroSamplingRate;

	double errorChange = inputError - m->lastError;
	m->lastError = inputError;

	double errorRate = errorChange / gyroSamplingRate;

	m->output =
			kp1 * inputError +
			ki1 * m->errorIntegral +
			kd1 * errorRate;

	sprintf(OLED_row5, "errg %d", (long)m->errorIntegral);
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
