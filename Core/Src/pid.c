#include "pid.h"

#define PID_MAX 6000

extern uint8_t OLED_row5[20];

void setTarget(pid_instance *m, double target){
	m->target = target;
}

extern float KP_MOTOR;
extern double KI_MOTOR;
extern float KD_MOTOR;
float motorSamplingRate = 200.0; //1/200=5ms == 0.005s

void apply_pid(pid_instance *m, int16_t measuredVelocity){
	int32_t inputError = m->target - measuredVelocity;
	m->errorIntegral += inputError;

	m->output =
			KP_MOTOR * inputError +
			KI_MOTOR * (m->errorIntegral)/motorSamplingRate +
			KD_MOTOR * (inputError-m->lastError)* motorSamplingRate;

	if(m->output >= PID_MAX)
		m->output = PID_MAX;
	else if(m->output <= 400)
		m->output = 400;

	m->lastError = inputError;
	osDelay(50);
}


//void apply_pid(pid_instance *m, int16_t measuredVelocity){
//	double inputError = m->target - measuredVelocity;
//
//	m->errorIntegral += inputError * motorSamplingRate;
//
//	double errorChange = inputError - m->lastError;
//	m->lastError = inputError;
//
//	double errorRate = errorChange / motorSamplingRate;
//
//	m->output =
//			KP_MOTOR * inputError +
//			KI_MOTOR * m->errorIntegral +
//			KD_MOTOR * errorRate;
//
//	if(m->output >= PID_MAX)
//		m->output = PID_MAX;
//	else if(m->output <=400)
//		m->output = 400;
//
////	sprintf(OLED_row5, "err %d", (int)m->errorIntegral);
//
//}

extern float KP_SERVO;
extern double KI_SERVO;
extern float KD_SERVO;
float gyroSamplingRate = 200.0;

void apply_pid_servo(pid_instance *m, double measuredGyro){
	int32_t inputError = 0 - measuredGyro;
	m->errorIntegral += inputError * gyroSamplingRate;

	double errorChange = inputError - m->lastError;
	m->lastError = inputError;

	double errorRate = errorChange / gyroSamplingRate;

	m->output =
			KP_SERVO * inputError +
			KI_SERVO * m->errorIntegral +
			KD_SERVO * errorRate;

//	sprintf(OLED_row5, "errg %d", (long)m->errorIntegral);
}


void pid_reset(pid_instance *m){
	m->lastError = 0;
	m->errorIntegral = 0;
	m->output = 0;
	m->target = 0;
}


