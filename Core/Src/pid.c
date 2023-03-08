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
	double inputError = m->target - measuredVelocity;

	m->errorIntegral += inputError * motorSamplingRate;

	double errorChange = inputError - m->lastError;
	m->lastError = inputError;

	double errorRate = errorChange / motorSamplingRate;

	m->output =
			KP_MOTOR * inputError +
			KI_MOTOR * m->errorIntegral +
			KD_MOTOR * errorRate;

	if(m->output >= PID_MAX)
		m->output = PID_MAX;
	else if(m->output <=0)
		m->output = 0;

//	sprintf(OLED_row5, "err %d", (int)m->errorIntegral);

}

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
