#include "encoder.h"

extern double fullRotationWheel;
extern double circumferenceWheel;

void update_encoder(encoder_instance *e, TIM_HandleTypeDef *htim){
	int cnt1, cnt2, diffC;
	cnt2 = __HAL_TIM_GET_COUNTER(htim);
	cnt1 = e->lastCounterValue;

	if(cnt2==cnt1){
		diffC=0;
	}
	else if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
	{
		if(cnt2 <= cnt1)
			diffC = cnt1 - cnt2;
		else
			diffC = (65535 - cnt2) + cnt1; //problem
	}
	else
	{
		if(cnt2 >= cnt1)
			diffC = cnt2 - cnt1;
		else
			diffC = (65535 - cnt1) + cnt2; //problem
	}

	//not sure why but something diffC very large number
	//seem like the counter value became too close
	//it went to the overflow line 65535 - ...
	//then when abs it it will return a negative which is probably overflow
	//this abs return negative most probably because of int16_t for velocity which is short
	//diff is using int which is 32bit
	//can manually change it to positive = workaround
	e->velocity = abs(diffC);
	if(e->velocity < 0)
		e->velocity = -e->velocity;

	e->distance += e->velocity/fullRotationWheel*circumferenceWheel;

	e->lastCounterValue = cnt2;
	osDelay(50);
}

void encoder_reset_counter(encoder_instance *e){
	e->lastCounterValue = 0;
}

void encoder_reset(encoder_instance *e){
	e->velocity = 0;
	e->distance = 0.0;
	e->direction = 1;
}
