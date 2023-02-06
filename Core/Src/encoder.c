#include "encoder.h"

void update_encoder(encoder_instance *e, TIM_HandleTypeDef *htim){
	uint32_t tempCounter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t firstTime = 0;
	if(firstTime == 0){
		e -> velocity = 0;
		firstTime = 1;
	}
	else{
		if(tempCounter == e->lastCounterValue)
			e->velocity = 0;
		else if(tempCounter > e->lastCounterValue){
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
				e->velocity = -(e->lastCounterValue) - (__HAL_TIM_GET_AUTORELOAD(htim)-tempCounter);
			else
				e->velocity = tempCounter - e->lastCounterValue;
		}
		else{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
				e->velocity = tempCounter - e->lastCounterValue;
			else
				e->velocity = tempCounter + (__HAL_TIM_GET_AUTORELOAD(htim)-e->lastCounterValue);
		}
	}
//	e->position += e->velocity;
	e->lastCounterValue = tempCounter;
	//osDelay(50);
}

void encoder_reset(encoder_instance *e){
	e->velocity = 0;
	e->distance = 0;
	e->lastCounterValue = 0;
}
