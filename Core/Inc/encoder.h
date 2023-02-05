
#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

typedef struct{
	int16_t velocity;
	int64_t position; //the integral of velocity
	uint16_t lastCounterValue;
}encoder_instance;
void update_encoder(encoder_instance *e, TIM_HandleTypeDef *htim);
void encoder_reset(encoder_instance *e);

#endif /* INC_ENCODER_H_ */
