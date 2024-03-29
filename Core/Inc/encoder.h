
#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

typedef struct{
	int16_t velocity;
	uint16_t lastCounterValue;
	double distance;
	int direction;
}encoder_instance;

void update_encoder(encoder_instance *e, TIM_HandleTypeDef *htim);
void encoder_reset(encoder_instance *e);
void encoder_reset_counter(encoder_instance *e, TIM_HandleTypeDef *htim);

#endif /* INC_ENCODER_H_ */
