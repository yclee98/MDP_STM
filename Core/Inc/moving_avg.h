
#ifndef INC_MOVING_AVG_H_
#define INC_MOVING_AVG_H_

#include "main.h"

#define MOVING_AVERAGE_LENGTH  50

typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int16_t out;
	int32_t sum;
}mov_aver_intance;

void reset_average_filter(mov_aver_intance* instance);
void apply_average_filter(mov_aver_intance* instance, int16_t input);

#endif /* INC_MOVING_AVG_H_ */
