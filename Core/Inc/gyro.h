#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include "main.h"

void gyro_Init();
void writeByte(uint8_t addr,uint8_t data);
void readByte(uint8_t addr, uint8_t* data);

#endif /* INC_GYRO_H_ */
