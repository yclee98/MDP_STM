#include "gyro.h"
uint8_t ICM_addr = 0x68;
uint8_t buff[20];

extern I2C_HandleTypeDef hi2c1;

void readByte(uint8_t addr, uint8_t* data){
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICM_addr<<1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICM_addr<<1, data, 2, 20);
}

void writeByte(uint8_t addr, uint8_t data){
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICM_addr << 1, buff, 2, 20);
}

void gyro_Init(){
	writeByte(0x06, 0x00);
	osDelayUntil(10);
	writeByte(0x03, 0x80);
	osDelayUntil(10);
	writeByte(0x07, 0x07);
	osDelayUntil(10);
	writeByte(0x06, 0x01);
	osDelayUntil(10);
	writeByte(0x7F, 0x20);
	osDelayUntil(10);
	writeByte(0x01, 0x2F);
	osDelayUntil(10);
	writeByte(0x0, 0x00);
	osDelayUntil(10);
	writeByte(0x7F, 0x00);
	osDelayUntil(10);
	writeByte(0x07, 0x00);
	osDelayUntil(10);
}
