/*
 * https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
 */
#include "gyro.h"
uint8_t ICM_addr = 0x68; //The slave address of the ICM-20948 is b110100X which is 7 bits long
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
	writeByte(0x06, 0x00); //0xC1 PWR_MGMT1
	osDelayUntil(10);
	writeByte(0x03, 0x80); //USER_CTRL
	osDelayUntil(10);
	writeByte(0x07, 0x3F); //PWR_MGMT2; ACCLEROMETER DISABLED, GYROSCOPE DISABLED
	osDelayUntil(10);
	writeByte(0x06, 0x01); //PWR_MGMT1; Auto select clock source
	osDelayUntil(10);
	writeByte(0x7F, 0x20); //REG_BANK_SEL; select user bank2
	osDelayUntil(10);
	writeByte(0x01, 0x2F); //GYRO_CONFIG_1; enable gyro DLPF, 2000dps, 11.6hz
	osDelayUntil(10);
	writeByte(0x00, 0x00); //GYRO_SMPLRT_DIV; gyro sample rate divider, ODR = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
	osDelayUntil(10);
	writeByte(0x7F, 0x00); //REG_BANK_SEL; select user bank0
	osDelayUntil(10);
	writeByte(0x07, 0x00); //0x38 PWR_MGMT2; ACCLEROMETER ENABLED, GYROSCOPE ENABLED
	osDelayUntil(10);
}
