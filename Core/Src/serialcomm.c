#include "serialcomm.h"
#include "pid.h"

extern UART_HandleTypeDef huart3;
extern uint8_t OLED_row0[20];

#define TxBUFFSIZE 35
#define RxBUFFSIZE 8

uint8_t aTxBuffer[TxBUFFSIZE];
uint8_t aRxBuffer[RxBUFFSIZE];

void SerialComm_Init(){
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer,RxBUFFSIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Prevent unused arguments compilation warning
	UNUSED(huart);

	HAL_UART_Transmit(huart,(uint8_t *)"turn left",9,0xFFFF);
	sprintf(OLED_row0, "rec %s", aRxBuffer);
	HAL_UART_Receive_IT(huart,(uint8_t *)aRxBuffer,RxBUFFSIZE); //setup new receive interrupt

	//for receiving pid from serial
//	int8_t counter =0;
//	float kp = 0;
//	while(*(aRxBuffer + counter) != 32 && counter < RxBUFFSIZE)
//	{
//		kp= *(aRxBuffer + counter) - 48 + kp * 10;
//		counter++;
//	}
//	counter++;
//	float ki = 0;
//	while(*(aRxBuffer + counter) != 32 && counter < RxBUFFSIZE)
//	{
//		ki= *(aRxBuffer + counter) - 48 + ki * 10;
//		counter++;
//	}
//	counter++;
//	float kd = 0;
//	while(*(aRxBuffer + counter) != 32 && counter < RxBUFFSIZE)
//	{
//		kd= (*(aRxBuffer + counter) - 48)/10.0 + kd / 10.0;
//		counter++;
//	}
//	sprintf(OLED_row0, "rec %d %d %d", (int)kp, (int)ki, (int)kd);
//	setPID(kp,ki,kd);
}

void printPIDdebug(int value1,int value2, int value3, int value4, long value5){
	HAL_UART_Transmit(&huart3,"\r\n",2,0xFFFF);
	sprintf(aTxBuffer, "%5d,%5d,%5d,%5d,%5d", value1,value2,value3,value4,value5);
	HAL_UART_Transmit(&huart3,aTxBuffer,TxBUFFSIZE,0xFFFF);
}


void printVelocity(int value1,int value2){
	HAL_UART_Transmit(&huart3,"\r\n",2,0xFFFF);
	sprintf(aTxBuffer, "%5d,%5d", value1,value2);
	HAL_UART_Transmit(&huart3,aTxBuffer,15,0xFFFF);
}

void printgyro(double value1, int value2){
	HAL_UART_Transmit(&huart3,"\r\n",2,0xFFFF);
	sprintf(aTxBuffer, "%5.2d, %5d", value1,value2);
	HAL_UART_Transmit(&huart3,aTxBuffer,7,0xFFFF);
}



