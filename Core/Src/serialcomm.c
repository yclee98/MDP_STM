#include "serialcomm.h"

extern UART_HandleTypeDef huart3;

uint8_t aRxBuffer[4];

void SerialComm_Init(){
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer,4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Prevent unused arguments compilation warning
	UNUSED(huart);

	HAL_UART_Transmit(huart,(uint8_t *)aRxBuffer,sizeof(aRxBuffer),0xFFFF);
	OLED_ShowString(10,20,(uint8_t *)"rec");
	OLED_ShowString(60,20,aRxBuffer);
	OLED_Refresh_Gram();

	HAL_UART_Receive_IT(huart,(uint8_t *)aRxBuffer,4); //setup new receive interrupt
}

