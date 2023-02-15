#include "serialcomm.h"
#include "pid.h"

extern UART_HandleTypeDef huart3;
extern uint8_t OLED_row5[20];

#define TxBUFFSIZE 35
#define RxBUFFSIZE 8

uint8_t aTxBuffer[TxBUFFSIZE];
uint8_t aRxBuffer[RxBUFFSIZE];

extern TIM_HandleTypeDef htim1;

extern uint8_t waitingForCommand;
extern uint8_t direction; //forward=1 or backward=0
extern uint8_t movement; //turn left/right or straight
extern uint32_t magnitude;

void SerialComm_Init(){
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer,RxBUFFSIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Prevent unused arguments compilation warning
	UNUSED(huart);
	sprintf(OLED_row5, "act %s", aRxBuffer);

	receiveCommand();
}

void commandCompleted(){
	waitingForCommand = 1;
	sprintf(aTxBuffer, "reached");
	sprintf(OLED_row5, "act %s", aTxBuffer);
	HAL_UART_Transmit(&huart3,aTxBuffer,8,0xFFFF);
	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,RxBUFFSIZE); //receive next set of command
}

void receiveCommand(){
	if(aRxBuffer[0] == 'f')
		direction = 1;
	else if(aRxBuffer[0] == 'b')
		direction = 0;
	else
		return;

	movement = aRxBuffer[1];
	magnitude = 0;
	for(int i=2; i<RxBUFFSIZE; i++){
		magnitude = (aRxBuffer[i] - 48) + magnitude *10;
	}
	waitingForCommand = 0;
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

void recievePID(){
	//for receiving pid from serial
	int8_t counter =0;
	float kp = 0;
	while(*(aRxBuffer + counter) != 32 && counter < RxBUFFSIZE)
	{
		kp= *(aRxBuffer + counter) - 48 + kp * 10;
		counter++;
	}
	counter++;
	float ki = 0;
	while(*(aRxBuffer + counter) != 32 && counter < RxBUFFSIZE)
	{
		ki= *(aRxBuffer + counter) - 48 + ki * 10;
		counter++;
	}
	counter++;
	float kd = 0;
	while(*(aRxBuffer + counter) != 32 && counter < RxBUFFSIZE)
	{
		kd= (*(aRxBuffer + counter) - 48)/10.0 + kd / 10.0;
		counter++;
	}
	sprintf(OLED_row5, "rec %d %d %d", (int)kp, (int)ki, (int)kd);
	setPID(kp,ki,kd);
}



