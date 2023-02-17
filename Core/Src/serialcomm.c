#include "serialcomm.h"
#include "pid.h"

extern UART_HandleTypeDef huart3;
extern uint8_t OLED_row5[20];

#define TxBUFFSIZE 35
#define RxBUFFSIZE 8
#define QUEUESIZE 16

uint8_t aTxBuffer[TxBUFFSIZE];
uint8_t aRxBuffer[RxBUFFSIZE];

extern TIM_HandleTypeDef htim1;

extern uint8_t waitingForCommand;
extern uint8_t direction; //forward=1 or backward=0
extern uint8_t movement; //turn left/right or straight
extern uint32_t magnitude;



uint8_t actionBuffer[QUEUESIZE][RxBUFFSIZE+1];
int frontCounter = 0;
int backCounter = 0;
int queueSize = 0;

int enqueue(){
	frontCounter = frontCounter %  QUEUESIZE;
	if(queueSize >= QUEUESIZE)
		return 0;

	for(int i=0; i<RxBUFFSIZE;i++){
		actionBuffer[frontCounter][i] = aRxBuffer[i];
	}

	actionBuffer[frontCounter][RxBUFFSIZE] = '\0';

	sprintf(OLED_row5, "en %s", actionBuffer[frontCounter]);

	if(strcmp(actionBuffer[frontCounter], "END00000\0") ==0){
		waitingForCommand = 0;
	}

	queueSize++;
	frontCounter++;
	return 1;
}

int dequeue(){
	backCounter = backCounter % QUEUESIZE;
	if(queueSize <= 0)
		return 0;

	sprintf(OLED_row5, "deq %s", actionBuffer[backCounter]);

	//check if end of action
	if(strcmp(actionBuffer[backCounter], "END00000\0") ==0){
		waitingForCommand = 1;
		queueSize--;
		backCounter++;
		sprintf(aTxBuffer, "STOP0000");
		HAL_UART_Transmit(&huart3,aTxBuffer,8,0xFFFF);
		return 0;
	}

	if(actionBuffer[backCounter][0] == 'F')
		direction = 1;
	else if(actionBuffer[backCounter][0] == 'B')
		direction = 0;
	else{
		queueSize--;
		backCounter++;
		return 0;
	}

	movement = actionBuffer[backCounter][1];
	magnitude = 0;
	for(int i=2; i<RxBUFFSIZE; i++){
		magnitude = (actionBuffer[backCounter][i] - 48) + magnitude *10;
	}

	queueSize--;
	backCounter++;
	return 1;
}

int isEmpty(){
	if(queueSize <= 0 || waitingForCommand == 1)
		return 1;
	else
		return 0;
}

void SerialComm_Init(){
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer,RxBUFFSIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Prevent unused arguments compilation warning
	UNUSED(huart);

	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,RxBUFFSIZE); //receive next set of command
	enqueue();
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



