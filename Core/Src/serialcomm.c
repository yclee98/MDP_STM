#include "serialcomm.h"
#include "pid.h"

extern UART_HandleTypeDef huart3;
extern uint8_t OLED_row5[20];

#define TxBUFFSIZE 35
#define RxBUFFSIZE 8
#define QUEUESIZE 30

uint8_t aTxBuffer[TxBUFFSIZE];
uint8_t aRxBuffer[RxBUFFSIZE];

extern TIM_HandleTypeDef htim1;

extern uint8_t waitingForCommand;
extern uint8_t direction; //forward=1 or backward=0
extern uint8_t movement; //turn left/right or straight
extern uint32_t magnitude;
extern int numOfEnd;

uint8_t actionBuffer[QUEUESIZE][RxBUFFSIZE+1];
int frontCounter = 0;
int backCounter = 0;
int queueSize = 0;

void startQueue(){
	char * actionsList[] ={
			"TPR00000","END00000"
	};

	for(int i=0; i<sizeof(actionsList)/sizeof(char*); i++){
		for(int j=0; j<8;j++){
			aRxBuffer[j] = actionsList[i][j];
		}
		enqueue();
	}
}
//BS020000FL090000FS008000FR180000BS030000END00000
int enqueue(){
	frontCounter = frontCounter %  QUEUESIZE;
	if(queueSize >= QUEUESIZE)
		return 0;

	for(int i=0; i<RxBUFFSIZE;i++){
		actionBuffer[frontCounter][i] = aRxBuffer[i];
	}

	actionBuffer[frontCounter][RxBUFFSIZE] = '\0';

	sprintf(OLED_row5, "en %s", actionBuffer[frontCounter]);
	//when receive END00000 need to reply with STOP0000
	//when receive END00001, no need to reply with STOP0000
	if(strcmp(actionBuffer[frontCounter], "END00000\0") ==0 || strcmp(actionBuffer[frontCounter], "END00001\0") ==0){
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

	sprintf(OLED_row5, "deq %d %s", queueSize, actionBuffer[backCounter]);

	if(strcmp(actionBuffer[backCounter], "START000\0") ==0){
		queueSize--;
		backCounter++;
		movement = 'Q';
		return 1;
	}
	else if(strcmp(actionBuffer[backCounter], "P1L00000\0") ==0){
		queueSize--;
		backCounter++;
		movement = 'W';
		return 1;
	}
	else if(strcmp(actionBuffer[backCounter], "P1R00000\0") ==0){
		queueSize--;
		backCounter++;
		movement = 'E';
		return 1;
	}

	else if(strcmp(actionBuffer[backCounter], "P2L00000\0") ==0){
		queueSize--;
		backCounter++;
		movement = 'R';
		return 1;
	}
	else if(strcmp(actionBuffer[backCounter], "P2R00000\0") ==0){
		queueSize--;
		backCounter++;
		movement = 'T';
		return 1;
	}
	//check if end of action
	else if(strcmp(actionBuffer[backCounter], "END00000\0") ==0){
		numOfEnd += 1;
		waitingForCommand = 1;
		queueSize--;
		backCounter++;
		sprintf(aTxBuffer, "STOP0000");
		HAL_UART_Transmit(&huart3,aTxBuffer,8,0xFFFF);
		return 0;
	}
	else if(strcmp(actionBuffer[backCounter], "FALSE0000\0") ==0){
			queueSize--;
			backCounter++;
			movement = 'A';
			return 1;
		}
	else if(strcmp(actionBuffer[backCounter], "END00001\0") ==0){
		waitingForCommand = 1;
		queueSize--;
		backCounter++;
		return 0;
	}

	if(actionBuffer[backCounter][0] == 'F')
		direction = 1;
	else if(actionBuffer[backCounter][0] == 'B')
		direction = 0;
	else if(actionBuffer[backCounter][0] == 'S') //SENSORXX
	{
		direction = 1;
		magnitude = 0;
		movement = 'D';
		for(int i=6; i<RxBUFFSIZE; i++){
			magnitude = (actionBuffer[backCounter][i] - 48) + magnitude *10;
		}
		queueSize--;
		backCounter++;
		return 1;
	}
	else{
		queueSize--;
		backCounter++;
		return 0;
	}

	movement = actionBuffer[backCounter][1];
	magnitude = 0;
	if(movement != 'A'){ //need find magnitude if it is not A; A is for the tilt
		for(int i=2; i<RxBUFFSIZE; i++){
			magnitude = (actionBuffer[backCounter][i] - 48) + magnitude *10;
		}
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
	enqueue();
	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,RxBUFFSIZE); //receive next set of command

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
	//setPID(kp,ki,kd);
}



