
#ifndef INC_SERIALCOMM_H_
#define INC_SERIALCOMM_H_

#include "main.h"

int enqueue();
int dequeue();
int isEmpty();

void startQueue();
void SerialComm_Init();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void printPIDdebug(int value1,int value2, int value3, int value4, long value5);
void printVelocity(int value1,int value2);
void printgyro(double value1, int value2);

#endif /* INC_SERIALCOMM_H_ */
