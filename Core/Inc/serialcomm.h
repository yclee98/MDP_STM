
#ifndef INC_SERIALCOMM_H_
#define INC_SERIALCOMM_H_

#include "main.h"
//#include "oled.h"

void SerialComm_Init();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_SERIALCOMM_H_ */
