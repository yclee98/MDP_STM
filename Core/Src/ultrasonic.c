#include "ultrasonic.h"

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
extern double ultrasonicDistance;
extern TIM_HandleTypeDef htim3;
extern uint8_t OLED_row4[20];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (Is_First_Captured==0)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			Is_First_Captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			ultrasonicDistance = Difference * .034/2;
			Is_First_Captured = 0;

			sprintf(OLED_row4, "Ultra %d",(int)ultrasonicDistance);

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
		}
	}
}

//It is recommended to wait for at least 60ms before starting the operation again
void HCSR04_Read()
{
	ultrasonicDistance = -1;
	//int c = 10;

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	osDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
	osDelay(100);
}


