
#include "main.h"
#include "MachineStatusManager.h"
#include "DigitIO.h"



unsigned char readDigitInputStatus(unsigned char ch)
{
	switch(ch)
	{
		case 0:
			return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
			break;
		
		case 1:
			return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
			break;
		
		case 2:
			return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
			break;
		
		case 3:
			return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
			break;
		
		default:
		break;
	}
	return 0;
}

unsigned char readDigitOutputStatus(unsigned char ch)
{
	switch(ch)
	{
		case 0:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0);
			break;
		
		case 1:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1);
			break;
		
		case 2:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3);
			break;
		
		case 3:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4);
			break;
		
		default:
		break;
	}
	return 0;
}

void writeDigitOutputStatus(unsigned char ch,unsigned char status)
{
	switch(ch)
	{
		case 0:
			if(status == 1)
				DIGIT_OUT4_HIGH();
			else
				DIGIT_OUT4_LOW();
			break;
		case 1:
			if(status == 1)
				DIGIT_OUT3_HIGH();
			else
				DIGIT_OUT3_LOW();
			break;
		case 2:
			if(status == 1)
				DIGIT_OUT2_HIGH();
			else
				DIGIT_OUT2_LOW();
			break;
		case 3:
			if(status == 1)
				DIGIT_OUT1_HIGH();
			else
				DIGIT_OUT1_LOW();
			break;
		
		default:
			break;
	}
}

