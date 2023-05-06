#ifndef DIGIT_IO_H
#define DIGIT_IO_H 


#define DIGIT_OUT1_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET )
#define DIGIT_OUT1_LOW() 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET)
#define DIGIT_OUT2_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET)
#define DIGIT_OUT2_LOW() 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET)
#define DIGIT_OUT3_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET)
#define DIGIT_OUT3_LOW() 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET)
#define DIGIT_OUT4_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)
#define DIGIT_OUT4_LOW() 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

#define SetMachineLED				DIGIT_OUT4_LOW();
#define ClearMachineLED 			DIGIT_OUT4_HIGH();
#define SetProductLED				DIGIT_OUT3_LOW();
#define ClearProductLED				DIGIT_OUT3_HIGH();
#define SetDataAbnormalLED		    DIGIT_OUT2_LOW();
#define ClearDataAbnormalLED	    DIGIT_OUT2_HIGH();
#define SetPowerLED					DIGIT_OUT1_LOW();

unsigned char readDigitInputStatus(unsigned char ch);
unsigned char readDigitOutputStatus(unsigned char ch);
void writeDigitOutputStatus(unsigned char ch,unsigned char status);

#endif
