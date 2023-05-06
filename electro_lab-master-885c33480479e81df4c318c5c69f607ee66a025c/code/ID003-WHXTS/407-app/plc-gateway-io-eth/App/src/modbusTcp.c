/*
 * modbusTcp.c
 *
 *  Created on: Sep 28, 2021
 *      Author: Administrator
 */


#include "MachineStatusManager.h"
#include "dataCollector.h"
#include "gpio.h"
#include "stdio.h"
#include "usart.h"
#include "rs485.h"

#define READ_DISCRETE_INPUTS       2
#define READ_HOLDING_REGISTERS     3
#define WRITE_SINGLE_COIL          5
#define WRITE_MULTIPLE_REGISTERS   16
#define KEY_START_ADDRESS 1
#define KEY_STOP_ADDRESS 3

enum KY{
	OFF = 0,
	ON,
};




extern float x_value, y_value;
extern double slave2_value_ch1;
extern double slave2_value_ch2;
extern double slave3_value_ch1;
extern double slave3_value_ch2;
extern double slave4_value_ch1;
extern double slave4_value_ch2;
extern double slave5_value_ch1;
extern double slave5_value_ch2;

extern double analogValues[18];
extern uint16_t speed_kilometer;

double  FloatData[10];
uint8_t Bytes[40];


static void floatToByte(float f,uint8_t *byte)
{
	uint8_t *pdata;
	pdata = (uint8_t*)&f;
	byte[1] = *pdata++;
	byte[0] = *pdata++;
	byte[3] = *pdata++;
	byte[2] = *pdata;
}

unsigned char* getFloatBytes(void)
{
	int8_t i;
	for(i=0; i<10; i++)
	{
		floatToByte(FloatData[i], &Bytes[i*4]);
	}
	return Bytes;
}

uint32_t writedata_to_flash(uint8_t *data, uint32_t len, uint32_t address)
{
	uint32_t i = 0, temp;

	HAL_FLASH_Unlock();
	for(i = 0; i < len; i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *data) == HAL_OK)
		{
			address = address + 1;
			data = data + 1;
		}
		else
		{
			return -1;
		}

	}

	HAL_FLASH_Lock();

	return address;
 }

//功能码0x03
// modbustcp地址：  0 -- A10 地址
//				  1 -- 苏州碰碰车开键位检测
//                2 -- A11 地址
//				  3 -- 苏州碰碰车关键位检测
//				  4 -- A20 地址
//				  6 -- A21 地址
//				  8 -- A30 地址
//				  10 -- A31 地址
//				  12 -- A40 地址
//				  14 -- A41 地址

//                16 -- 角度传感器X轴数据
//                18 -- 角度传感器y轴数据
//                20 -- addr = 2 的板子的ch1 数据
//                22 -- addr = 2 的板子的ch2 数据
//                24 -- addr = 3 的板子的ch1 数据
//                26 -- addr = 3 的板子的ch2 数据
//                28 -- addr = 4 的板子的ch1 数据
//                30 -- addr = 5 的板子的ch2 数据
//                32 -- addr = 5 的板子的ch1 数据
//                34 -- addr = 5 的板子的ch2 数据

uint8_t parseResult[200]={0};

uint8_t* parseModbusCommand(uint8_t *modbusCmd, int32_t comLength, uint8_t* size)//解析modbusTCP数据
{
	char     slaveId = 0;
	int32_t  Address = 0;
	int32_t  Length = 0;
	int32_t  commandlenght=0;
	int32_t  dataLength = 0;
	uint8_t* analogValueBytes;
	int32_t  i=0;

	*size = 0;

	if((modbusCmd[2]!=0)||(modbusCmd[3]!=0))
	{
		return 0;//protocol type
	}

	commandlenght = modbusCmd[4] << 8 | modbusCmd[5];
	if(commandlenght != (comLength-6))
	{
		return 0;
	}

	slaveId = modbusCmd[6];

	Address = modbusCmd[8] << 8 | modbusCmd[9];
	Length = modbusCmd[10] << 8 | modbusCmd[11];//在写线圈的时候，这代表高或者低
	if(Address < 0 || Length < 0)
	{
		return 0;
	}
	if(Address == KEY_START_ADDRESS){
		printf("start!\r\n");
	}
	else if(Address == KEY_STOP_ADDRESS){
		printf("stop!\r\n");
	}

	bzero(parseResult, sizeof(parseResult));

	parseResult[0] = modbusCmd[0];
	parseResult[1] = modbusCmd[1];
	parseResult[2] = 0;
	parseResult[3] = 0;
	parseResult[6] = slaveId;
	//printf("modbus cmd = %d\n", modbusCmd[7]);

	switch(modbusCmd[7])//function code
	{
		case WRITE_SINGLE_COIL:
			if(Address > 3)
			{
				return 0;
			}
			parseResult[4] = modbusCmd[4];
			parseResult[5] = modbusCmd[5];
			parseResult[7] = WRITE_SINGLE_COIL;
			parseResult[8] = modbusCmd[8];
			parseResult[9] = modbusCmd[9];
			parseResult[10] = modbusCmd[10];
			parseResult[11] = modbusCmd[11];
			*size = 12;
			if(parseResult[10] == 0xFF)
			{
				//writeDigitOutputStatus(Address,0x00);
			}
			else
			{
				//writeDigitOutputStatus(Address,0x01);
			}
			break;

		case READ_DISCRETE_INPUTS:
			if(Address + Length > 4)return 0;
			parseResult[8] = 1;
			dataLength = 3 + parseResult[8];
			parseResult[4] = dataLength >> 8;
			parseResult[5] = dataLength & 0xff;
			parseResult[7] = READ_DISCRETE_INPUTS;
			*size = dataLength + 6;
			parseResult[9] = 0;
			for(i=0; i<Length; i++)
			{
				parseResult[9] |= readDigitInputStatus(Address+i) << (i);
			}
			break;

		case READ_HOLDING_REGISTERS:
//			printf("address = %d\r\n", Address);


			if (Length > 38)
			{
				printf("modbus tcp read holding register length more than %d!\r\n", Length);
				Length = 38; //只能读取36个保持寄存器
			}

			analogValues[8] = x_value; //角度传感器x轴数据
			analogValues[9] = y_value; //角度传感器x轴数据
			analogValues[10] = slave2_value_ch1;       //addr = 1 的板子的ch1 数据
			analogValues[11] = slave2_value_ch2;       //addr = 1 的板子的ch2 数据
			analogValues[12] = slave3_value_ch1;       //addr = 2 的板子的ch1 数据
			analogValues[13] = slave3_value_ch2;       //addr = 2 的板子的ch2 数据
			analogValues[14] = slave4_value_ch1;       //addr = 3 的板子的ch1 数据
			analogValues[15] = slave4_value_ch2;       //addr = 3 的板子的ch2 数据
			analogValues[16] = slave5_value_ch1;       //addr = 4 的板子的ch1 数据
			analogValues[17] = slave5_value_ch2;       //addr = 4 的板子的ch2 数据

			parseResult[8] = Length*2;
			dataLength = 3 + parseResult[8];
			parseResult[4] = dataLength >> 8;
			parseResult[5] = dataLength & 0xff;
			parseResult[7] = READ_HOLDING_REGISTERS;
			*size = dataLength + 6;
			analogValueBytes = getAnalogValueBytes();

			for(i=0; i<36; i++)
			{
				parseResult[9+i*2]   = analogValueBytes[Address*2+ i*2];
				parseResult[9+i*2+1] = analogValueBytes[Address*2 + i*2+1];
			}
			parseResult[9+i*2] = *(__IO uint16_t*)(0x0805FFF0);
			i++;
			parseResult[9+i*2] = speed_kilometer / 256;
			parseResult[9+i*2+1] = speed_kilometer % 256;
			break;

		case WRITE_MULTIPLE_REGISTERS:
			parseResult[4] = 0x00;
			parseResult[5] = 0x06;
			parseResult[7] = WRITE_MULTIPLE_REGISTERS;
			parseResult[8] = modbusCmd[8];
			parseResult[9] = modbusCmd[9];
			parseResult[10] = modbusCmd[10];
			parseResult[11] = modbusCmd[11];
			*size = 12;
			printf("\n modbus recv\n");
			int k = 0;
			for(k = 0; k < commandlenght+6; k++){
				printf("%02X ", modbusCmd[k]);
			}

			for( k = 0; k < 12; k++){
				printf("%02X ", parseResult[k]);
			}
			if((modbusCmd[13] == 0x33 ) && (modbusCmd[14] == 0x44)){
				uint8_t ota_flag = 0x00;
				writedata_to_flash(&ota_flag, 1, 0x0805FFF0);
				ota_flag = *(__IO uint8_t*)(0x0805FFF0);
				printf("write ota_flag = %X\n", ota_flag);
				printf("\n modbus send\n");

				printf("\n reset device\n");
				HAL_NVIC_SystemReset();
			}

			break;

		default:
			return 0;
			break;
	}
//	printf("analogValueBytes:");
//		for(int i = 0 ; i < 84 ; i++){
//			printf("%02x ",analogValueBytes[i]);
//		}
//	printf("\r\n");
//	printf("parseResult:\r\n");
//	for(int i = 0 ; i < sizeof(parseResult) ; i++){
//		printf("%02x ",parseResult[i]);
//	}
//	printf("\r\n");

	return parseResult;
}




