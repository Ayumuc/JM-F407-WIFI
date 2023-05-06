/*
 * dataCollector.h
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */

#ifndef INC_DATACOLLECTOR_H_
#define INC_DATACOLLECTOR_H_

#include "MachineStatusManager.h"


typedef struct
{
	long long time;
	MACHINE* pstMachine;
	double value1, value2; //采集值，顺便用于保存最大值
	double minvalue1, minvalue2;
	char value1Overrange;
	char value2Overrange;
	char type;
}COLLECTION_DATA;

double *getAnalogValues(void);
unsigned char* getAnalogValueBytes(void);
void initDataCollector(void);
void collectDataHandler(void);




#endif /* INC_DATACOLLECTOR_H_ */
