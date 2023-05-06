/*
 * MachineStatusManager.h
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */


#ifndef MACHINE_STATUS_MANAGER_H
#define MACHINE_STATUS_MANAGER_H


#include "boardConfig.h"



typedef struct
{
	double scale;
	int minAdcValue;
	int maxAdcValue;
}CHANNEL_PARAMETER;

typedef struct
{
	CHANNEL_PARAMETER channelParameter[2];
}MACHINE;

void initMachineStatus(DEVICE_CONFIG* config);

#endif
