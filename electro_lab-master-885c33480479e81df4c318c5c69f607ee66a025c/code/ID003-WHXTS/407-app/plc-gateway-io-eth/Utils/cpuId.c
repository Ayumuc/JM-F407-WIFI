#include "main.h"

unsigned int cpuId[3];

unsigned int* readCpuId(void)
{
	cpuId[0] = *(__IO uint32_t*)(0x1fff7a10);
	cpuId[1] = *(__IO uint32_t *)(0x1fff7a14);
	cpuId[2] = *(__IO uint32_t *)(0x1fff7a18);
	
	return cpuId;
}
