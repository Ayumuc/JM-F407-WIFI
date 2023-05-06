#include <string.h>
#include <stdlib.h>

unsigned int strToIpv4(char* str)
{
	char delim[] = ".";
	char *p = NULL;
 	int count = 0;
	unsigned int result=0;
	for(p = strtok(str, delim); p != NULL; p = strtok(NULL, delim))
	{
		result += atoi(p);
		if( ++count < 4)
		result <<= 8;
	}
	return result;
}
