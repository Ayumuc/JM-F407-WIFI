/*
 * md5.h
 *
 *  Created on: 2023年2月8日
 *      Author: sfs
 */

#ifndef INC_MD5_H_
#define INC_MD5_H_

typedef struct
{
	unsigned int count[2];
	unsigned int state[4];
	unsigned char buffer[64];
}MD5_CTX;

void MD5Init(MD5_CTX *context);
void MD5Update(MD5_CTX *context,unsigned char *input,unsigned int inputlen);
void MD5Final(MD5_CTX *context,unsigned char digest[16]);

#endif /* INC_MD5_H_ */
