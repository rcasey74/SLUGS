#ifndef _HIL_H_
#define _HIL_H_

#include "apDefinitions.h"
#include "protDecoder.h"
#include "circBuffer.h"

void hilRead (unsigned char* hilChunk);
void hil_getRawRead (short * rawData);
void hil_getGPSRead (unsigned char * gpsMsg);
void hil_getVned (float* vned);
void hil_getXYZ (float* xyz);
void hil_getEuler (float* euler);
void hil_getRates (float* pqr);
unsigned short hil_getTs (void);

#endif /* _HIL_H_ */
