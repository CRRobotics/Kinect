#ifndef _BEAGLESENDER_H
#define _BEAGLESENDER_H

#include <sys/types.h>

typedef uint16_t UINT16;
typedef int16_t INT16;

typedef struct {
	char magic[4];
	UINT16 distHigh;
	INT16 angleHigh;
	UINT16 distRight;
	INT16 angleRight;
	UINT16 distLeft;
	INT16 angleLeft;
	UINT16 distLow;
	INT16 angleLow;
} TrackingData;	

int openSocket();
void sendData(TrackingData* data, int sock);

#endif // _BEAGLESENDER_H
