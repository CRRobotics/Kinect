#ifndef _BEAGLESENDER_H
#define _BEAGLESENDER_H

typedef struct {
	char magic[4];
	UINT16 distHigh;
	UINT16 angleHigh;
	UINT16 distRight;
	UINT16 angleRight;
	UINT16 distLeft;
	UINT16 angleLeft;
	UINT16 distLow;
	UINT16 angleLow;
} TrackingData	

int openSocket();
void sendData(TrackingData* data, int sock);

#endif // _BEAGLESENDER_H
