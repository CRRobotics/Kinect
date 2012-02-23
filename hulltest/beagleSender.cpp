#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
// Not sure how many of those are actually used, but including them to be safe

#include "beagleSender.h"

#define CRIO_ADDRESS "10.6.39.5"
#define CRIO_PORT 6639

// static struct sockaddr_in crio;

int openSocket()
{
	int sock;


	// Now create the socket we're sending from
	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (sock < 0)
	{
		printf("Failed to open socket.\n");	
	}

	return sock;
}

void sendData(TrackingData* data, int sock)
{
	struct sockaddr_in crio;
	memset(&crio, 0, sizeof(crio));

	// Initialize the crio address
	crio.sin_family = AF_INET;
	crio.sin_addr.s_addr = inet_addr(CRIO_ADDRESS);
	crio.sin_port = htons(CRIO_PORT);	

	printf("\n\nSending data...\n\n");

	// Make sure the header is correct
	data->magic[0] = '0';
	data->magic[1] = '6';
	data->magic[2] = '3';
	data->magic[3] = '9';

	// Byte-swap remaining fields
	data->distHigh = htons( data->distHigh );
	data->angleHigh = htons( data->angleHigh );
	data->distRight = htons( data->distRight );
	data->angleRight = htons( data->angleRight );
	data->distLeft = htons( data->distLeft );
	data->angleLeft = htons( data->angleLeft );
	data->distLow = htons( data->distLow );
	data->angleLow = htons( data->angleLow );

	int sent_bytes = sendto(sock, (char*)data, sizeof(TrackingData),
			0, (struct sockaddr *)&crio, sizeof(crio));
	if (sent_bytes != sizeof(TrackingData))
	{
		printf("Failed to transmit data to cRIO: sendto returned %d\n", sent_bytes);
		printf("errno is %s\n", strerror(errno));
	}
}

