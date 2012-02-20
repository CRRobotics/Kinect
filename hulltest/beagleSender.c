#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
// Not sure how many of those are actually used, but including them to be safe

#include "beagleSender.h"

#define CRIO_ADDRESS "10.6.39.2"
#define CRIO_PORT 6639

static struct sockaddr_in crio;

int openSocket()
{
	int sock;

	// Initialize the crio address
	crio.sin_family = AF_INET
	crio.sin_addr.s_addr = inet_addr(CRIO_ADDRESS);
	crio.sin_port = htons(CRIO_PORT);	

	// Now create the socket we're sending from
	sock = socket(PF_INET, SOCK_DGRAM, 0);

	return sock;
}

void sendData(TrackingData* data, int sock)
{
	// Make sure the header is correct
	data->magic[0] = '0';
	data->magic[1] = '6';
	data->magic[2] = '3';
	data->magic[3] = '9';

	int sent_bytes = sendto(sock, (char*)data, sizeof(TrackingData),
			0, (struct sockaddr *)&crio, sizeof(struct sockaddr_in));
	if (sent_bytes != sizeof(TrackingData))
	{
		printf("Failed to transmit data to cRIO: sendto returned %d\n", sent_bytes);
	}
}

