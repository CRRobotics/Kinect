#ifndef _AXIS_SENDER_H
#define _AXIS_SENDER_H

#include <cv.h>

// Starts the webserver and begins listening for requests
// Pass in a template image for what axisSetImg will receive
// Returns nonzero on error
int axisInit(IplImage* tmp);

// Brings down the server nicely
void axisStop();

// Post the new image to the server code
void axisSetImg(IplImage* img);

#endif // _AXIS_SENDER_H
