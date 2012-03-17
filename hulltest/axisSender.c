#include "axisSender.h"

// mongoose is our embedded server
#include "mongoose.h"

// Need highgui functions to encode the mjpeg stream
#include <highgui.h>

#include <pthread.h>

struct mg_context* ctx;

pthread_mutex_t mutex_dbgimg = PTHREAD_MUTEX_INITIALIZER;
IplImage* dbgimg_mid = 0;
IplImage* dbgimg_out = 0;

int haveImg = 0;

// Options for the http server:
// This code can only handle 1 request at a time.
// With 1 thread, simultaneous requests will not occur
static const char *options[] = {
	"document_root", "html",
	"listening_ports", "80",
	"num_threads", "1",
	NULL
};

void axis_respond(struct mg_connection* conn,
		  const struct mg_request_info* request_info) {
	IplImage* tmp;
	CvMat* encImg;
	char* data;
	CvSize imgSize;
	int dataSize;

	if (haveImg == 0) {
		// We don't have any processed images yet!
		mg_printf(conn, "HTTP/1.1 503 Service Unavailable\r\n\r\n");
		return;
	}

	// Grab the current image
	pthread_mutex_lock(&mutex_dbgimg);
	
	tmp = dbgimg_out;
	dbgimg_out = dbgimg_mid;
	dbgimg_mid = tmp;

	pthread_mutex_unlock(&mutex_dbgimg);

	// Encode the image
	// args are type, image, quality (0-100, 100 is best, 95 is default)
	encImg = cvEncodeImage(".jpg", dbgimg_out, 70);

	// Extract size from OpenCV's structure
	imgSize = cvGetSize(encImg);
	dataSize = imgSize.width * imgSize.height;

	// Pull out the actual data
	cvGetRawData(encImg, &data);

	// Send the response headers
	mg_printf(conn, "HTTP/1.1 200 OK\r\n
			Content-Type: video/x-motion-jpeg\r\n
			Content-Length: %u\r\n\r\n", dataSize);

	// Send the actual data
	mg_write(conn, data, dataSize);

	// Clean up
	// Uncomment this if there appears to be a leak
	// The cv sources suggest that it isn't needed
	// cvReleaseData(encImg);
	cvReleaseMat(&encImg);
}

static void* axis_handler(enum mg_event event,
			  struct mg_connection* conn,
			  const struct mg_request_info* request_info) {
	// Doing it this way because of example code
	void *processed = (void*) "yes";

	if (event == MG_NEW_REQUEST) {
		if (strcmp(request_info->uri, "/mjpg/video.mjpg") == 0) {
			axis_respond(conn, request_info);
		} else {
			processed = NULL;
		}
	} else {
		processed = NULL;
	}

	return processed;
}

void axisSetImg(IplImage* img) {
	// Make sure we have been initialized
	if (ctx != NULL) {
		haveImg = 1;
	
		pthread_mutex_lock(&mutex_dbgimg);
		cvCopy(img, dbgimg_mid);
		pthread_mutex_unlock(&mutex_dbgimg);
	}
}

int axisInit(IplImage* tmp) {
	// Create our buffer images
	dbgimg_mid = cvCloneImage(tmp); 
	dbgimg_out = cvCloneImage(tmp);

	// Launch the http server
	ctx = mg_start(&axis_handler, NULL, options);
	if (ctx == NULL) return -1;

	return 0;
}

void axisStop() {
	// Stop any new requests, block until old ones are finished
	mg_stop(ctx);
	ctx = NULL;

	// Clean up
	haveImg = 0;
	cvReleaseImage(&dbgimg_mid);
	cvReleaseImage(&dbgimg_out);
}
