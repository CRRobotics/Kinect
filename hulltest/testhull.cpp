//#define DEBUG_MAIN
#define DEBUG_SORT
//#define DEBUG_POLY

/*
testhull.cpp

Written by the members of 
FIRST Robotics Team #639 - Code Red Robotics
For the 2012 FIRST Robotics Competition.

Based on:
freenectopencv.cpp

    Copyright (C) 2010  Arne Bernin <arne@alamut.de>

    This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
	see:
	http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
	http://www.gnu.org/licenses/gpl-3.0.txt
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>

#include <libfreenect/libfreenect.h>
#include <pthread.h>

#define CV_NO_BACKWARD_COMPATIBILITY

#include <cv.h>
#include <highgui.h>

// Display images?
bool display;

#include "PolyProc.h"
#include "RobotMath.h"
#include "beagleSender.h"

#define FREENECTOPENCV_WINDOW_N "Normalimage"
#define FREENECTOPENCV_RGB_DEPTH 3 // Currently used for tempImg.
#define FREENECTOPENCV_IR_DEPTH 1 // Current display depth.
#define FREENECTOPENCV_RGB_WIDTH 640
#define FREENECTOPENCV_RGB_HEIGHT 480


IplImage* rgbimg = 0;
IplImage* tempimg = 0;
pthread_mutex_t mutex_rgb = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;

// Network socket
int CRRsocket;

// callback for rgbimage, called by libfreenect
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock( &mutex_rgb );
	memcpy(rgbimg->imageData, rgb, FREENECT_VIDEO_RGB_SIZE);
	pthread_mutex_unlock( &mutex_rgb );
}


/*
 * Main thread for Kinect input, vision processing, and network send - everything, really.
 */
void *cv_threadfunc (void *ptr) {
	// Images for openCV
	IplImage* timg = cvCloneImage(rgbimg); // Image we do our processing on
	IplImage* dimg = cvCloneImage(timg); // Image we draw on
	CvSize sz = cvSize( timg->width & -2, timg->height & -2);
	IplImage* outimg = cvCreateImage(sz, 8, 3);

	// Mem. mgmt. Remember to clear each time we run loop.
	CvMemStorage* storage = cvCreateMemStorage(0);

	// Set region of interest
	cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));
	if (display) { cvSetImageROI(dimg, cvRect(0, 0, sz.width, sz.height)); }

	// Open network socket.
	CRRsocket = openSocket();
	if (CRRsocket < 0) pthread_exit(NULL);

	/*
	 * MAIN LOOP
	 */
	while (1) 
	{ 
		// Sequence to run ApproxPoly on
		CvSeq* polyseq = cvCreateSeq( CV_SEQ_KIND_CURVE | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
		// Raw contours list
		CvSeq* contours; 
		// Current convex hull
		CvSeq* hull;
		// # of points in hull
		int hullcount; 


		/* PULL RAW IMAGE FROM KINECT */
		pthread_mutex_lock( &mutex_rgb );
		if (display) { cvCopy(rgbimg, dimg, 0); }
		cvCopy(rgbimg, timg, 0);
		pthread_mutex_unlock( &mutex_rgb );

		/* DILATE */
		IplConvKernel* element = cvCreateStructuringElementEx(3, 3, 1, 1, 0);
		IplConvKernel* element2 = cvCreateStructuringElementEx(5, 5, 2, 2, 0);
		cvDilate(timg, timg, element2, 1);
		cvErode(timg, timg, element, 1);

		/* THRESHOLD*/
		cvThreshold(timg, timg, 100, 255, CV_THRESH_BINARY);

		/* OUTPUT PROCESSED OR RAW IMAGE */
		if (display) { cvCvtColor(dimg, outimg, CV_GRAY2BGR); }

		/* CONTOUR FINDING */
		cvFindContours(timg, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
 
		/* CONVEX HULL + POLYGON APPROXIMATION + CONVERT TO RECTANGLE + FILTER FOR INVALID RECTANGLES */ 
		// Store points to draw line between
		CvPoint* draw1;
		CvPoint* draw2; 
		vector<PolyVertices> rectangleList;

		while (contours) // Run for all polygons
		{
			// Raw rectangle
			PolyVertices fullrect;

			// Filter noise
			if (fabs(cvContourArea(contours, CV_WHOLE_SEQ)) > 600)
			{
				// Get convex hull
				hull = cvConvexHull2( contours, storage, CV_CLOCKWISE, 1 );
				hullcount = hull->total;

				// Draw hull (red line)
				if (display) {
					draw1 = (CvPoint*)cvGetSeqElem(hull, hullcount - 1);
					for (int i = 0; i < hullcount; i++)
					{
						draw2 = (CvPoint*)cvGetSeqElem( hull, i );
						cvLine( outimg, *draw1, *draw2, CV_RGB(255,0,0), 1, 8, 0 );
						draw1 = draw2;
					}
				}

				// Convert polys from convex hull to rectangles
				polyToQuad(hull, &fullrect, outimg, display);

				// Filter for bad rectangles
				if(!(fullrect.points[0] == NULL || fullrect.points[1] == NULL || 
					fullrect.points[2] == NULL || fullrect.points[3] == NULL)
					&& !fullrect.isMalformed())
				{
					/* FILL rectangleList */
					rectangleList.push_back(fullrect);

					#ifdef DEBUG_MAIN
					printf("RESULT: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",
						fullrect.points[0]->x, fullrect.points[0]->y, 
						fullrect.points[1]->x, fullrect.points[1]->y,
						fullrect.points[2]->x, fullrect.points[2]->y, 
						fullrect.points[3]->x, fullrect.points[3]->y);
					fflush(stdout);
					#endif
				}

			}
			cvClearSeq(polyseq);
			contours = contours->h_next;
		}

		/* FILTER OVERLAPPING RECTANGLES */
		FilterInnerRects(rectangleList);

		/* CHECK DEGREE OF FILL TO FILTER LIGHTS, ETC */
		FilterBrightRects(rectangleList);

		/* SORT INTO CORRECT BUCKET */
		SortRects(rectangleList);

		/* DRAW & PROCESS MATH; FILL SEND STRUCT */
		// TODO: Make the math stuff static
		RobotMath robot;
		TrackingData outgoing;
		memset(&outgoing, 0, sizeof(TrackingData));

		// Fill packets
		// Packet fields are unsigned 16bit integers, so we need to scale them up
		// Currently both dist and angle scaled 100x (hundredths precision)
		// NOTE:
		// Currently correct results are only calculated by using bottom basket and constant for top.
		if (rectangleList[0].isValid())
		{
			outgoing.distHigh = 100 * robot.GetDistance(*(rectangleList[0].points[2]), *(rectangleList[0].points[3]), 0);
			outgoing.angleHigh = 100 * robot.GetAngle(*(rectangleList[0].points[2]), *(rectangleList[0].points[3]));
		}
		/* LEFT AND RIGHT SWAPPED TEMPORARILY, FIND REAL PROBLEM LATER */
		if (rectangleList[1].isValid())
		{
			outgoing.distLeft = 100 * robot.GetDistance(*(rectangleList[1].points[2]), *(rectangleList[1].points[3]), 1);
			outgoing.angleLeft = 100 * robot.GetAngle(*(rectangleList[1].points[2]), *(rectangleList[1].points[3]));
		}
		if (rectangleList[2].isValid())
		{
			outgoing.distRight = 100 * robot.GetDistance(*(rectangleList[2].points[2]), *(rectangleList[2].points[3]), 2);
			outgoing.angleRight = 100 * robot.GetAngle(*(rectangleList[2].points[2]), *(rectangleList[2].points[3]));
		}
		if (rectangleList[3].isValid())
		{
			outgoing.distLow = 100 * robot.GetDistance(*(rectangleList[3].points[2]), *(rectangleList[3].points[3]), 3);
			outgoing.angleLow = 100 * robot.GetAngle(*(rectangleList[3].points[2]), *(rectangleList[3].points[3]));
		}

		// Draw filtered rects (thick blue line)
		if (display) {
			for (int i = 0; i < 4; i++)
			{
				if (outimg && rectangleList[i].isValid())
				{
					cvLine( outimg, *(rectangleList[i].points[3]), *(rectangleList[i].points[2]), CV_RGB(0,0,255), 2, 8, 0 );
					cvLine( outimg, *(rectangleList[i].points[2]), *(rectangleList[i].points[0]), CV_RGB(0,0,255), 2, 8, 0 );
					cvLine( outimg, *(rectangleList[i].points[0]), *(rectangleList[i].points[1]), CV_RGB(0,0,255), 2, 8, 0 );
					cvLine( outimg, *(rectangleList[i].points[1]), *(rectangleList[i].points[3]), CV_RGB(0,0,255), 2, 8, 0 );
				}
			}
		}

		#ifdef DEBUG_MAIN
		printf("Top distance: %d\n", outgoing.distHigh);
		printf("Top angle: %d\n\n", outgoing.angleHigh);
		#endif

		if (display) {
		CvPoint cent1 = cvPoint(320, 0);
		CvPoint cent2 = cvPoint(320, 480);
		CvPoint cent3 = cvPoint(0, 240);
		CvPoint cent4 = cvPoint(640, 240);
			cvLine( outimg, cent1, cent2, CV_RGB(0,255,0), 1, 8, 0 );
			cvLine( outimg, cent3, cent4, CV_RGB(0,255,0), 1, 8, 0 ); 
		}

		/* SEND TO CRIO */
		sendData(&outgoing, CRRsocket);

		if( cvWaitKey( 15 )==27 )
		{
			// Empty for now.
		}

		/* DISPLAY */
		if (display) { cvShowImage (FREENECTOPENCV_WINDOW_N,outimg); }
		
		/* CLEANUP */
		cvClearMemStorage(storage);
	}
	pthread_exit(NULL);
}


/*
 * Main method; primarily just initializes our thread and handles Kinect details.
 */
int main(int argc, char **argv)
{
	freenect_context *f_ctx;
	freenect_device *f_dev;

	display = 0;

	int res = 0;
	int die = 0;
	printf("Code Red Kinect Vision init\n");

	if (argc > 1 && strcmp(argv[1],"--display") == 0) {
		display = 1;
	}
			
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
		printf("Could not open device\n");
		return 1;
	}

	freenect_set_led(f_dev, LED_RED);

	if (display) { cvNamedWindow( FREENECTOPENCV_WINDOW_N, CV_WINDOW_AUTOSIZE ); }
	rgbimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_IR_DEPTH);
	tempimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);

	// create opencv display thread
	res = pthread_create(&cv_thread, NULL, cv_threadfunc, (void*) tempimg);
	if (res) {
		printf("pthread_create failed\n");
		return 1;
	}

	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_format(f_dev, FREENECT_VIDEO_IR_8BIT);

	printf("init done\n");

	freenect_start_video(f_dev);

	while(!die && freenect_process_events(f_ctx) >= 0 );
}
