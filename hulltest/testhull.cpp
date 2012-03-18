//#define DEBUG_MAIN
//#define DEBUG_SORT
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

#include "RobotMath.h"
#include "beagleSender.h"
#include "axisSender.h"


#define FREENECTOPENCV_WINDOW_N "Normalimage"
#define FREENECTOPENCV_RGB_DEPTH 3 // Currently used for tempImg.
#define FREENECTOPENCV_IR_DEPTH 1 // Current display depth.
#define FREENECTOPENCV_RGB_WIDTH 640
#define FREENECTOPENCV_RGB_HEIGHT 480

class PolyVertices
{
public:
	CvPoint* points[4];
	CvPoint center; 
	int dist;

	PolyVertices();
	void invalidate();
	bool isValid();
	bool isMalformed();
};

PolyVertices::PolyVertices()
{
	points[0] = NULL;
	points[1] = NULL;
	points[2] = NULL;
	points[3] = NULL;
	center.x = 0;
	center.y = 0;
	dist = 0;
}

void PolyVertices::invalidate()	{ center.x = center.y = 0; }

bool PolyVertices::isValid() { return (center.x != 0 && center.y != 0); }

bool PolyVertices::isMalformed()
{
	return false;
}

// Compare centers
bool operator== (const PolyVertices& arg1, const PolyVertices& arg2) { return (arg1.center.x == arg2.center.x && arg1.center.y == arg2.center.y); }


IplImage* rgbimg = 0;
IplImage* tempimg = 0;
pthread_mutex_t mutex_rgb = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;

// Network socket
int CRRsocket;

// Display images?
bool display;
bool draw;
bool axis;


/*
 * Converts n-gons to 4-gons.
 * qKey order is, CCW from top right, 3->2->0->1.
 */
void polyToQuad(CvSeq* sequence, PolyVertices *poly, IplImage* img )
{
	// Sequence is the hull.
	// poly->points is the array that stores the points of the rectangle.
	// img is the image we are drawing lines and such on. Not strictly necessary.
	#ifdef DEBUG_POLY
	printf("Sequence: %d\n", sequence->total);
	#endif

	/*CALCULATE CENTER*/
	int extremes[4]; //minX, maxX, minY, maxY
	extremes[0] = ((CvPoint*)cvGetSeqElem(sequence, 0))->x;
	extremes[1] = extremes[0];
	extremes[2] = ((CvPoint*)cvGetSeqElem(sequence, 0))->y;
	extremes[3] = extremes[2];


	CvSeqReader seqReader;
	cvStartReadSeq(sequence, &seqReader, 0);
	for(int i = 0; i < sequence->total; i++)
	{
		CvPoint* point = (CvPoint*) seqReader.ptr;

		if(point->x < extremes[0]) extremes[0] = point->x;
		if(point->x > extremes[1]) extremes[1] = point->x;

		if(point->y < extremes[2]) extremes[2] = point->y;
		if(point->y > extremes[3]) extremes[3] = point->y;
		CV_NEXT_SEQ_ELEM(seqReader.seq->elem_size, seqReader);
	}
	poly->center.x = (extremes[0] + extremes[1])/2;
	poly->center.y = (extremes[2] + extremes[3])/2;

	if (display) {
		cvCircle( img, poly->center, 2, CV_RGB(255,0,255), -1, 8, 0 );
	}

	#ifdef DEBUG_POLY
	printf("Calculated Center (%d): %i, %i\n", sequence->total, poly->center.x, poly->center.y);
	#endif

	/*CALCULATE DISTANCES FROM CENTER AND CALCULATE MAX DIST IN EACH QUADRANT*/
	double distances[4] = {0, 0, 0, 0};
	double distance; //current point's distance

	int qKey = 0;

	CvSeqReader seqReader2;
	cvStartReadSeq(sequence, &seqReader2, 0);
	for(int i = 0; i < sequence->total; i++)
	{
		CvPoint* point = (CvPoint*) seqReader2.ptr;
		distance = pow((double)point->x - poly->center.x, 2) + pow((double)point->y - poly->center.y, 2);
		qKey = ((point->x > poly->center.x ? 1 : 0) + (point->y < poly->center.y ? 2 : 0));

		/* FILL POINTS */
		if(distance > distances[qKey])
		{
			poly->points[qKey] = point;
			distances[qKey] = distance;
		}
		CV_NEXT_SEQ_ELEM(seqReader2.seq->elem_size, seqReader2);
	}

	if(!(poly->points[0] == NULL || poly->points[1] == NULL || poly->points[2] == NULL || poly->points[3] == NULL))
	{
		/* FILL poly->dist */
		poly->dist = pow((double)poly->points[0]->x - poly->center.x, 2) + pow((double)poly->points[0]->y - poly->center.y, 2);

		#ifdef DEBUG_POLY
		printf("Dist to 0: %d\n", poly->dist);
		#endif

		// Draw vertices (purple circles)
		if (display) {
			cvCircle( img, *poly->points[0], 3, CV_RGB(255,0,255), -1, 8, 0 );
			cvCircle( img, *poly->points[1], 3, CV_RGB(255,0,255), -1, 8, 0 );
			cvCircle( img, *poly->points[2], 3, CV_RGB(255,0,255), -1, 8, 0 );
			cvCircle( img, *poly->points[3], 3, CV_RGB(255,0,255), -1, 8, 0 );
		}
	}
}

// Return whether we want to delete poly. Subroutine of FilterInnerRects.
bool FilterSub(vector<PolyVertices> &list, PolyVertices poly)
{
	for (int i = 0; i < list.size(); i++)
	{
		double result = pow((double)list[i].center.x - (double)poly.center.x, 2) + pow((double)list[i].center.y - (double)poly.center.y, 2);
		if ((result < 50 * 50) && (list[i].dist != poly.dist))
			if (list[i].dist > poly.dist)
				return true;
	}
	return false;
}

// Remove rectangles enclosed by others.
void FilterInnerRects(vector<PolyVertices> &list)
{
	vector<PolyVertices>::iterator p = list.begin();
	while (p < list.end()) 
	{
		if (FilterSub(list, *p))
			p = list.erase(p);
		else
			++p;
	}
	/* Version without goto:
	vector<PolyVertices>::iterator p = list.begin();
	while (p < list.end()) {
		if (FilterSub(list, *p))
		{
			fflush(stdout);
			p = list.erase(p);
		} else {
			++p;
		}
	}
	*/
}

// The word "must" in the following function should be interpreted to mean "if we are not missing more than 
// however many rectangles we think we are missing." There is really no provision yet for missing >1, but it 
// shouldn't be too much of an issue.
void SortRects(vector<PolyVertices> &list)
{
	// Fill with invalid structs to ensure no segfaults.
	// TODO: if we decide that missing >1 is not worth coding, we could return here if 4 - list.size() is >1.
	#ifdef DEBUG_SORT
	printf("Empty structs pushed: %d\n", 4 - list.size());
	#endif

	for (int i = list.size(); i < 4; i++)
	{
		PolyVertices temp;
		list.push_back(temp);
	}

	PolyVertices top, left, right, bottom;
	top = left = right = bottom = list[0];

	/* FILL WITH PRELIM. DATA */
	for (vector<PolyVertices>::iterator p = list.begin(); p < list.end(); p++)
	{
		if ((*p).isValid())
		{
			if ((*p).center.y < top.center.y)
				top = *p;	
			if ((*p).center.y > bottom.center.y)
				bottom = *p;
			if ((*p).center.x < left.center.x)
				left = *p;
			if ((*p).center.x > right.center.x)
				right = *p;
		}
	}
	
 	/* FIND WHICH RECTANGLE IS MISSING */ 
 	if (top == left || top == right)
 	{
 		// CASE: top and left are ambiguous
 		if (abs(top.center.x - left.center.x) < 50)
 		{
 			// right and bottom must be correct
 			list[2] = right;
 			list[3] = bottom;
 
 			if (abs(top.center.x - bottom.center.x) < 50) // both are top
 			{
 				list[0] = top;
 				list[1].invalidate();
 			}
 			else // both are left
 			{
 				list[0].invalidate();
 				list[1] = left;
 			}
 		}
 		// CASE: top and right are ambiguous
 		else if (abs(top.center.x - right.center.x) < 50)
 		{
 			// left and bottom must be correct
 			list[1] = left;
 			list[3] = bottom;
 			
 			if (abs(top.center.x - bottom.center.x) < 50) // both are top
 			{
 				list[0] = top;
 				list[2].invalidate();
 			}
 			else // both are right
 			{
 				list[0].invalidate();
 				list[2] = right;
 			}
 		}
	}	

	else if (bottom == left || bottom == right)
	{
		// CASE: bottom and left are ambiguous
 		if (abs(bottom.center.x - left.center.x) < 50)
 		{
 			// top and right must be correct
 			list[0] = top;
 			list[2] = right;
 
 			if (abs(top.center.x - bottom.center.x) < 50) // both are bottom
 			{
 				list[1].invalidate();
 				list[3] = bottom;
 			}
 			else // both are left
 			{
 				list[1] = left;
 				list[3].invalidate();
 			}
 		}
 		// CASE: bottom and right are ambiguous
 		else if (abs(bottom.center.x - right.center.x) < 50)
 		{
 			// top and left must be correct
 			list[0] = top;
 			list[1] = left;
 			
 			if (abs(top.center.x - bottom.center.x) < 50) // both are bottom
 			{
 				list[2].invalidate();
 				list[3] = bottom;
 			}
 			else // both are right
 			{
 				list[2] = right;
 				list[3].invalidate();
 			}
 		}
 	}

	else // none missing.
	{
		list[0] = top;
		list[1] = left;
		list[2] = right;
		list[3] = bottom;
	}

	#ifdef DEBUG_SORT
 	for (int i = 0; i < 4; i++)
 	{
 		printf("[%d]: (%d, %d)\n", i, list[i].center.x, list[i].center.y);
 	}
	#endif
}


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
	cvSetImageROI(dimg, cvRect(0, 0, sz.width, sz.height));

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
		CvSeq* contours; // Raw contours list
		CvSeq* hull; // Current convex hull
		int hullcount; // # of points in hull

		/* PULL RAW IMAGE FROM KINECT */
		pthread_mutex_lock( &mutex_rgb );
		if (draw) { cvCopy(rgbimg, dimg, 0); }
		cvCopy(rgbimg, timg, 0);
		pthread_mutex_unlock( &mutex_rgb );

		/* DILATE */
		IplConvKernel* element = cvCreateStructuringElementEx(3, 3, 1, 1, 0);
		IplConvKernel* element2 = cvCreateStructuringElementEx(5, 5, 2, 2, 0);
		cvDilate(timg, timg, element2, 1);
		cvErode(timg, timg, element, 1);

		/* THRESHOLD*/
		cvThreshold(timg, timg, 100, 255, CV_THRESH_BINARY);

		/* OUTPUT PROCESSED OR RAW IMAGE (FindContours destroys image) */
		if (draw) cvCvtColor(dimg, outimg, CV_GRAY2BGR);

		/* CONTOUR FINDING */
		cvFindContours(timg, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
 
		/* CONVEX HULL + POLYGON APPROXIMATION + CONVERT TO RECTANGLE + FILTER FOR INVALID RECTANGLES */ 
		// Store points to draw line between
		CvPoint* draw1;
		CvPoint* draw2; 
		vector<PolyVertices> rectangleList;

		while (contours) // Run for all polygons
		{
			// List of raw rectangles
			PolyVertices fullrect;

			// Filter noise
			if (fabs(cvContourArea(contours, CV_WHOLE_SEQ)) > 600)
			{
				// Get convex hull
				hull = cvConvexHull2( contours, storage, CV_CLOCKWISE, 1 );
				hullcount = hull->total;

				// Draw hull (red line)
				if (draw) {
					draw1 = (CvPoint*)cvGetSeqElem(hull, hullcount - 1);
					for (int i = 0; i < hullcount; i++)
					{
						draw2 = (CvPoint*)cvGetSeqElem( hull, i );
						cvLine( outimg, *draw1, *draw2, CV_RGB(255,0,0), 1, 8, 0 );
						draw1 = draw2;
					}
				}

				// Convert polys from convex hull to rectangles, fill list
				polyToQuad(hull, &fullrect, outimg);

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

		/* SORT INTO CORRECT BUCKET */
		SortRects(rectangleList);

		/* DRAW & PROCESS MATH; FILL SEND STRUCT */
		// TODO: Might want to make the math stuff static for efficiency.
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
//		if (rectangleList[1].isValid())
//		{
//			outgoing.distLeft = 100 * robot.GetDistance(*(rectangleList[1].points[2]), *(rectangleList[1].points[3]), 1);
//			outgoing.angleLeft = 100 * robot.GetAngle(*(rectangleList[1].points[2]), *(rectangleList[1].points[3]));
//		}
//		if (rectangleList[2].isValid())
//		{
//			outgoing.distRight = 100 * robot.GetDistance(*(rectangleList[2].points[2]), *(rectangleList[2].points[3]), 2);
//			outgoing.angleRight = 100 * robot.GetAngle(*(rectangleList[2].points[2]), *(rectangleList[2].points[3]));
//		}
		if (rectangleList[3].isValid())
		{
			outgoing.distLow = 100 * robot.GetDistance(*(rectangleList[3].points[2]), *(rectangleList[3].points[3]), 3);
			outgoing.angleLow = 100 * robot.GetAngle(*(rectangleList[3].points[2]), *(rectangleList[3].points[3]));
		}

		// Draw filtered rects (thick blue line)
		if (draw) {
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

		CvPoint cent1 = cvPoint(320, 0);
		CvPoint cent2 = cvPoint(320, 480);
 		if (draw) cvLine( outimg, cent1, cent2, CV_RGB(0,255,0), 1, 8, 0 );

		/* SEND TO CRIO */
		sendData(&outgoing, CRRsocket);

		if (axis) axisSetImg(outimg);

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
	int i;
	freenect_context *f_ctx;
	freenect_device *f_dev;

	display = 0;
	draw = 0;
	axis = 0;

	int res = 0;
	int die = 0;
	printf("Code Red Kinect Vision init\n");

	for (i=1; i<argc; i++) {
		if (strcmp(argv[i],"--display")) display = 1;
		else if (strcmp(argv[i],"--axis")) axis = 1;
	}

	if (axis || display) draw = 1;

	/* if (argc > 1 && strcmp(argv[1],"--display") == 0) {
		display = 1;
	} */
			
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
		printf("Could not open device\n");
		return 1;
	}


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

	if (axis) {
		if (axisInit(tempimg) != 0) {
			printf("Failed to launch axis server\n");
		}
	}

	printf("init done\n");

	freenect_start_video(f_dev);

	while(!die && freenect_process_events(f_ctx) >= 0 );

	// If we ever get here, might as well let the webserver shutdown
	if (axis) axisStop();
}
