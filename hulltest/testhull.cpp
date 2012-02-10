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


#define FREENECTOPENCV_WINDOW_N "Normalimage"
#define FREENECTOPENCV_RGB_DEPTH 3 // Currently used for tempImg.
#define FREENECTOPENCV_IR_DEPTH 1 // Current display depth.
#define FREENECTOPENCV_RGB_WIDTH 640
#define FREENECTOPENCV_RGB_HEIGHT 480

typedef struct 
{
	CvPoint* points[4];
	CvPoint center;
	int dist;
}PolyVertices;

IplImage* rgbimg = 0;
IplImage* tempimg = 0;
pthread_mutex_t mutex_rgb = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;


void polyToQuad(CvSeq* sequence, PolyVertices *poly, IplImage* img ){
	// Sequence is the hull
	// poly->points is the array that stores the points of the rectangle
	// img is the image we are drawing lines and such on. Not strictly necessary.
	printf("Sequence: %d\n", sequence->total);

	/*CALCULATE CENTER*/
	// int center[2] = {0}; //center point of the poly x, y
	int extremes[4]; //minX, maxX, minY, maxY
	extremes[0] = ((CvPoint*)cvGetSeqElem(sequence, 0))->x;
	extremes[1] = extremes[0];
	extremes[2] = ((CvPoint*)cvGetSeqElem(sequence, 0))->y;
	extremes[3] = extremes[2];


	CvSeqReader seqReader;
	cvStartReadSeq(sequence, &seqReader, 0);
	for(int i = 0; i < sequence->total; i++){
		CvPoint* point = (CvPoint*) seqReader.ptr;

		// printf("Center x, y: %d, %d\n", point->x, point->y); 

		if(point->x < extremes[0]) extremes[0] = point->x;
		if(point->x > extremes[1]) extremes[1] = point->x;

		if(point->y < extremes[2]) extremes[2] = point->y;
		if(point->y > extremes[3]) extremes[3] = point->y;
		CV_NEXT_SEQ_ELEM(seqReader.seq->elem_size, seqReader);
	}
	poly->center.x = (extremes[0] + extremes[1])/2;
	poly->center.y = (extremes[2] + extremes[3])/2;

	cvCircle( img, poly->center, 2, CV_RGB(255,0,255), -1, 8, 0 );
	printf("Calculated Center (%d): %i, %i\n", sequence->total, poly->center.x, poly->center.y);

	/*CALCULATE DISTANCES FROM CENTER AND CALCULATE MAX DIST IN EACH QUADRANT*/
	double distances[4] = {0, 0, 0, 0};
	double distance; //current point's distance

	int qKey = 0;

	CvSeqReader seqReader2;
	cvStartReadSeq(sequence, &seqReader2, 0);
	for(int i = 0; i < sequence->total; i++){
		CvPoint* point = (CvPoint*) seqReader2.ptr;
		distance = pow((double)point->x - poly->center.x, 2) + pow((double)point->y - poly->center.y, 2);
		qKey = ((point->x > poly->center.x ? 1 : 0) + (point->y < poly->center.y ? 2 : 0));

		//printf("X: %d, Y: %d, Dist: %f, Key: %d\n ", point->x, point->y, distance, qKey);
		/* FILL POINTS */
		if(distance > distances[qKey]){
			poly->points[qKey] = point;
			distances[qKey] = distance;

		}
		CV_NEXT_SEQ_ELEM(seqReader2.seq->elem_size, seqReader2);
	}

	if(!(poly->points[0] == NULL || poly->points[1] == NULL || poly->points[2] == NULL || poly->points[3] == NULL))
	{
		/* FILL poly->dist */
		poly->dist = pow((double)poly->points[0]->x - poly->center.x, 2) + pow((double)poly->points[0]->y - poly->center.y, 2);
		printf("Dist0: %d\n", poly->dist);

		// Draw vertices
		cvCircle( img, *poly->points[0], 4, CV_RGB(255,0,255), -1, 8, 0 );
		cvCircle( img, *poly->points[1], 4, CV_RGB(255,0,255), -1, 8, 0 );
		cvCircle( img, *poly->points[2], 4, CV_RGB(255,0,255), -1, 8, 0 );
		cvCircle( img, *poly->points[3], 4, CV_RGB(255,0,255), -1, 8, 0 );
	}
// 	printf("Vertices returned:  (%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",
// 			poly->points[0]->x,poly->points[0]->y,poly->points[1]->x,poly->points[1]->y,
// 			poly->points[2]->x,poly->points[2]->y,poly->points[3]->x,poly->points[3]->y);
}

// Return whether we want to delete poly.
bool FilterSub(vector<PolyVertices> &list, PolyVertices poly)
{
	for (int i = 0; i < list.size(); i++)
	{
		double result = sqrt(pow((double)list[i].center.x - (double)poly.center.x, 2) + pow((double)list[i].center.y - (double)poly.center.y, 2));
		if ((result < 50) && (list[i].dist != poly.dist))
			if (list[i].dist > poly.dist)
				return true;
	}
	return false;
}

void FilterInnerRects(vector<PolyVertices> &list)
{
	ugly:
	for (vector<PolyVertices>::iterator p = list.begin(); p < list.end(); p++)
	{
		printf("Filter\n");
		if (FilterSub(list, *p))
		{
			printf("Delete\n");
			fflush(stdout);
			list.erase(p);
			goto ugly;
		}
	}
}


// callback for rgbimage, called by libfreenect
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	// lock mutex for opencv rgb image
	pthread_mutex_lock( &mutex_rgb );
	memcpy(rgbimg->imageData, rgb, FREENECT_VIDEO_RGB_SIZE);
	// unlock mutex
	pthread_mutex_unlock( &mutex_rgb );
}


/*
 * thread for displaying the opencv content
 */
void *cv_threadfunc (void *ptr) {
	IplImage* timg = cvCloneImage(rgbimg); // Image we do our processing on
	IplImage* dimg = cvCloneImage(timg); // Image we draw on
	CvSize sz = cvSize( timg->width & -2, timg->height & -2);
	IplImage* outimg = cvCreateImage(sz, 8, 3);

	CvMemStorage* storage = cvCreateMemStorage(0);

	// Set region of interest
	cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));
	cvSetImageROI(dimg, cvRect(0, 0, sz.width, sz.height));

	// Main loop
	while (1) 
	{
		CvSeq* polyseq = cvCreateSeq( CV_SEQ_KIND_CURVE | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage ); // Sequence to run ApproxPoly on
		CvSeq* contours; // Raw contours list
		CvSeq* hull; // Current convex hull
		int hullcount; // # of points in hull

		pthread_mutex_lock( &mutex_rgb );
		cvCopy(rgbimg, dimg, 0);
		cvCopy(rgbimg, timg, 0);
		pthread_mutex_unlock( &mutex_rgb );

		/* DILATE TEST */
		IplConvKernel* element = cvCreateStructuringElementEx(3, 3, 1, 1, 0);
		IplConvKernel* element2 = cvCreateStructuringElementEx(5, 5, 2, 2, 0);
		cvDilate(timg, timg, element2, 1);
		cvErode(timg, timg, element, 1);

		/* THRESHOLD TEST */
		cvThreshold(timg, timg, 100, 255, CV_THRESH_BINARY);

		/* Output processed or raw image. */
		cvCvtColor(dimg, outimg, CV_GRAY2BGR);

		/* CONTOUR FINDING */
		cvFindContours(timg, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
 
		/* CONVEX HULL + POLYGON APPROXIMATION*/
		CvPoint* draw1; // Store points to draw line between
		CvPoint* draw2; // ''
		vector<PolyVertices> rectangleList;

		printf("\n\n");
		while (contours)
		{
			// List of raw rectangles
			PolyVertices fullrect;
			memset((char*)&fullrect, 0, sizeof(PolyVertices));

			// Filter noise
			if (fabs(cvContourArea(contours, CV_WHOLE_SEQ)) > 600)
			{
				hull = cvConvexHull2( contours, storage, CV_CLOCKWISE, 1 );
				hullcount = hull->total;

				// Draw hull
 				draw1 = (CvPoint*)cvGetSeqElem(hull, hullcount - 1);
 				for (int i = 0; i < hullcount; i++)
 				{
 					draw2 = (CvPoint*)cvGetSeqElem( hull, i );
 					cvLine( outimg, *draw1, *draw2, CV_RGB(255,0,0), 1, 8, 0 );
 					draw1 = draw2;
					// cvCircle( outimg, *draw2, 4, CV_RGB(0,100,150), -1, 8, 0 );
 				}

				// Draw polygon
				polyToQuad(hull, &fullrect, outimg);
				if(!(fullrect.points[0] == NULL || fullrect.points[1] == NULL || fullrect.points[2] == NULL || fullrect.points[3] == NULL))
				{
					/* FILL rectangleList */
					rectangleList.push_back(fullrect);

					printf("RESULT: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",
					fullrect.points[0]->x, fullrect.points[0]->y, fullrect.points[1]->x, fullrect.points[1]->y,
						fullrect.points[2]->x, fullrect.points[2]->y, fullrect.points[3]->x, fullrect.points[3]->y);
					fflush(stdout);
				}

			}
			cvClearSeq(polyseq);
			contours = contours->h_next;
		}

		/* FILTER OVERLAPPING RECTANGLES */
		printf("RectangleList: %d\n", rectangleList.size());
		FilterInnerRects(rectangleList);
		printf("RectangleList: %d\n", rectangleList.size());

		if (outimg)
		{
			for (int i = 0; i < rectangleList.size(); i++)
			{
				cvLine( outimg, *(rectangleList[i].points[3]), *(rectangleList[i].points[2]), CV_RGB(0,0,255), 2, 8, 0 );
				cvLine( outimg, *(rectangleList[i].points[2]), *(rectangleList[i].points[0]), CV_RGB(0,0,255), 2, 8, 0 );
				cvLine( outimg, *(rectangleList[i].points[0]), *(rectangleList[i].points[1]), CV_RGB(0,0,255), 2, 8, 0 );
				cvLine( outimg, *(rectangleList[i].points[1]), *(rectangleList[i].points[3]), CV_RGB(0,0,255), 2, 8, 0 );
			}
		}

		/* ADD MATH/SENDING FOR rectangleList HERE */

		if( cvWaitKey( 15 )==27 )
		{
			// Empty for now.
		}

		cvShowImage (FREENECTOPENCV_WINDOW_N,outimg);
		cvClearMemStorage(storage);
	}
	pthread_exit(NULL);
}


int main(int argc, char **argv)
{
	freenect_context *f_ctx;
	freenect_device *f_dev;


	int res = 0;
	int die = 0;
	printf("Kinect camera test\n");

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
		printf("Could not open device\n");
		return 1;
	}

	cvNamedWindow( FREENECTOPENCV_WINDOW_N, CV_WINDOW_AUTOSIZE );
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
