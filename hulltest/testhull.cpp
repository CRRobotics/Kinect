/*  freenectopencv.cpp

    Copyright (C) 2010  Arne Bernin <arne@alamut.de>

    This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
see:
http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
http://www.gnu.org/licenses/gpl-3.0.txt
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <libfreenect/libfreenect.h>
#include <pthread.h>

#define CV_NO_BACKWARD_COMPATIBILITY

#include <cv.h>
#include <highgui.h>
#include <cvblobslib/Blob.h>
#include <cvblobslib/BlobResult.h>
#include <cvblobslib/BlobExtraction.h>

#define FREENECTOPENCV_WINDOW_N "Normalimage"
#define FREENECTOPENCV_RGB_DEPTH 3 // Currently used for tempImg.
#define FREENECTOPENCV_IR_DEPTH 1 // Current display depth.
#define FREENECTOPENCV_RGB_WIDTH 640
#define FREENECTOPENCV_RGB_HEIGHT 480


IplImage* rgbimg = 0;
IplImage* tempimg = 0;
pthread_mutex_t mutex_rgb = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;

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
	IplImage* dimg = cvCloneImage(rgbimg); // Image we draw on
	CvSize sz = cvSize( timg->width & -2, timg->height & -2);
	IplImage* outimg = cvCreateImage(sz, 8, 3);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours; // Raw contours list
	CvSeq* hull; // Current convex hull
	int hullcount; // # of points in hull
	CvSeq* polyseq = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage ); // Sequence to run ApproxPoly on
	CvSeq* polyresult; // Single contour being processed

	IplImage *pyr = cvCreateImage(cvSize(sz.width/2, sz.height/2), 8, 1);

	// Set region of interest
	cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));
	cvSetImageROI(dimg, cvRect(0, 0, sz.width, sz.height));

	// use image polling
	while (1) {
		pthread_mutex_lock( &mutex_rgb );
		cvCopy(rgbimg, dimg, 0);
		cvCopy(rgbimg, timg, 0);
		pthread_mutex_unlock( &mutex_rgb );

		// BLUR TEST
		// cvPyrDown(dimg, pyr, 7);
		// cvPyrUp(pyr, timg, 7);

		// DILATE TEST
		IplConvKernel* element = cvCreateStructuringElementEx(3, 3, 1, 1, 0);
		IplConvKernel* element2 = cvCreateStructuringElementEx(5, 5, 2, 2, 0);
		cvDilate(timg, timg, element2, 1);
		cvErode(timg, timg, element, 1);

		// THRESHOLD TEST 
		cvThreshold(timg, timg, 200, 255, CV_THRESH_BINARY);

		// Output processed or raw image.
		cvCvtColor(dimg, outimg, CV_GRAY2BGR);

		// CONTOUR FINDING + CONVEX HULL
		cvFindContours(timg, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

		CvPoint draw1; // Store points to draw line between
		CvPoint draw2; // ''
		while (contours)
		{
			// Filter noise
			if (fabs(cvContourArea(contours, CV_WHOLE_SEQ)) > 100)
			{
				hull = cvConvexHull2( contours, storage, CV_CLOCKWISE, 0 );
				hullcount = hull->total;
				printf("Hull count: %d\n", hullcount);

				// Draw hull and push into polyseq
				draw1 = **CV_GET_SEQ_ELEM( CvPoint*, hull, hullcount - 1 );
				for (int i = 0; i < hullcount; i++)
				{
					draw2 = **CV_GET_SEQ_ELEM( CvPoint*, hull, i );
					printf("Point: %d, %d\n", draw2.x, draw2.y);
					fflush(stdout);
					cvSeqPush( polyseq, &draw2 );
					printf("Pushed");
					fflush(stdout);
					cvLine( outimg, draw1, draw2, CV_RGB(255,0,0), 2, 8, 0 );

					// Figure out how to draw onto a grayscale image.
					// Also, does FindContours just clear the image, or change it such
					// that it no longer works as a normal image and cannot be drawn on?
					// cvLine( dimg, draw1, draw2, 255, 2, 8, 0 );

					draw1 = draw2;
				}
				// polyseq->flags = polyseq->flags | 512;

				hull->flags = hull->flags | 512;
				cvApproxPoly( hull, sizeof(CvPoint), storage, CV_POLY_APPROX_DP, 1, 0);
			}
			contours = contours->h_next;
		}


		// Unsure what to do here. One solution is to draw the lines in the above block back into timg, (or another buffer)
		// and cvFindContours() again, then cvApproxPoly the result, but that is inelegant.
		// Ideally we could get cvApproxPoly to work on either the raw hull or a copied CvSeq.

		// CURRENTLY TRYING: Drawing then using FindContours again.


//		while (contours)
//		{
//			// Approximate contour, accuracy proportional to perimeter of contour; may want to tune accuracy.
//			result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours) * 0.02, 0);
//			// Filter small contours and contours w/o 4 vertices (filters noise, finds rectangles)
//			if (result->total == 4 && 
//				fabs(cvContourArea(result, CV_WHOLE_SEQ)) > 600 && 
//				cvCheckContourConvexity(result))
//			{
//				// Skipped checking whether angles were close to 90 degrees here; may want to implement.
//				// Probably also want to check if it's square enough to filter out ex. long windows.
//
//				for (int i = 0; i < 4; i++)
//				{
//					// Write vertices to output sequence
//					cvSeqPush(squares, (CvPoint*)cvGetSeqElem(result, i));
//				}
//			}
//
//			// Take next contour
//			contours = contours->h_next;
//		}
//
//
//		// DRAW RECTANGLES
//		CvSeqReader reader;
//		cvStartReadSeq(squares, &reader, 0);
//
//		// Read 4 points at a time
//		CvPoint pt[4];
//		CvPoint *rect = pt;
//		CvRect out[4];
//		CvRect *outrect = out;
//		for (int i = 0; i < squares->total; i += 4)
//		{
//			int count = 4;
//			
//			// Which point is which corner is unpredictable.
//			CV_READ_SEQ_ELEM(pt[0], reader); 
//			CV_READ_SEQ_ELEM(pt[1], reader);
//			CV_READ_SEQ_ELEM(pt[2], reader);
//			CV_READ_SEQ_ELEM(pt[3], reader);
//			// Draw rectangle on output
//			cvPolyLine(outimg, &rect, &count, 1, 1, CV_RGB(0,255,0), 1, CV_AA, 0);
//		}

		// Print on order
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
