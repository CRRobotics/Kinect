/*  freenectopencv.cpp

Copyright (C) 2010  Arne Bernin <arne@alamut.de>

This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
see:
 http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 http://www.gnu.org/licenses/gpl-3.0.txt
*/


/*
 * Makefile for ubuntu, assumes that libfreenect.a is in /usr/lib, and libfreenect.h is in /usr/include
 *
 * make sure you have the latest version of freenect from git!

***************************************************************************************************************************
* Makefile
***************************************************************************************************************************
 CXXFLAGS =     -O2 -g -Wall -fmessage-length=0 `pkg-config opencv --cflags ` -I /usr/include/libusb-1.0



 OBJS =         freenectopencv.o

LIBS =    `pkg-config opencv --libs` -lfreenect

TARGET =        kinectopencv

$(TARGET):      $(OBJS)
        $(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:    $(TARGET)

clean:
        rm -f $(OBJS) $(TARGET)


***************************************************************************************************************************
* End of Makefile
***************************************************************************************************************************


*/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <libfreenect/libfreenect.h>
#include <pthread.h>


#define CV_NO_BACKWARD_COMPATIBILITY

#include <cv.h>
#include <highgui.h>


#define FREENECTOPENCV_WINDOW_D "Depthimage"
#define FREENECTOPENCV_WINDOW_N "Normalimage"
#define FREENECTOPENCV_RGB_DEPTH 3
#define FREENECTOPENCV_IR_DEPTH 1
#define FREENECTOPENCV_DEPTH_DEPTH 1
#define FREENECTOPENCV_RGB_WIDTH 640
#define FREENECTOPENCV_RGB_HEIGHT 480
#define FREENECTOPENCV_DEPTH_WIDTH 640
#define FREENECTOPENCV_DEPTH_HEIGHT 480




// IplImage* depthimg = 0;
IplImage* rgbimg = 0;
IplImage* tempimg = 0;
pthread_mutex_t mutex_depth = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_rgb = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;


// callback for depthimage, called by libfreenect
// void depth_cb(freenect_device *dev, void *depth, uint32_t timestamp)
// 
// {
//         cv::Mat depth8;
//         cv::Mat mydepth = cv::Mat( FREENECTOPENCV_DEPTH_WIDTH,FREENECTOPENCV_DEPTH_HEIGHT, CV_16UC1, depth);
//         
//         mydepth.convertTo(depth8, CV_8UC1, 1.0/4.0);
//         pthread_mutex_lock( &mutex_depth );
//         memcpy(depthimg->imageData, depth8.data, 640*480);
//         // unlock mutex
//         pthread_mutex_unlock( &mutex_depth );
// 
// }



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
	IplImage* timg = cvCloneImage(rgbimg); 
	CvSize sz = cvSize( timg->width & -2, timg->height & -2);
	IplImage *pyr = cvCreateImage(cvSize(sz.width/2, sz.height/2), 8, 1);

	CvStorage* storage = cvCreateMemStorage(0);
	CvSeq* contourResult;
	CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);
	
	cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));

	// use image polling
        while (1) {
                // //lock mutex for depth image
                // pthread_mutex_lock( &mutex_depth );
                // // show image to window
                // cvCvtColor(depthimg,tempimg,CV_GRAY2BGR);
                // cvCvtColor(tempimg,tempimg,CV_HSV2BGR);
                // cvShowImage(FREENECTOPENCV_WINDOW_D,tempimg);
                // //unlock mutex for depth image
                // pthread_mutex_unlock( &mutex_depth );

                //lock mutex for rgb image
                pthread_mutex_lock( &mutex_rgb );
		cvCopy(rgbimg, timg, 0);
                //unlock mutex
                pthread_mutex_unlock( &mutex_rgb );

		// /* Blur test */
		// cvPyrDown(timg, pyr, 7);
		// cvPyrUp(pyr, timg, 7);

		// /* Threshold test */
		// cvThreshold(timg, timg, 90, 255, CV_THRESH_BINARY);

		// Contour finding
		cvFindContours(timg, storage, &contourResult, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		
		// START HERE

		cvShowImage (FREENECTOPENCV_WINDOW_N,timg);
                // wait for quit key
                if( cvWaitKey( 15 )==27 )
                                break;

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

        // cvNamedWindow( FREENECTOPENCV_WINDOW_D, CV_WINDOW_AUTOSIZE );
        cvNamedWindow( FREENECTOPENCV_WINDOW_N, CV_WINDOW_AUTOSIZE );
        // depthimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
        rgbimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_IR_DEPTH);
	printf("rgbimg:%x\n", rgbimg);
	fflush(stdout);
        tempimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);

        // create opencv display thread
        res = pthread_create(&cv_thread, NULL, cv_threadfunc, (void*) tempimg);
        if (res) {
                printf("pthread_create failed\n");
                return 1;
        }

        // freenect_set_depth_callback(f_dev, depth_cb);
        freenect_set_video_callback(f_dev, rgb_cb);
        freenect_set_video_format(f_dev, FREENECT_VIDEO_IR_8BIT);

        printf("init done\n");



        // freenect_start_depth(f_dev);
        freenect_start_video(f_dev);

        while(!die && freenect_process_events(f_ctx) >= 0 );


}
