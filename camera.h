/*! 
* 	\file    camera.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Headers for stereo rig class
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/


#ifndef __CAMERA__
#define __CAMERA__

// Project libs
#include "defines.h"

// OpenCV libs
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

// uEye camera libs
#include <uEye.h>
#include <ueye_deprecated.h>

// Common libs
#include <stdint.h>
#include <iomanip>
#include <limits>
#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

typedef struct _UEYE_IMAGE
{
	char    *pBuf;
	int     img_id;
	int     img_seqNum;
	int     nBufferSize;
} UEYE_IMAGE;

namespace cam
{

	using namespace std;
	using namespace cv;
	
	class Cameras
	{
		// Private variables
		int iterDummy;

		HIDS h_cam1;
		HIDS h_cam2;
		CAMINFO camInfo1;
		CAMINFO camInfo2;
		char * act_img_buf1;
		char * act_img_buf2;
		char * last_img_buf1;
		char * last_img_buf2;
		UEYE_IMAGE Cam1_ringbuffer[RING_BUFFER_SIZE];
		UEYE_IMAGE Cam2_ringbuffer[RING_BUFFER_SIZE];
		char CAMERA_1_SERIAL[11];

		char *bufDummy1, *bufDummy2;

		double currenttime, zerotime;
		timeval timeRAW;

		
		float frameRate;
		int exposureTime;
		
		int imgWidth;
		int imgHeight;

		int hwGain;

		int pixelClockFreq; // in MHz
		float maxFrameRate;
		int flashDelay;
		int flashDuration;
		int imgBufferCount;
		int imgBitsPixel;
		int numberOfBytesToCopy;

		int prevImgNum;


		// Private Functions
		unsigned int initTwoCamerasNotSynch();
		unsigned int closeTwoCamerasNotSynch();
		unsigned int startTwoCamerasNotSynch();
		unsigned int stopTwoCamerasNotSynch();
		unsigned int captureTwoImagesNotSynch(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2);

		unsigned int initTwoCamerasSynch();
		unsigned int closeTwoCamerasSynch();
		unsigned int startTwoCamerasSynch();
		unsigned int stopTwoCamerasSynch();
		unsigned int captureTwoImagesSynch(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, int& synchCheckFlag);


	public:

		// Public Variables
		char CAMERA_1_SERIAL_temp1[11];
		char CAMERA_1_SERIAL_temp2[11];

		char LEFT_CAM_SERIAL[16][11];
		int OPTICS_ID[16];
		
		int leftImgNum;
		int rightImgNum;

		bool useSynchCams;
		bool reduceImageSizeTo320x240;
		bool useCameras;

		int getImageWidth();
		int getImageHeight();

		// Public Functions
		Cameras(void);
		double getFrameRate();
		void setFrameRate(float newFR);
		double getExposureTime();
		void setExposureTime(int newET);
		void setHWGain(int gain);
		char* getCam1Serial();
		void setCam1Serial(const char* serialNum);

		unsigned int initTwoCameras();
		unsigned int closeTwoCameras();
		unsigned int startTwoCameras();
		unsigned int stopTwoCameras();
		unsigned int captureTwoImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, int& synchCheckFlag);

	};


	class Rectifier
	{
		// Private variables
		char intrinsic_filename[200];
		char extrinsic_filename[200];

		cv::Mat LeftRectMap1, LeftRectMap2;
		cv::Mat RightRectMap1, RightRectMap2;

		cv::Mat Q;
		cv::Mat R, T, R1, P1, R2, P2;
		cv::Mat M1, D1, M2, D2;
		double Tx, Ty, Tz;
		double f;
		double cx;
		double cy;
		cv::Rect roi1, roi2;


	public:
		// Public variables
		bool rectifierOn;

		// Public functions
		Rectifier();
		int calcRectificationMaps(int imgwidth, int imgheight, const char calibParamDir[200]);
		int rectifyImages(cv::Mat& leftImgFrame, cv::Mat& rightImgFrame);
		void getCameraParameters(cv::Mat& Qin, cv::Mat& Rin, cv::Mat& Tin, cv::Mat& R1in, cv::Mat& P1in,
				cv::Mat& R2in, cv::Mat& P2in, cv::Mat& M1in, cv::Mat& D1in, cv::Mat& M2in, cv::Mat& D2in,
				double& Txin, double& Tyin, double& Tzin, double& fin, double& cxin, double& cyin);
		void getCameraParameters(cv::Mat& Rin, cv::Mat& Tin, cv::Mat& M1in, cv::Mat& D1in, cv::Mat& M2in, cv::Mat& D2in);
		void getCameraParameters(cv::Mat& Qin);
		void getCameraParameters(double & cx_left,double &  cy_left,double &  cx_right,double &  cy_right,
				double &  f_left,double &  f_right,double &  Tx,double &  Ty,double &  Tz);
	};
};

#endif 
