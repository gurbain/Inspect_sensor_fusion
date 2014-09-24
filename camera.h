/*! 
* 	\file    camera.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Headers for stereo rig class adapted from VERTIGO project
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/


#ifndef __CAMERA__
#define __CAMERA__

// Project libs
#include "defines.h"
#include "utils.h"
#include "calibration.h"

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
#include<fstream>

using namespace std;
using namespace cv;
	
class Cameras : virtual public Calibration {
		// Private variables
		int iterDummy;

		// Camera handles
		HIDS h_cam1;
		HIDS h_cam2;
		CAMINFO camInfo1;
		CAMINFO camInfo2;
		
		// Camera buffers
		char * act_img_buf1;
		char * act_img_buf2;
		char * last_img_buf1;
		char * last_img_buf2;
		UEYE_IMAGE Cam1_ringbuffer[RING_BUFFER_SIZE];
		UEYE_IMAGE Cam2_ringbuffer[RING_BUFFER_SIZE];
		char CAMERA_1_SERIAL[11];
		char *bufDummy1, *bufDummy2;

		// Timestamp parameters
		double currenttime, zerotime;
		timeval timeRAW;

		// Camera parameters
		float frameRate;
		int exposureTime;
		int imgWidth;
		int imgHeight;
		Size imageSize;
		int hwGain;
		int pixelClockFreq;
		float maxFrameRate;
		int flashDelay;
		int flashDuration;
		int imgBufferCount;
		int imgBitsPixel;
		int numberOfBytesToCopy;

		// Synch parameters
		int prevImgNum;
		
		// File load parameters
		string load_dir;
		bool load_image;

		// File save parameters
		string save_dir;
		ofstream tsfile;
		string timestamps;
		int imgNum;
		int tslast;		

	public:
		// Cameras parameters
		char CAMERA_1_SERIAL_temp1[11];
		char CAMERA_1_SERIAL_temp2[11];
		char LEFT_CAM_SERIAL[16][11];
		int OPTICS_ID[16];
		int leftImgNum;
		int rightImgNum;
		bool useSynchCams;
		bool reduceImageSizeTo320x240;
		bool useCameras;
		
		// Calibration matrices
		Mat intrinsicMatrixL, distorsionCoeffsL, intrinsicMatrixR, distorsionCoeffsR, projMatrixL, projMatrixR, rotMatrixL, rotMatrixR;
		Mat Q, R, T, E, F;
		Mat mapxL, mapyL, mapxR, mapyR;
		double Tx, Ty, Tz, f, cxL, cyL, cxR, cyR;

		// Public Functions
		Cameras(void);
		
		double getFrameRate();
		void setFrameRate(float newFR);
		double getExposureTime();
		void setExposureTime(int newET);
		void setHWGain(int gain);
		char* getCam1Serial();
		void setCam1Serial(const char* serialNum);
		int getImageWidth();
		int getImageHeight();

		unsigned int init();
		unsigned int init(string dir);
		unsigned int close();
		unsigned int start();
		unsigned int stop();
		unsigned int captureTwoImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, TimeStamp& ts, int& synchCheckFlag, int num=0);
		unsigned int captureTwoRectifiedImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, TimeStamp& ts, int num=0, string filename="OM_calib.xml");
		unsigned int saveTwoRectifiedImages();
		unsigned int saveTwoImages();
		int calib(string filename="OM_calib.xml");
		int capture3Dcloud(vector<Point3d>& pointcloud, vector<Vec3b>& rgbcloud, int num=0, int downsampling=4, string filename="OM_calib.xml");
		
		void parseParameterFile();
};

#endif 
