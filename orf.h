/*! 
* 	\file    orf.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Headers for optical range finder class
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef ORF_HH
#define ORF_HH

// OpenCV libs
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

// ORF libs
#include <libMesaSR.h>
#include <definesSR.h>

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

// Project libs
#include "defines.h"
#include "utils.h"

#define DEBUG_CHESSBOARD

namespace orf
{
	using namespace std;
	using namespace cv;
  
	const int ORF_COLS = 176;
	const int ORF_ROWS = 144;
	const int ORF_IMAGES = 3;

	class ORF {
	
		public:
			// Public variables
			string device_id_;
			string lib_version_;
			
			// Public functions
			ORF (bool use_filter=USE_FILTER);
			~ORF ();

			int initOrf(bool auto_exposure=true, int integration_time=100, int modulation_freq=21, int amp_threshold=20, string ether_addr="192.168.1.42");
			int closeOrf();
			int captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts);
			int captureRectifiedOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, string filename="ORF_calib.xml");
			int intrinsicCalib(string filename="ORF_calib.xml");
			
			int setAutoExposure (bool on);
			int setIntegrationTime (int time);
			int setAmplitudeThreshold (int thresh);
			int setModulationFrequency (int freq);
			int getIntegrationTime ();
			double getModulationFrequency ();
			int getAmplitudeThreshold ();

		private:
			// Camera variables
			CMesaDevice* orfCam_;			
			ImgEntry* imgEntryArray_;
			float *buffer_, *xp_, *yp_, *zp_;
			int integration_time_, modulation_freq_;
			bool use_filter_;
			
			// Camera parameters
			int imgWidth;
			int imgHeight;
			Size imageSize;
			
			// Calibration parameters
			int boardWidth;
			int boardHeight;
			int numberBoards;
			float squareSize; // in mm
			const int acqStep;
			Size boardSize;
			
			// Rectification variables
			Mat mapx;
			Mat mapy;
			
			// Private functions
			vector<Point3f> Create3DChessboardCorners(Size boardSize, float squareSize);
			void SafeCleanup();
			string getDeviceString ();
			string getLibraryVersion ();
	};
};

#endif

