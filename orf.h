/*! 
* 	\file    orf.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
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
#include "opencv2/contrib/contrib.hpp"

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
#include <fstream>
#include <sys/stat.h>

// Project libs
#include "defines.h"
#include "utils.h"
#include "calibration.h"
#include "visualization.h"

using namespace std;
using namespace cv;

class ORF : virtual public Calibration {

	public:
		// Common variables
		string device_id_;
		string lib_version_;
		
		// Calibration matrices
		Mat intrinsicMatrix, distorsionCoeffs;
		double f, cx, cy;
		
		// Public functions
		ORF ();
		~ORF ();

		int init(string directory);
		int init();
		int close();
		int captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, int num=0);
		int captureRectifiedOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, int num=0, string filename="ORF_calib.xml");
		int saveRectifiedOrf();
		int saveOrf();
		int capture3Dcloud(vector<Point3d>& pointcloud, vector<Vec3b>& rgbcloud, int num=0, int downsampling=4, string filename="ORF_calib.xml");
		int calib(string filename="ORF_calib.xml");
		
		int setAutoExposure (bool on);
		int setIntegrationTime (int time);
		int setAmplitudeThreshold (int thresh);
		int setModulationFrequency (int freq);
		int getIntegrationTime ();
		double getModulationFrequency ();
		int getAmplitudeThreshold ();

	private:
		// Camera parameters
		int imgWidth;
		int imgHeight;
		Size imageSize;
		bool auto_exposure;
		int integration_time, modulation_freq, amp_threshold;
		string ether_addr;
		
		// Camera variables
		CMesaDevice* orfCam_;			
		ImgEntry* imgEntryArray_;
		float *buffer_, *xp_, *yp_, *zp_;
		int integration_time_, modulation_freq_;
		bool use_filter_;
		
		// Rectification variables
		Mat mapx;
		Mat mapy;
		
		// File save parameters
		string save_dir;
		ofstream tsfile;
		string timestamps;
		int imgNum;
		int tslast;
		
		// File load parameters
		string load_dir;
		bool load_image;
		
		// Private functions
		void SafeCleanup();
		string getDeviceString ();
		string getLibraryVersion ();
};

#endif

