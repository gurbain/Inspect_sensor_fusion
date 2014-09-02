/*! 
* 	\file    orf.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Headers for a Halo class (ORF + Cameras)
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef HALO_HH
#define HALO_HH

// OpenCV libs
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


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
#include "orf.h"
#include "camera.h"

using namespace std;
using namespace cv;

class Halo : virtual public Calibration
{
	public:
		// Public functions
		Halo();
		~Halo();
		
		int init();
		int init(string directory);
		int close();
		int captureAllImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag=ASYNCHRONOUS);
		int captureAllRectifiedImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag=ASYNCHRONOUS);
		int capture3Dcloud();
		int calib(string filename="HALO_calib.xml");
		
	private:		
		// Main objects
		ORF orf;
		Cameras stereo;
		
		// Capture mode
		bool isCamOpen;
		bool isOrfOpen;
		
		// Capture parameters
		int imgNum;
		int imgWidth;
		int imgHeight;
		
		// Load images parameters
		bool load_image;
		ifstream omFile, orfFile;
		int load_num;
		string load_directory;
};

#endif

