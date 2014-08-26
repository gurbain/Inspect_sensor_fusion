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
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

// PCL
// #include <pcl/common/common_headers.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>

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
#include "SimpleTriangulator.h"
#include "SfM/Triangulation.h"
#include "SfM/Visualization.h"
#include "SfM/Common.h"

#define SYNCHRONOUS	1
#define ASYNCHRONOUS	0
#define CALIB_DIR	"calib"

using namespace std;
using namespace cv;
//using namespace pcl;


class Halo{

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
		int calib();
		
	private:
		
		// Capture mode
		bool isCamOpen;
		bool isOrfOpen;
		
		// Main objects
		ORF orf;
		Cameras stereo;
		
		// Capture parameters
		int imgNum;
		int imgWidth;
		int imgHeight;
		
		// Calibration parameters
		int boardWidth;
		int boardHeight;
		int numberBoards;
		float squareSize; // in mm
		const int acqStep;
		Size boardSize;
		
		// Load images parameters
		bool load_image;
		ifstream omFile, orfFile;
		int load_num;
		string load_directory;
};

#endif

