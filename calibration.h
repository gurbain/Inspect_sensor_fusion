/*! 
* 	\file    calibration.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Tools for ORF, Stereo Mount and Halo Optics calibration
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef CALIBRATION_HH
#define CALIBRATION_HH

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
#include "triangulator.h"
#include "orientation.h"
#include "SfM/Triangulation.h"
#include "SfM/Visualization.h"
#include "SfM/Common.h"

using namespace std;
using namespace cv;

class Calibration {
	
	public:
		int orfCalib(string filename);
		int stereoCalib(string filename);
		int haloCalib(string filename);
		
		int acqHaloCheckPoints(vector<vector<Point2f> >& imagePointsL, vector<vector<Point2f> >& imagePointsR, vector<vector<Point2f> >& imagePointsT, vector<Mat>& savedConf, vector<Mat>& savedDepth);
		int computeHalo3DPoints(vector<vector<Point2f> >& imagePointsL, vector<vector<Point2f> >& imagePointsR, vector<vector<Point2f> >& imagePointsT, vector<Mat>& savedConf, vector<Mat>& savedDepth, vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, Mat& RLR, Mat& TLR);
		int computeHaloMinimization(vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, Mat& MTL);
		int fillHaloMatrices(Mat& RLR, Mat& TLR, Mat& MTL);
		
		// Those virtual functions are declared here cause calibration process need to acquire images
		virtual int captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, int num=0);
		virtual unsigned int captureTwoImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, TimeStamp& ts, int& synchCheckFlag, int num=0);
		virtual int captureAllRectifiedImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag=ASYNCHRONOUS);
};

#endif