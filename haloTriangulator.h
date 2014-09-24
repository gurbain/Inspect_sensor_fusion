/*! 
* 	\file    haloTriangulator.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    S 2014
* 	\version 0.1
* 	\brief   Triangulation and fusion from stereo and orf images
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef HALOTRIANGULATOR_HH_
#define HALOTRIANGULATOR_HH_

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"

#include <numeric>

#include "defines.h"
#include "utils.h"
#include "triangulator.h"
#include "visualization.h"

#define NONE_FLAG	0
#define TIME_FLAG	1
#define CONF_FLAG	2
#define DEBUG_FLAG	4

using namespace cv;
using namespace std;

class HaloTriangulator  {
	
private:
	OrfTriangulator *orf;
	StereoTriangulator *stereo;
	StereoTriangulator *stereo2;
	
	// Calibration matrices
	Mat MLT, MTL, MRT;
	Mat intrinsicL, intrinsicR, intrinsicT, projMatrixStereo;
	
	// Intersection cloud parameters
	double a1, a2, a3, a4, b;
	
	// Other
	double baseline;
	
	// Private functions
	void computeInterParams();
	double estimateNoiseVariance(Mat& iL, Mat& iR);
	
public:
	// Constructor
	HaloTriangulator(string filenameORF="ORF_calib.xml", string filenameOM="OM_calib.xml", string filenameHALO="HALO_calib.xml");
	HaloTriangulator(Mat& projMatrixR_, Mat& intrinsicT_, Mat& intrinsicL_, Mat& intrinsicR_, string filenameHALO="HALO_calib.xml");
	HaloTriangulator(Mat& intrinsicT_, Mat& intrinsicL_, Mat& intrinsicR_, Mat& MLT_, Mat& MTL_, Mat& MRT_);
	
	vector<Point3d> fusion(Mat& dT, Mat& cT, Mat& iL, Mat& iR, vector<Vec3b>& rgbcloud, int flag=NONE_FLAG);
	Point2d reprojectT(double x, double y, double z);
	Point2d reprojectT(const Point3d & p) {
		return reprojectT(p.x, p.y, p.z);
	}
	Point2d reprojectL(double x, double y, double z);
	Point2d reprojectL(const Point3d & p) {
		return reprojectL(p.x, p.y, p.z);
	}
	Point2d reprojectR(double x, double y, double z);
	Point2d reprojectR(const Point3d & p) {
		return reprojectR(p.x, p.y, p.z);
	}

};

#endif
