/*! 
* 	\file    triangulator.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Triangulation tools for Stereo cams and ORF
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef TRIANGULATOR_HH_
#define TRIANGULATOR_HH_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "defines.h"

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
#include <math.h>

using namespace cv;
using namespace std;


/* This class provides SIMPLE tools for projection
 * and triangulation of 3D stereo cameras
 * It has been adapted from Muggler and Tweddle
 * For more information, try the examples or read Gabriel
 * Urbain sensor fusion work
 */
class StereoTriangulator {
	
	Mat projMatrix;
	
public:
	StereoTriangulator(Mat projMatrix_);
	Point3d triangulateStereo(double u1, double u2, double v);
	Point3d triangulateStereo(const Point2i & p1, const Point2i & p2);
	Mat triangulateStereoMat(double u1, double u2, double v);
	void reprojectStereo(Point2d& pL, Point2d& pR, double x, double y, double z);
	void reprojectStereo(Point2d& pL, Point2d& pR, Point3d & p);
};

/* This class provides more realistic tools for projection
 * and triangulation of 3D stereo cameras
 * It has been adapted from 
 * http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/
 * For more information, try the examples or read Gabriel
 * Urbain sensor fusion work
 */
class StereoTriangulator2 {
	
	Mat rotL, rotR, intrinsicL, intrinsicR;
	
private:
	Mat linearTriangulation(Point2d pL, Mat projMatrixL, Point2d pR, Mat projMatrixR);
	Mat iterativeTriangulation(Point2d pL, Mat projMatrixL, Point2d pR, Mat projMatrixR);
public:
	StereoTriangulator2(Mat rotL_, Mat rotR_, Mat intrinsicL_, Mat intrinsicR_);
	Mat triangulateStereoMat(Mat& pL, Mat& pR);
	Mat triangulateStereoLinear(Point2d pL, Point2d pR);
	Mat triangulateStereoIterative(Point2d pL, Point2d pR);
	void reprojectStereo(Point2d& pL_, Point2d& pR_, Point3d p_);
	void reprojectStereoMat(Point2d& pL_, Point2d& pR_, Mat& p);
};


/* This class helps the user to compute 3D coordinates
 * from one pixel coordinate and the depth provided
 * by the ORF along with the Intrinsic Matrix of 
 * this camera.
 */
class OrfTriangulator {
	
	Mat intrinsicMatrix;
	
public:
	OrfTriangulator(Mat intrinsicMatrix_);
	Point3d triangulateOrf(double u, double v, double r);
	Mat triangulateOrfMat(double u, double v, double r);
	Point3d triangulateSphericalOrf(double u, double v, double z);
	Point2d reprojectOrf(double x, double y, double z);
	Point2d reprojectOrf(const Point3d & p);
};

#endif
