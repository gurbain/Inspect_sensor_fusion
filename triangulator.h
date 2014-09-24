/*! 
* 	\file    triangulator.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Simple triangulation based on Muggler and Tweddle work and sources
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


/* This class allows the user to compute 3D coordinates
 * from two pixels coordinates and the Intrinsic and
 * Extrinsic matrices of the stereo rig
 */
class StereoTriangulator {
	
	Mat projMatrix;
	
public:
	/*
	 * With this constructor, you can do a simple triangulation and simple reconstruction
	 */
	StereoTriangulator(Mat projMatrix_) {
		projMatrix = projMatrix_;
	}

	Point3d triangulateStereo(double u1, double u2, double v) {
		Point3d p(0.0, 0.0, 0.0);
		double f = projMatrix.at<double>(0,0);
		//double Tx = -T.at<double>(0,0); // Warning T is defined from Right to Left
		double cx = projMatrix.at<double>(0,2);
		double cy = projMatrix.at<double>(1,2);
		
		p.z = -projMatrix.at<double>(0,3) / (u1-u2);
		p.x = (u1 - cx) * p.z / f;
		p.y = (v - cy) * p.z / f;
		return p;
	}

	Point3d triangulateStereo(const Point2i & p1, const Point2i & p2) {
		return triangulateStereo(p1.x, p2.x, p1.y);
	}
	
	Mat triangulateStereoMat(double u1, double u2, double v) {
		Mat p = Mat(4, 1, CV_64F);
		double f = projMatrix.at<double>(0,0);
		//double Tx = -T.at<double>(0,0); // Warning T is defined from Right to Left
		double cx = projMatrix.at<double>(0,2);
		double cy = projMatrix.at<double>(1,2);
		
		p.at<double>(0,2) = -projMatrix.at<double>(0,3) / (u1-u2);
		p.at<double>(0,0) = (u1 - cx) * p.at<double>(0,2) / f;
		p.at<double>(0,1) = (v - cy) * p.at<double>(0,2) / f;
		p.at<double>(0,3) = 1;
		
		return p;
	}

	/* Reproject in the L coordinate system considering ideal model where L and R are in the same plane
	 * i.e no rotation matrix and only one coordinate in translation
	 */
	void reprojectStereo(Point2d& pL, Point2d& pR, double x, double y, double z) {
		double f = projMatrix.at<double>(0,0);
		double cx = projMatrix.at<double>(0,2);
		double cy = projMatrix.at<double>(1,2);
		Point2d p;
		
		pL.x = (x * f / z) + cx; //u1
		pL.y = (y * f / z) + cy; //v1
		pR.x = ( projMatrix.at<double>(0,3) / z ) + pL.x; //u2
		pR.y = pL.y; // v2
		return ;
	}

	void reprojectStereo(Point2d& pL, Point2d& pR, Point3d & p) {
		reprojectStereo(pL, pR, p.x, p.y, p.z);
		return;
	}
	
};


class StereoTriangulator2 {
	
	Mat rotL, rotR, intrinsicL, intrinsicR;
	
public:
	
	StereoTriangulator2(Mat rotL_, Mat rotR_, Mat intrinsicL_, Mat intrinsicR_) {
		rotR = rotR_;
		rotL = rotL_;
		intrinsicL = intrinsicL_;
		intrinsicR = intrinsicR_;
	}
	
	Mat triangulateStereoMat(Mat& pL, Mat& pR) {
		Mat point;
		
		Mat projMatrixL = intrinsicL * rotL;
		Mat projMatrixR = intrinsicR * rotR;
		
		triangulatePoints(projMatrixL, projMatrixR, pL, pR, point);
		
		return point;
	}
	
	Mat triangulateStereo(Point2d& pL_, Point2d& pR_) {
		Mat point;
		
		Mat projMatrixL = intrinsicL * rotL;
		Mat projMatrixR = intrinsicR * rotR;
		
		Mat pL = Mat(2,1,CV_64F);
		Mat pR = Mat(2,1,CV_64F);
		pL.at<double>(0) = pL_.x;
		pL.at<double>(0) = pL_.y;
		pR.at<double>(0) = pR_.x;
		pR.at<double>(0) = pR_.y;
		
		triangulatePoints(projMatrixL, projMatrixR, pL, pR, point);
		
		return point;
	}
	
	void reprojectStereo(Point2d& pL_, Point2d& pR_, Point3d & p_) {
		Mat pL, pR;
		Mat p = Mat::zeros(4, 1, CV_64F);
		p.at<double>(0, 0) =  p_.x;
		p.at<double>(0, 1) =  p_.y;
		p.at<double>(0, 2) =  p_.z;
		p.at<double>(0, 3) =  1;
		
		pL = intrinsicL * rotL * p;
		pR = intrinsicR * rotR * p;
		
		pL = pL/pL.at<double>(2);
		pR = pR/pR.at<double>(2);
		
		pL_.x = (int)pL.at<double>(0, 0);
		pL_.y = (int)pL.at<double>(0, 1);
		pR_.x = (int)pR.at<double>(0, 0);
		pR_.y = (int)pR.at<double>(0, 1);
		
		return;
	}
	
};


/* This class allows the user to compute 3D coordinates
 * from one pixel coordinate and the depth provided
 * by the ORF along with the Intrinsic Matrix of 
 * this camera.
 */
class OrfTriangulator {
	
	Mat intrinsicMatrix;
	
public:
	OrfTriangulator(Mat intrinsicMatrix_) {
		intrinsicMatrix = intrinsicMatrix_;
	}
	
	Point3d triangulateOrf(double u, double v, double r) {
		Point3d p(0.0, 0.0, 0.0);
		double f = intrinsicMatrix.at<double>(0,0);
		double cx = intrinsicMatrix.at<double>(0,2);
		double cy = intrinsicMatrix.at<double>(1,2);
		
		p.z = sqrt(pow(r, 2) / ( 1 + ( pow((u - cx), 2) + pow((v - cy), 2)) / pow(f, 2)));
		p.x = (u - cx) * p.z / f;
		p.y = (v - cy) * p.z / f;
		return p;
	}
	
	Mat triangulateOrfMat(double u, double v, double r) {
		Mat p = Mat(4, 1, CV_64F);
		double f = intrinsicMatrix.at<double>(0,0);
		double cx = intrinsicMatrix.at<double>(0,2);
		double cy = intrinsicMatrix.at<double>(1,2);
		
		p.at<double>(0,2) = sqrt(pow(r, 2) / ( 1 + ( pow((u - cx), 2) + pow((v - cy), 2)) / pow(f, 2)));
		p.at<double>(0,0) = (u - cx) * p.at<double>(0,2) / f;
		p.at<double>(0,1) = (v - cy) * p.at<double>(0,2) / f;
		p.at<double>(0,3) = 1;
		
		return p;
	}
	
	Point3d triangulateSphericalOrf(double u, double v, double z) {
		Point3d p(0.0, 0.0, 0.0);
		double f = intrinsicMatrix.at<double>(0,0);
		double cx = intrinsicMatrix.at<double>(0,2);
		double cy = intrinsicMatrix.at<double>(1,2);
		
		p.z = z;
		p.x = (u - cx) * z / f;
		p.y = (v - cy) * z / f;
		return p;
	}

	Point2d reprojectOrf(double x, double y, double z) {
		double f = intrinsicMatrix.at<double>(0,0);
		double cx = intrinsicMatrix.at<double>(0,2);
		double cy = intrinsicMatrix.at<double>(1,2);
		Point2d p;
		
		p.x = (x * f / z) + cx; //u1
		p.y = (y * f / z) + cy; //v
		return p;
	}

	Point2d reprojectOrf(const Point3d & p) {
		return reprojectOrf(p.x, p.y, p.z);
	}
};

#endif
