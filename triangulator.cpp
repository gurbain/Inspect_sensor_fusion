/*! 
* 	\file    triangulator.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Triangulation tools for Stereo cams and ORF
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "triangulator.h"

using namespace cv;
using namespace std;


StereoTriangulator::StereoTriangulator(Mat projMatrix_)
{
	projMatrix = projMatrix_;
}

Point3d StereoTriangulator::triangulateStereo(double u1, double u2, double v)
{
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

Point3d StereoTriangulator::triangulateStereo(const Point2i & p1, const Point2i & p2)
{
	return triangulateStereo(p1.x, p2.x, p1.y);
}

Mat StereoTriangulator::triangulateStereoMat(double u1, double u2, double v)
{
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

void StereoTriangulator::reprojectStereo(Point2d& pL, Point2d& pR, double x, double y, double z)
{
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

void StereoTriangulator::reprojectStereo(Point2d& pL, Point2d& pR, Point3d & p) 
{
	reprojectStereo(pL, pR, p.x, p.y, p.z);
	return;
}

StereoTriangulator2::StereoTriangulator2(Mat rotL_, Mat rotR_, Mat intrinsicL_, Mat intrinsicR_) {
	rotR = rotR_;
	rotL = rotL_;
	intrinsicL = intrinsicL_;
	intrinsicR = intrinsicR_;
}

Mat StereoTriangulator2::triangulateStereoMat(Mat& pL, Mat& pR)
{
	Mat point;
	
	Mat projMatrixL = intrinsicL * rotL;
	Mat projMatrixR = intrinsicR * rotR;
	
	triangulatePoints(projMatrixL, projMatrixR, pL, pR, point);
	
	return point;
}

Mat StereoTriangulator2::triangulateStereoLinear(Point2d pL, Point2d pR)
{
	Mat projMatrixL = intrinsicL * rotL;
	Mat projMatrixR = intrinsicR * rotR;
	
	Mat p;
	vconcat(linearTriangulation(pL, projMatrixL, pR, projMatrixR), Mat::ones(1,1,CV_64F), p);
	
	return p;
}

Mat StereoTriangulator2::triangulateStereoIterative(Point2d pL, Point2d pR)
{
	Mat projMatrixL = intrinsicL * rotL;
	Mat projMatrixR = intrinsicR * rotR;
	
	Mat p;
	vconcat(iterativeTriangulation(pL, projMatrixL, pR, projMatrixR), Mat::ones(1,1,CV_64F), p);
	return p;
}

Mat StereoTriangulator2::linearTriangulation(Point2d pL, Mat projMatrixL, Point2d pR, Mat projMatrixR)
{
	Mat A(4, 3, CV_64F);
	A.at<double>(0,0) = pL.x * projMatrixL.at<double>(2,0) - projMatrixL.at<double>(0,0);
	A.at<double>(0,1) = pL.x * projMatrixL.at<double>(2,1) - projMatrixL.at<double>(0,1);
	A.at<double>(0,2) = pL.x * projMatrixL.at<double>(2,2) - projMatrixL.at<double>(0,2);
	A.at<double>(1,0) = pL.y * projMatrixL.at<double>(2,0) - projMatrixL.at<double>(1,0);
	A.at<double>(1,1) = pL.y * projMatrixL.at<double>(2,1) - projMatrixL.at<double>(1,1);
	A.at<double>(1,2) = pL.y * projMatrixL.at<double>(2,2) - projMatrixL.at<double>(1,2);
	A.at<double>(2,0) = pR.x * projMatrixR.at<double>(2,0) - projMatrixR.at<double>(0,0);
	A.at<double>(2,1) = pR.x * projMatrixR.at<double>(2,1) - projMatrixR.at<double>(0,1);
	A.at<double>(2,2) = pR.x * projMatrixR.at<double>(2,2) - projMatrixR.at<double>(0,2);
	A.at<double>(3,0) = pR.y * projMatrixR.at<double>(2,0) - projMatrixR.at<double>(1,0);
	A.at<double>(3,1) = pR.y * projMatrixR.at<double>(2,1) - projMatrixR.at<double>(1,1);
	A.at<double>(3,2) = pR.y * projMatrixR.at<double>(2,2) - projMatrixR.at<double>(1,2);

	Mat B(4, 1, CV_64F);
	B.at<double>(0,0) = pL.x * projMatrixL.at<double>(2,3) + projMatrixL.at<double>(0,3);
	B.at<double>(1,0) = pL.y * projMatrixL.at<double>(2,3) + projMatrixL.at<double>(1,3);
	B.at<double>(2,0) = pR.x * projMatrixR.at<double>(2,3) + projMatrixR.at<double>(0,3);
	B.at<double>(3,0) = pR.y * projMatrixR.at<double>(2,3) + projMatrixR.at<double>(1,3);

	Mat X;
	solve(A,B,X,DECOMP_SVD);
	
	return X;
}

Mat StereoTriangulator2::iterativeTriangulation(Point2d pL, Mat projMatrixL, Point2d pR, Mat projMatrixR)
{
	double wiL = 1, wiR = 1;
	Mat X(4, 1, CV_64F); 
	for (int i=0; i<10; i++) {
		Mat X_ = linearTriangulation(pL, projMatrixL, pR, projMatrixR);
		X.at<double>(0) = X_.at<double>(0);
		X.at<double>(1) = X_.at<double>(1);
		X.at<double>(2) = X_.at<double>(2);
		X.at<double>(3) = 1.0;

		//recalculate weights
		double p2xL =  Mat(projMatrixL.row(2) * X).at<double>(0);
		double p2xR =  Mat(projMatrixR.row(2) * X).at<double>(0);

		//breaking point
		if(fabsf(wiL - p2xL) <= IT_EPSILON && fabsf(wiR - p2xR) <= IT_EPSILON) break;

		wiL = p2xL;
		wiR = p2xR;

		//reweight equations and solve
		Mat A(4, 3, CV_64F);
		A.at<double>(0,0) = (pL.x * projMatrixL.at<double>(2,0) - projMatrixL.at<double>(0,0)) / wiL;
		A.at<double>(0,1) = (pL.x * projMatrixL.at<double>(2,1) - projMatrixL.at<double>(0,1)) / wiL;
		A.at<double>(0,2) = (pL.x * projMatrixL.at<double>(2,2) - projMatrixL.at<double>(0,2)) / wiL;
		A.at<double>(1,0) = (pL.y * projMatrixL.at<double>(2,0) - projMatrixL.at<double>(1,0)) / wiL;
		A.at<double>(1,1) = (pL.y * projMatrixL.at<double>(2,1) - projMatrixL.at<double>(1,1)) / wiL;
		A.at<double>(1,2) = (pL.y * projMatrixL.at<double>(2,2) - projMatrixL.at<double>(1,2)) / wiL;
		A.at<double>(2,0) = (pR.x * projMatrixR.at<double>(2,0) - projMatrixR.at<double>(0,0)) / wiR;
		A.at<double>(2,1) = (pR.x * projMatrixR.at<double>(2,1) - projMatrixR.at<double>(0,1)) / wiR;
		A.at<double>(2,2) = (pR.x * projMatrixR.at<double>(2,2) - projMatrixR.at<double>(0,2)) / wiR;
		A.at<double>(3,0) = (pR.y * projMatrixR.at<double>(2,0) - projMatrixR.at<double>(1,0)) / wiR;
		A.at<double>(3,1) = (pR.y * projMatrixR.at<double>(2,1) - projMatrixR.at<double>(1,1)) / wiR;
		A.at<double>(3,2) = (pR.y * projMatrixR.at<double>(2,2) - projMatrixR.at<double>(1,2)) / wiR;

		Mat B(4, 1, CV_64F);
		B.at<double>(0,0) = (pL.x * projMatrixL.at<double>(2,3) + projMatrixL.at<double>(0,3)) / wiL;
		B.at<double>(1,0) = (pL.y * projMatrixL.at<double>(2,3) + projMatrixL.at<double>(1,3)) / wiL;
		B.at<double>(2,0) = (pR.x * projMatrixR.at<double>(2,3) + projMatrixR.at<double>(0,3)) / wiR;
		B.at<double>(3,0) = (pR.y * projMatrixR.at<double>(2,3) + projMatrixR.at<double>(1,3)) / wiR;
		
		solve(A,B,X_,DECOMP_SVD);
		X.at<double>(0) = X_.at<double>(0);
		X.at<double>(1) = X_.at<double>(1);
		X.at<double>(2) = X_.at<double>(2);
		X.at<double>(3) = 1.0;
	}
	return X;
}

void StereoTriangulator2::reprojectStereo(Point2d& pL_, Point2d& pR_, Point3d p_)
{
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

void StereoTriangulator2::reprojectStereoMat(Point2d& pL_, Point2d& pR_, Mat& p)
{
	Mat pL, pR;
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

OrfTriangulator::OrfTriangulator(Mat intrinsicMatrix_)
{
	intrinsicMatrix = intrinsicMatrix_;
}

Point3d OrfTriangulator::triangulateOrf(double u, double v, double r)
{
	Point3d p(0.0, 0.0, 0.0);
	double f = intrinsicMatrix.at<double>(0,0);
	double cx = intrinsicMatrix.at<double>(0,2);
	double cy = intrinsicMatrix.at<double>(1,2);
	
	p.z = sqrt(pow(r, 2) / ( 1 + ( pow((u - cx), 2) + pow((v - cy), 2)) / pow(f, 2)));
	p.x = (u - cx) * p.z / f;
	p.y = (v - cy) * p.z / f;
	return p;
}

Mat OrfTriangulator::triangulateOrfMat(double u, double v, double r)
{
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

Point3d OrfTriangulator::triangulateSphericalOrf(double u, double v, double z)
{
	Point3d p(0.0, 0.0, 0.0);
	double f = intrinsicMatrix.at<double>(0,0);
	double cx = intrinsicMatrix.at<double>(0,2);
	double cy = intrinsicMatrix.at<double>(1,2);
	
	p.z = z;
	p.x = (u - cx) * z / f;
	p.y = (v - cy) * z / f;
	return p;
}

Point2d OrfTriangulator::reprojectOrf(double x, double y, double z)
{
	double f = intrinsicMatrix.at<double>(0,0);
	double cx = intrinsicMatrix.at<double>(0,2);
	double cy = intrinsicMatrix.at<double>(1,2);
	Point2d p;
	
	p.x = (x * f / z) + cx; //u1
	p.y = (y * f / z) + cy; //v
	return p;
}

Point2d OrfTriangulator::reprojectOrf(const Point3d & p)
{
	return reprojectOrf(p.x, p.y, p.z);
}