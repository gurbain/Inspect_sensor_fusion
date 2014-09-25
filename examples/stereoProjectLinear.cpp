/*! 
* 	\file    haloCapture.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to project and triangulate point with a linear method for STEREO cameras
*
* 	This example shows how to project a point from its XYZ metric coordinates in the left camera frame
* 	to its uv pixel coordinates in both L and R with the pinhole equation.
* 	Then, to reproject it from its pixel coordinates to its XYZ metric coordinates in the left camera
* 	frame by triangulation.
* 	We use a simplified model where camera are assumed perfectly aligned in Y since we use OpenCV projection
* 	matrix (http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify)
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

// Project libs
#include "../main.h"
#include "../triangulator.h"
#include "../visualization.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	
	// -- STEP 0 : INITIALIZE --
	init();
	
	// Get stereo calibration matrices
	Mat intrinsicMatrixL, intrinsicMatrixR, R, T;
	FileStorage storage;
	int retVal = storage.open("OM_calib.xml", FileStorage::READ);
	if (retVal != 1) {
		INFO<<"STEREO Calibration file not found! Calibration needed!"<<endl;
		exit(1);
	}
	storage["intrinsicMatrixL"]>>intrinsicMatrixL;
	storage["intrinsicMatrixR"]>>intrinsicMatrixR;
	storage["R"]>>R;
	storage["T"]>>T;
	storage.release();
	Mat rotMatrixL = Mat::zeros(3,4, CV_64F);
	rotMatrixL.at<double>(0,0) = 1;
	rotMatrixL.at<double>(1,1) = 1;
	rotMatrixL.at<double>(2,2) = 1;
	Mat rotMatrixR = Mat::zeros(3,4, CV_64F);
	rotMatrixR.at<double>(0,0) = R.at<double>(0,0);
	rotMatrixR.at<double>(0,1) = R.at<double>(0,1);
	rotMatrixR.at<double>(0,2) = R.at<double>(0,2);
	rotMatrixR.at<double>(0,3) = T.at<double>(0);
	rotMatrixR.at<double>(1,0) = R.at<double>(1,0);
	rotMatrixR.at<double>(1,1) = R.at<double>(1,1);
	rotMatrixR.at<double>(1,2) = R.at<double>(1,2);
	rotMatrixR.at<double>(1,3) = T.at<double>(1);
	rotMatrixR.at<double>(2,0) = R.at<double>(2,0);
	rotMatrixR.at<double>(2,1) = R.at<double>(2,1);
	rotMatrixR.at<double>(2,2) = R.at<double>(2,2);
	rotMatrixR.at<double>(2,3) = T.at<double>(2);
	
	// Create triangulator object
	StereoTriangulator2 stereo(rotMatrixL, rotMatrixR, intrinsicMatrixL, intrinsicMatrixR);
	
	// Create orginal world pointcloud in L coordinate system (in meters)
	vector<Point3d> p_init;
	vector<Vec3b> rgb_init;
	
	p_init.push_back(Point3d(1, 1, 5));
	rgb_init.push_back(Vec3b(255,0,0));
	p_init.push_back(Point3d(0.2, 0.2, 0.5));
	rgb_init.push_back(Vec3b(255,0,0));
	p_init.push_back(Point3d(0.5, 0, 3));
	rgb_init.push_back(Vec3b(255,0,0));
	p_init.push_back(Point3d(0, 0, 10));
	rgb_init.push_back(Vec3b(255,0,0));
	
	// -- STEP 1 : PROJECT --
	// Project each point to camera coordinates (in pixels)
	// WARNING : reprojectStereo takes coorinates in mm, we have to convert them
	vector<Point2d> p_inter_L, p_inter_R;
	
	for (int i=0; i<p_init.size(); i++) {
		Point2d coordL, coordR;
		Point3d p(p_init[i].x * 1000, p_init[i].y * 1000, p_init[i].z * 1000);
		stereo.reprojectStereo(coordL, coordR, p);
		p_inter_L.push_back(coordL);
		p_inter_R.push_back(coordR);
	}
	
	// -- STEP 2 : REPROJECT --
	// Reproject back to real world in L coordinate system (in meters)
	// WARNING : triangulateStereo gives coorinates in mm, we have to convert afterwards
	vector<Point3d> p_final;
	vector<Vec3b> rgb_final;
	
	for (int i=0; i<p_init.size(); i++) {
		Mat p_inter_L_Mat = Mat::ones(3, 1, CV_64F);
		Mat p_inter_R_Mat = Mat::ones(3, 1, CV_64F);
		p_inter_L_Mat.at<double>(0) = p_inter_L[i].x;
		p_inter_L_Mat.at<double>(1) = p_inter_L[i].y;
		p_inter_R_Mat.at<double>(0) = p_inter_R[i].x;
		p_inter_R_Mat.at<double>(1) = p_inter_R[i].y;
		
		Mat coord = stereo.triangulateStereoLinear(p_inter_L[i], p_inter_R[i]);
		p_final.push_back(Point3d(coord.at<double>(0)/1000, coord.at<double>(1)/1000, coord.at<double>(2)/1000));
		rgb_final.push_back(Vec3b(0,255,0));
	}	
	
	// -- STEP 3 : DISPLAY --
	// Print text
	for (int i=0; i<p_init.size(); i++) {
		cout<<"Point "<<i+1<<": Original (X;Y;Z) in L: ("<<p_init[i].x<<";"<<p_init[i].y<<";"<<p_init[i].z<<")  \tIntermediate (u;v) in L and R: ("<<(int)p_inter_L[i].x<<";"<<(int)p_inter_L[i].y<<") ("<<(int)p_inter_R[i].x<<";"<<(int)p_inter_R[i].y<<")\tFinal (X;Y;Z) in L: ("<<p_final[i].x<<";"<<p_final[i].y<<";"<<p_final[i].z<<")"<<endl;
	}
	
	// Display images
	Mat img = Mat::zeros(480, 640, CV_8UC3);
	
	for (int i=0; i<p_init.size(); i++) {
		circle(img, p_inter_L[i], 4, Scalar(255, 0, 0));
		circle(img, p_inter_R[i], 4, Scalar(0, 255, 0));
	}
	imshow("Left (blue) and Right (green)", img);
	cvWaitKey(0);
	// We can see that the point in the bottom right corner is very near the cameras
	// cause the u coordinates are very different. It's not the case for the other one.
	
	// Display 3D point
	vector<Point3d> p_tot;
	vector<Vec3b> rgb_tot;
	
	p_tot.insert(p_tot.end(), p_init.begin(), p_init.end());
	p_tot.insert(p_tot.end(), p_final.begin(), p_final.end());
	rgb_tot.insert(rgb_tot.end(), rgb_init.begin(), rgb_init.end());
	rgb_tot.insert(rgb_tot.end(), rgb_final.begin(), rgb_final.end());
	RunVisualization(p_tot, rgb_tot);
	
	return 0;
}