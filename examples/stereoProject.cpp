/*! 
* 	\file    haloCapture.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to project and triangulate point in stereo
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
	Mat intrinsicL, intrinsicR, projMatrixStereo;
	FileStorage storage;
	int retVal = storage.open("OM_calib.xml", FileStorage::READ);
	if (retVal != 1) {
		INFO<<"STEREO Calibration file not found! Calibration needed!"<<endl;
		exit(1);
	}
	storage["intrinsicL"]>>intrinsicL;
	storage["intrinsicR"]>>intrinsicR;
	storage["projMatrixR"]>>projMatrixStereo;
	storage.release();
	
	// Create triangulator object
	StereoTriangulator stereo(projMatrixStereo);
	
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
		stereo.reprojectStereo(coordL, coordR, p_init[i].x*1000, p_init[i].y*1000, p_init[i].z*1000);
		p_inter_L.push_back(coordL);
		p_inter_R.push_back(coordR);
	}
	
	// -- STEP 2 : REPROJECT --
	// Reproject back to real world in L coordinate system (in meters)
	// WARNING : triangulateStereo gives coorinates in mm, we have to convert afterwards
	vector<Point3d> p_final;
	vector<Vec3b> rgb_final;
	
	for (int i=0; i<p_init.size(); i++) {
		Point3d coord;
		coord = stereo.triangulateStereo((double)p_inter_L[i].x, (double)p_inter_R[i].x, (double)p_inter_L[i].y);
		p_final.push_back(Point3d(coord.x/1000, coord.y/1000, coord.z/1000));
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