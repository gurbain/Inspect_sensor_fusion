/*! 
* 	\file    haloCapture.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to project XYZ from ORF to stereo images then reproject and move them to XYZ ORF coordinate system again
*
* 	This example shows how to convert a point from its XYZ metric coordinates in the TOF camera frame
* 	to the left camera frame, then project it to its uv pixel coordinates in both L and R with the pinhole equation.
* 	Next, to reproject it from its pixel coordinates to its XYZ metric coordinates in the left camera
* 	frame by triangulation and finally from this coordinate to the T coordinate system.
* 
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
	Mat projMatrixStereo, MLT, MTL;
	FileStorage storage;
	int retVal = storage.open("OM_calib.xml", FileStorage::READ);
	if (retVal != 1) {
		INFO<<"STEREO Calibration file not found! Calibration needed!"<<endl;
		exit(1);
	}
	storage["projMatrixR"]>>projMatrixStereo;
	storage.release();
	
	// Get Halo calibration matrices
	retVal = storage.open("HALO_calib.xml", FileStorage::READ);
	if (retVal != 1) {
		INFO<<"HALO Calibration file not found! Calibration needed!"<<endl;
		exit(1);
	}
	storage["MLT"]>>MLT;
	storage["MTL"]>>MTL;
	storage.release();
	
	// Create triangulator object
	StereoTriangulator stereo(projMatrixStereo);
	
	// Create orginal world pointcloud in T coordinate system (in meters)
	vector<Point3d> p_init_T;
	vector<Vec3b> rgb_init;
	
	p_init_T.push_back(Point3d(1, 1, 5));
	rgb_init.push_back(Vec3b(255,0,0));
	p_init_T.push_back(Point3d(0.2, 0.2, 0.5));
	rgb_init.push_back(Vec3b(255,0,0));
	p_init_T.push_back(Point3d(0.5, 0, 3));
	rgb_init.push_back(Vec3b(255,0,0));
	p_init_T.push_back(Point3d(0, 0, 10));
	rgb_init.push_back(Vec3b(255,0,0));
	
	// -- STEP 1 : MOVE P from T coordinates to L coordinates --
	vector<Point3d> p_init_L;
	
	for (int i=0; i<p_init_T.size(); i++) {
		Mat p_init_T_mat = Mat(p_init_T[i]).reshape(1);
		Mat a = Mat::ones(1,1,p_init_T_mat.type());
		p_init_T_mat.push_back(a);
		Mat p_init_L_mat = MLT * p_init_T_mat;
		p_init_L.push_back(Point3d(p_init_L_mat.at<double>(0,0), p_init_L_mat.at<double>(0,1), p_init_L_mat.at<double>(0,2)));
	}
	
	// -- STEP 2 : PROJECT --
	// Project each point to camera coordinates (in pixels)
	// WARNING : reprojectStereo takes coordinates in mm, we have to convert them
	vector<Point2d> p_inter_L, p_inter_R;
	
	for (int i=0; i<p_init_L.size(); i++) {
		Point2d coordL, coordR;
		stereo.reprojectStereo(coordL, coordR, p_init_L[i].x*1000, p_init_L[i].y*1000, p_init_L[i].z*1000);
		p_inter_L.push_back(coordL);
		p_inter_R.push_back(coordR);
	}
	
	// -- STEP 3 : REPROJECT --
	// Reproject back to real world in L coordinate system (in meters)
	// WARNING : triangulateStereo gives coorinates in mm, we have to convert afterwards
	vector<Point3d> p_final_L;
	
	for (int i=0; i<p_init_T.size(); i++) {
		Point3d coord;
		coord = stereo.triangulateStereo((double)p_inter_L[i].x, (double)p_inter_R[i].x, (double)p_inter_L[i].y);
		p_final_L.push_back(Point3d(coord.x/1000, coord.y/1000, coord.z/1000));
	}
	
	// -- STEP 4 : MOVE P from L coordinates to T coordinates --
	vector<Point3d> p_final_T;
	vector<Vec3b> rgb_final;
	
	for (int i=0; i<p_init_T.size(); i++) {
		Mat p_final_L_mat = Mat(p_final_L[i]).reshape(1);
		Mat a = Mat::ones(1,1,p_final_L_mat.type());
		p_final_L_mat.push_back(a);
		Mat p_final_T_mat = MTL * p_final_L_mat;
		p_final_T.push_back(Point3d(p_final_T_mat.at<double>(0,0), p_final_T_mat.at<double>(0,1), p_final_T_mat.at<double>(0,2)));
		rgb_final.push_back(Vec3b(0,255,0));
	}	
	
	// -- STEP 5 : DISPLAY --
	// Print text
	for (int i=0; i<p_init_T.size(); i++) {
		cout.precision(3);
		cout<<"Point "<<i+1<<":\tOriginal (X;Y;Z) \tin T: ("<<fixed<<p_init_T[i].x<<";"<<fixed<<p_init_T[i].y<<";"<<fixed<<p_init_T[i].z<<")  \tand in L: ("<<fixed<<p_init_L[i].x<<";"<<fixed<<p_init_L[i].y<<";"<<fixed<<p_init_L[i].z<<")"<<endl;
		cout<<"\t\tIntermediate (u;v) \tin L: ("<<fixed<<(int)p_inter_L[i].x<<";"<<fixed<<(int)p_inter_L[i].y<<") \t\tand in R  ("<<fixed<<(int)p_inter_R[i].x<<";"<<fixed<<(int)p_inter_R[i].y<<")"<<endl;
		cout<<"\t\tFinal (X;Y;Z) \t\tin L: ("<<fixed<<p_final_L[i].x<<";"<<fixed<<p_final_L[i].y<<";"<<fixed<<p_final_L[i].z<<") \tand in T: ("<<fixed<<p_final_T[i].x<<";"<<fixed<<p_final_T[i].y<<";"<<fixed<<p_final_T[i].z<<")"<<endl;
	}
	
	// Display images
	Mat img = Mat::zeros(480, 640, CV_8UC3);
	
	for (int i=0; i<p_init_T.size(); i++) {
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
	
	p_tot.insert(p_tot.end(), p_init_T.begin(), p_init_T.end());
	p_tot.insert(p_tot.end(), p_final_T.begin(), p_final_T.end());
	rgb_tot.insert(rgb_tot.end(), rgb_init.begin(), rgb_init.end());
	rgb_tot.insert(rgb_tot.end(), rgb_final.begin(), rgb_final.end());
	RunVisualization(p_tot, rgb_tot);
	
	return 0;
}