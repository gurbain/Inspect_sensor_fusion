/*! 
* 	\file    main.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    June 2014
* 	\version 0.1
* 	\brief   Main program
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/
/*
// Project libs
#include "main.h"
#include "utils.h"
#include "defines.h"
#include "orf.h"

using namespace std;
using namespace cv;

// Allocate variables
ORF orf;

// Create a CTRL-C handler
void my_handler(int s){
	orf.closeOrf();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,my_handler);
	
	// Initialize
	int retVal = orf.initOrf();
	
	// Main loop
	int num = 0;
	while(true) {

		// Capture everything
		Mat leftNewImageFrame, rightNewImageFrame, depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
		//halo.captureAllImages(leftNewImageFrame, rightNewImageFrame, depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, SYNCHRONOUS);
		//retVal = halo.loadAllImages(leftNewImageFrame, rightNewImageFrame, depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, SYNCHRONOUS);
		TimeStamp ts;
		retVal = orf.captureOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, ts);
		if (retVal==-1)
			break;
		//INFO<<"PROC: "<<ts.getProcTime()<<endl;
		
		//DEBUG<<"Depth: "<<depthNewImageFrame.depth()<<" et "<<depthNewImageFrame.channels()<<endl;
		double result = ((depthNewImageFrame.at<unsigned short>(71, 87)>>2) & 0x3FFF)  * 0.00061;
		cout<<"Result: "<<result<<endl;

		// Display everything
		imshow("Depth", depthNewImageFrame);
		//imshow("Intensity", visualNewImageFrame);
		imshow("Confidency", confidenceNewImageFrame);
// 		imshow("Left", leftNewImageFrame);
// 		imshow("Right", rightNewImageFrame);
// 		
// 		usleep(20000);
// 		num++;
		
		// Handle pause/unpause and ESC
		int c = cvWaitKey(1);
		if(c == 'p') {
			DEBUG<<"Acquisition is now paused"<<endl;
			c = 0;
			while(c != 'p' && c != 27){
				c = cvWaitKey(250);
			}
			DEBUG<<"Acquisition is now unpaused"<<endl;
		}
		if(c == 27) {
			DEBUG<<"Acquisition has been stopped by user"<<endl;
			break;
		}
 	}
 	
 	// Close cameras
	orf.closeOrf();
	
	return 0;
}*/


// // Project libs
// #include "main.h"
// #include "utils.h"
// #include "defines.h"
// #include "halo.h"
// 
// using namespace std;
// using namespace cv;
// 
// 
// // Allocate variables
// ORF orf;
// 
// // Create a CTRL-C handler
// void keyboard_handle(int s) {
// 	orf.close();
// 	exit(1);
// }
// 
// // Create a mouse clicks handler
// int x_img, y_img;
// void mouse_handle(int event, int x, int y, int flags, void* userdata) {
// 	if  ( event == EVENT_LBUTTONDOWN ) {
// 		x_img = x;
// 		y_img = y;
// 	}
// }
// 
// // Main program
// int main(int argc, char **argv) {
// 	
// 	init();
// 	
// 	// For catching a CTRL-C
// 	signal(SIGINT,keyboard_handle);
// 	
// 	// Initialize
// 	orf.init();
// 	
// 	// Initialize mouse clicks
// 	x_img = 0;
// 	y_img = 0;
// 	namedWindow("Depth", 1);
// 	namedWindow("Intensity", 1);
// 	namedWindow("Confidency", 1);
// 	setMouseCallback("Depth", mouse_handle, NULL);
// 	setMouseCallback("Intensity", mouse_handle, NULL);
// 	setMouseCallback("Confidency", mouse_handle, NULL);
// 	
// 	// Main loop
// 	while(true) {
// 		// Capture orf images
// 		Mat dT, iT, cT;
// 		TimeStamp t;
// 		orf.captureRectifiedOrf(dT, iT, cT, t);		
// 		
// 		// Add circle and text on images
// 		Point p(x_img, y_img);
// 		char buffer[15];
// 		sprintf(buffer, "%f", ((dT.at<unsigned short>(p.y, p.x)>>2) & 0x3FFF)*0.00061);
// 		string disp = String("Distance from this point: ") + buffer;
// 		Mat dT_txt(dT.size(), CV_8UC3), iT_txt(iT.size(), CV_8UC3), cT_txt(cT.size(), CV_8UC3);
// 		cvtColor(dT.clone(), dT_txt, CV_GRAY2RGB);
// 		cvtColor(iT, iT_txt, CV_GRAY2RGB);
// 		cvtColor(cT, cT_txt, CV_GRAY2RGB);
// 		circle (dT_txt, p, 5, Scalar(0, 0, 255), 3);
// 		circle (iT_txt, p, 5, Scalar(0, 0, 255), 3);
// 		circle (cT_txt, p, 5, Scalar(0, 0, 255), 3);
// 		putText(dT_txt, disp, Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 0, 0), 1, CV_AA);
// 		putText(iT_txt, disp, Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 0, 0), 1, CV_AA);
// 		putText(cT_txt, disp, Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 0, 0), 1, CV_AA);
// 		
// 		// Display images
// 		imshow("Depth", dT_txt);
// 		imshow("Intensity", iT_txt);
// 		imshow("Confidency", cT_txt);
// 		
// 		// Handle pause/unpause and ESC
// 		int c = cvWaitKey(1);
// 		if(c == 'p') {
// 			DEBUG<<"Acquisition is now paused"<<endl;
// 			c = 0;
// 			while(c != 'p' && c != 27){
// 				c = cvWaitKey(250);
// 			}
// 			DEBUG<<"Acquisition is now unpaused"<<endl;
// 		}
// 		if(c == 27) {
// 			DEBUG<<"Acquisition has been stopped by user"<<endl;
// 			break;
// 		}
// 	}
// 	
//  	// Close camera
//   	orf.close();
// 	
// 	return 0;
// }

// Project libs
#include "main.h"
#include "utils.h"
#include "defines.h"
#include "halo.h"/*

using namespace std;
using namespace cv;


// Allocate variables
ORF orf;

// Create a CTRL-C handler
void my_handler(int s){
	WaitForVisualizationThread();
	orf.close();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,my_handler);
	
	// Initialize
	orf.init();
	
	// Launch vizualization (you can give the pixel dimensions)
	RunVisualizationThread(2);
	
	// Main loop
	while(true) {
		// Capture cloud
		vector<Point3d> pointcloud;
		vector<Vec3b> rgbcloud;
		orf.capture3Dcloud(pointcloud, rgbcloud);
		
		// Display cloud
		ShowClouds(pointcloud, rgbcloud);
	}
	
	// Wait for updating vizualization
	WaitForVisualizationThread();
	
 	// Close camera
  	orf.close();
	
	return 0;
}*/

// using namespace std;
// using namespace cv;
// 
// 
// // Allocate variables
// Halo halo;
// 
// // Create a CTRL-C handler
// void exit_handler(int s){
// 	WaitForVisualizationThread();
// 	halo.close();
// 	exit(1);
// }
// 
// int main(int argc, char **argv) {
// 	
// 	init();
// 	
// 	// For catching a CTRL-C
// 	signal(SIGINT,exit_handler);
// 	
// 	// Initialize
// 	int retVal = halo.init("../examples/image_set_chessboard");
// 	if (retVal!=0)
// 		exit_handler(0);
// 	
// 	// Launch vizualization (you can give the pixel dimensions)
// 	RunVisualizationThread(3);
// 		
// 	// Main loop
// 	while(true) {
// 		// Capture cloud
// 		vector<Point3d> pointcloud;
// 		vector<Vec3b> rgbcloud;
// 		halo.capture3Dcloud(pointcloud, rgbcloud);
// 		
// 		// Display cloud
// 		ShowClouds(pointcloud, rgbcloud);
// 		
// 		// Wait between each cloud
// 		usleep(600000);
// 	}
// 
// 	// Wait for updating vizualization
// 	WaitForVisualizationThread();
// 	
// 	// Close cameras
// 	halo.close();
// 	
// 	return 0;
// }


// // Project libs
// #include "main.h"
// #include "utils.h"
// #include "defines.h"
// #include "halo.h"
// 
using namespace std;
using namespace cv;


// Allocate variables
Halo halo;

// Create a CTRL-C handler
void my_handler(int s){
	halo.close();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,my_handler);
	
	// Initialize
	int retVal = halo.init();
	
	// Calibrate
	halo.calib();

 	// Close cameras
  	halo.close();
	
	return 0;
}



// // Project libs
// #include "main.h"
// #include "utils.h"
// #include "defines.h"
// #include "camera.h"
// halo
// #include "opencv2/core/core.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include "opencv2/contrib/contrib.hpp"
// #include <stdio.h>
// 
// using namespace cv;
// using namespace std;
// 
// int main(int argc, char* argv[])
// {
//     Cameras cam;
//     cam.initTwoCameras("/home/gabs48/results/13-08-2014-07-49");
//     int numBoards = 15;
//     int board_w = 6;
//     int board_h = 11;
// 
//     Size board_sz = Size(board_w, board_h);
//     int board_n = board_w*board_h;
// 
//     vector<vector<Point3f> > object_points;
//     vector<vector<Point2f> > imagePoints1, imagePoints2;
//     vector<Point2f> corners1, corners2;
// 
//     vector<Point3f> obj;
//     for (int j=0; j<board_n; j++)
//     {
//         obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
//     }
// 
//     Mat img1, img2, gray1, gray2;
// 
//     int success = 0, k = 0;
//     bool found1 = false, found2 = false;
//     
//     int *img_num1, *img_num2;
//     TimeStamp ts;
//     int flags = 0;
//     int num = 0;
// 
//     while (success < numBoards)
//     {
//         cam.captureTwoImages(img1, img2, img_num1, img_num2, ts, flags, num);
//         //resize(img1, img1, Size(320, 280));
//         //resize(img2, img2, Size(320, 280));
//         cvtColor(img1, gray1, CV_BGR2GRAY);
//         cvtColor(img2, gray2, CV_BGR2GRAY);
// 
//         found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
//         found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
// 
//         if (found1)
//         {
//             cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
//             drawChessboardCorners(gray1, board_sz, corners1, found1);
//         }
// 
//         if (found2)
//         {
//             cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
//             drawChessboardCorners(gray2, board_sz, corners2, found2);
//         }
//         
//         imshow("image1", gray1);
//         imshow("image2", gray2);
// 
//         k = waitKey(10);
//         if (found1 && found2)
//         {
//             k = waitKey(0);
//         }
//         if (k == 27)
//         {
//             break;
//         }
//         if (k == ' ' && found1 !=0 && found2 != 0)
//         {
//             imagePoints1.push_back(corners1);
//             imagePoints2.push_back(corners2);
//             object_points.push_back(obj);
//             printf ("Corners stored\n");
//             success++;
// 
//             if (success >= numBoards)
//             {
//                 break;
//             }
//         }
//         num++;
//     }
// 
//     destroyAllWindows();
//     printf("Starting Calibration\n");
//     Mat CM1 = Mat(3, 3, CV_64FC1);
//     Mat CM2 = Mat(3, 3, CV_64FC1);
//     Mat D1, D2;
//     Mat R, T, E, F;
// 
//     stereoCalibrate(object_points, imagePoints1, imagePoints2, 
//                     CM1, D1, CM2, D2, img1.size(), R, T, E, F,
//                     cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
//                     CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);
// 
//     FileStorage fs1("mystereocalib.yml", FileStorage::WRITE);
//     fs1 << "CM1" << CM1;
//     fs1 << "CM2" << CM2;
//     fs1 << "D1" << D1;
//     fs1 << "D2" << D2;
//     fs1 << "R" << R;
//     fs1 << "T" << T;
//     fs1 << "E" << E;
//     fs1 << "F" << F;
// 
//     printf("Done Calibration\n");
// 
//     printf("Starting Rectification\n");
// 
//     Mat R1, R2, P1, P2, Q;
//     stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
//     fs1 << "R1" << R1;
//     fs1 << "R2" << R2;
//     fs1 << "P1" << P1;
//     fs1 << "P2" << P2;
//     fs1 << "Q" << Q;
// 
//     printf("Done Rectification\n");
// 
//     printf("Applying Undistort\n");
// 
//     Mat map1x, map1y, map2x, map2y;
//     Mat imgU1, imgU2;
// 
//     initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
//     initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_32FC1, map2x, map2y);
// 
//     printf("Undistort complete\n");
// 
//     num = 0;
//     while(1)
//     {    
//         cam.captureTwoImages(img1, img2, img_num1, img_num2, ts, flags, num);
// 
//         remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
//         remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
// 
//         imshow("image1", imgU1);
//         imshow("image2", imgU2);
// 
//         k = waitKey(5);
// 
//         if(k==27)
//         {
//             break;
//         }
//         num++;
// 	usleep(100000);
//     }
//     
//     cam.closeTwoCameras();
// 
//     return(0);
// }