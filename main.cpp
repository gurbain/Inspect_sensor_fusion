/*! 
* 	\file    main.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Main program for optical range finder and stereo cameras acquisition
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

// Project libs
#include "utils.h"
#include "defines.h"
#include "camera.h"
#include "orf.h"

using namespace std;
using namespace cv;
using namespace orf;
using namespace cam;


int main(int argc, char **argv) {
	
	intro();

	// Create
	ORF tof;
	//Cameras stereo;
	
	// Initialize
	tof.initOrf();
	//stereo.initTwoCameras();
	

	//cam2.startTwoCameras();
	//namedWindow("a");
	
	int flag;
	while(true) {
		//Mat leftNewImageFrame, rightNewImageFrame;
		
		//cam2.captureTwoImages(leftNewImageFrame, rightNewImageFrame, &cam2.rightImgNum, &cam2.leftImgNum, flag);

		
		// Change parameters
		//changeParams(cam);
		
		// Capture ToF images
		Mat depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
		int retVal = tof.captureOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame);
		if (retVal==-1)
			break;
		imshow("Depth", depthNewImageFrame);
		imshow("Intensity", visualNewImageFrame);
		imshow("Confidency", confidenceNewImageFrame);
		
		// Wait for user to close the software
		int key = waitKey(1);
		if (key == 2) break;
	}

// 	try{
// 		DEBUG<<"arg"<<endl;
// 		UeyeOpencvCam cam1 = UeyeOpencvCam(640,480);
// 		//UeyeOpencvCam cam2 = UeyeOpencvCam(640,480);
// 
// 
// 		while (true) {
// 			cv::imshow("cam1", cam1.getFrame());
// 			cout<<"ok1"<<endl;
// // 			cv::namedWindow("cam2", CV_WINDOW_AUTOSIZE);
// // 			cv::imshow("cam2", cam2.getFrame());
// // 			cout<<"ok2"<<endl;
// 			if (cv::waitKey(1) >= 0) {
// 				break;
// 			}
// 		}
// 	} catch(UeyeOpenCVException& e) {
// 		ERROR << e.what() << endl;
// 	}
	
		return 0;
}