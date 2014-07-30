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
#include "main.h"
#include "utils.h"
#include "defines.h"
#include "camera.h"
#include "orf.h"

using namespace std;
using namespace cv;
using namespace orf;
using namespace cam;


int main(int argc, char **argv) {
	
	init();

	// Allocate variables
	ORF tof;
	Cameras stereo;
	
	// Initialize
	tof.initOrf();
	int retVal = stereo.initTwoCameras();
	if (retVal!=0) {
		tof.closeOrf();
		stereo.closeTwoCameras();
		return -1;
	}
	retVal = stereo.startTwoCameras();
	if (retVal!=0) {
		tof.closeOrf();
		stereo.closeTwoCameras();;
		return -1;
	}
	
	// Main loop
	int flag = 0;
	while(true) {
		
		// Create TimeStamp
		TimeStamp ts, tsr;
		ts.start();
		
		Mat leftNewImageFrame, rightNewImageFrame;
		retVal = stereo.captureTwoImages(leftNewImageFrame, rightNewImageFrame, &stereo.rightImgNum, &stereo.leftImgNum, flag);
		if (retVal!=0)
			break;

		
		// Capture non rectified ToF images
		Mat depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
		retVal = tof.captureRectifiedOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, tsr);
		if (retVal==-1)
			break;
		ts.stop();
		
		// Display everything
		imshow("Depth", depthNewImageFrame);
		imshow("Intensity", visualNewImageFrame);
		imshow("Confidency", confidenceNewImageFrame);
		imshow("Left", leftNewImageFrame);
		imshow("Right", rightNewImageFrame);

		// Display timeStamps
		INFO<<"Time stamp: "<<tsr.getProcTime()<<"ms of processing at "<<tsr.getMeanTime()<<"ms"<<endl;
		
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
 	tof.closeOrf();
 	stereo.closeTwoCameras();
	
	return 0;
}