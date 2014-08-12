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
#include "halo.h"

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
	if (retVal!=0) {
		halo.close();
		return -1;
	}
	
	// Calibrate
	//halo.calib();
	
	// Main loop
	while(true) {

		// Capture everything
		Mat leftNewImageFrame, rightNewImageFrame, depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
		halo.captureAllImages(leftNewImageFrame, rightNewImageFrame, depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, SYNCHRONOUS);
		
		// Display everything
		imshow("Depth", depthNewImageFrame);
		imshow("Intensity", visualNewImageFrame);
		imshow("Confidency", confidenceNewImageFrame);
		imshow("Left", leftNewImageFrame);
		imshow("Right", rightNewImageFrame);
		
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
 	halo.close();
	
	return 0;
}