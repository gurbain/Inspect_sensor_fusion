/*! 
* 	\file    stereoCapture.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to capture and display STEREO images
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

// Project libs
#include "../main.h"
#include "../utils.h"
#include "../defines.h"
#include "../halo.h"

using namespace std;
using namespace cv;


// Allocate variables
Cameras stereo;

// Create a CTRL-C handler
void exit_handler(int s){
	stereo.close();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,exit_handler);
	
	// Initialize
	int retVal = stereo.init();
	if (retVal !=0 )
		exit_handler(0);
	
	// Start the acquisition flow
	retVal = stereo.start();
	if (retVal !=0 )
		exit_handler(0);
	
	// Main loop
	while(true) {
		// Capture stereo images
		Mat iL, iR;
		TimeStamp t;
		int *dummy1, *dummy2, dummy3;
		retVal = stereo.captureTwoImages(iL, iR, dummy1, dummy2, t, dummy3);
		if (retVal !=0 )
			exit_handler(0);
		
		// Display images
		imshow("Left", iL);
		imshow("Right", iR);
		
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
	
	// Stop the acquisition flow
	stereo.stop();
	
 	// Close cameras
  	stereo.close();
	
	return 0;
}