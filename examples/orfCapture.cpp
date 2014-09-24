/*! 
* 	\file    orfCapture.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to capture and display ORF images
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
ORF orf;

// Create a CTRL-C handler
void exit_handler(int s){
	orf.close();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,exit_handler);
	
	// Initialize
	int retVal = orf.init();
	if (retVal !=0 )
		exit_handler(0);
	
	// Main loop
	while(true) {
		// Capture orf rectified images
		Mat dT, iT, cT;
		TimeStamp t;
		retVal = orf.captureRectifiedOrf(dT, iT, cT, t);
		if (retVal !=0 )
			exit_handler(0);
		
		// Capture orf images
		Mat dTr, iTr, cTr;
		TimeStamp tr;
		retVal = orf.captureOrf(dTr, iTr, cTr, tr);
		if (retVal !=0 )
			exit_handler(0);
		
		// Display images
		imshow("Depth Rectified", dTr);
		imshow("Intensity Rectified", iTr);
		imshow("Confidency Rectified", cTr);
		imshow("Depth", dT);
		imshow("Intensity", iT);
		imshow("Confidency", cT);
		
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
	
 	// Close camera
  	orf.close();
	
	return 0;
}