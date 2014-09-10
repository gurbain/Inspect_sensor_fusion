/*! 
* 	\file    stereoCalib.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to calibrate STEREO cameras
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
	int retVal = stereo.init("../examples/stereo_calib_dataset_3/");
	if (retVal !=0 )
		exit_handler(0);
	
	// Main loop
	retVal = stereo.calib();
	if (retVal !=0 )
		exit_handler(0);
	
 	// Close camera
  	stereo.close();
	
	return 0;
}