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
void exit_handler(int s){
	halo.close();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,exit_handler);
	
	// Initialize
	int retVal = halo.init("../examples/halo_calib_dataset/");
	if (retVal !=0 )
		exit_handler(0);
	
	// Launch vizualization (you can give the pixel dimensions)
	//RunVisualizationThread(2);
	// Main loop
	while(true) {
		// Capture cloud
		vector<Point3d> pointcloud;
		vector<Vec3b> rgbcloud;
		retVal = halo.capture3Dcloud(pointcloud, rgbcloud);
		if (retVal !=0 )
			exit_handler(0);
		
		
		// Display cloud
		ShowClouds(pointcloud, rgbcloud);
	}
	
	// Wait for updating vizualization
	//WaitForVisualizationThread();
	
	// Close cameras
	halo.close();
	
	return 0;
}