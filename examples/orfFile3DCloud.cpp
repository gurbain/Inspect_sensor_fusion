/*! 
* 	\file    orfFile3DCloud.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to load ORF images and display the 3D cloud
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
	InterruptVisualizationThread();
	exit(1);
}

int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,exit_handler);
	
	// Initialize
	int retVal = orf.init("../examples/test1/");
	if (retVal !=0 )
		exit_handler(0);
	
	// Launch vizualization (you can give the pixel dimensions)
	RunVisualizationThread(2);
	
	// Main loop
	int num = 0;
	int downsampling = 5;
	while(true) {
		// Capture cloud
		vector<Point3d> pointcloud;
		vector<Vec3b> rgbcloud;
		retVal = orf.capture3Dcloud(pointcloud, rgbcloud, num, downsampling);
		if (retVal !=0 )
			exit_handler(0);
		
		// Display cloud
		ShowClouds(pointcloud, rgbcloud);
		
		usleep(1000000);
		num++;
	}
	
	// Wait for updating vizualization
	WaitForVisualizationThread();
	
 	// Close camera
  	orf.close();
	
	return 0;
}