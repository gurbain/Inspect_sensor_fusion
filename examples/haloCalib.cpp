/*! 
* 	\file    haloCalib.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to calibrate all cameras on Halo Hardware
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
	int retVal = halo.init();
	if (retVal !=0 )
		exit_handler(0);
	
	// Calibrate
	retVal = halo.calib();
	if (retVal !=0 )
		exit_handler(0);

	// Close cameras
	halo.close();
	
	return 0;
}