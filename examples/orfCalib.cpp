/*! 
* 	\file    orfCalib.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to calibrate ORF
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
	retVal = orf.calib();
	if (retVal !=0 )
		exit_handler(0);
	
 	// Close camera
  	orf.close();
	
	return 0;
}