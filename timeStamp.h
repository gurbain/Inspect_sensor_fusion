/*! 
* 	\file    timeStamp.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Headers of the Time Stamp utilities
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef TIMESTAMP_HH
#define TIMESTAMP_HH

// Project libs
#include "defines.h"

// OpenCV libs
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

// Common libs
#include <stdint.h>
#include <iomanip>
#include <limits>
#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

// Defines
#define DELTATMIN	50
#define DELTATMAX	150

using namespace std;

class TimeStamp {
	
	public:
		timeval timeBegin;
		timeval timeEnd;
		timeval timeInit;
		int meanTime;
		int procTime;
		bool isRunning;
	
	
		TimeStamp();
		~TimeStamp();
		
		void start();
		void stop();
		int getProcTime();
		int getMeanTime();
		bool isSynchro(TimeStamp t);
};

#endif