/*! 
* 	\file    utils.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Software global tools headers
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef UTILS_HH
#define UTILS_HH

// Project libs
#include "defines.h"

// OpenCV libs
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

// Common libs
#include <sys/utsname.h>
#include <stdint.h>
#include <iomanip>
#include <limits>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/version.h>
#include <sys/stat.h>

// Defines
#define DELTATMIN	150
#define DELTATMAX	300

using namespace std;

static timeval timeValInit;

void init();
vector<cv::Point3f> create3DChessboardCorners(cv::Size boardSize, float squareSize);

class TimeStamp {
	
	public:
		timeval timeBegin;
		timeval timeEnd;
		timeval timeInit;
		int meanTime;
		int procTime;
		int startTime;
		int stopTime;
		int initTime;
		bool isRunning;
	
	
		TimeStamp();
		~TimeStamp();
		
		void start();
		void stop();
		int load(string line_);
		int getProcTime();
		int getMeanTime();
		int getStartTime();
		int getStopTime();
		int setTime(int meanTime_, int procTime_);
		bool isSynchro(TimeStamp t_);
};


#endif