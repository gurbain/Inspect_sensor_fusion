/*! 
* 	\file    main.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    June 2014
* 	\version 0.1
* 	\brief   Headers of the main program
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/


#ifndef MAIN_HH
#define MAIN_HH

// Project libs
#include "defines.h"
#include "camera.h"

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
#include <string>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/version.h>
#include <fstream>
#include <signal.h>

void init();
int main(int argc, char **argv);

#endif