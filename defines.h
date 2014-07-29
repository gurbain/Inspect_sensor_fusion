/*! 
* 	\file    defines.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   All the constant parameters of the software
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/


#ifndef DEFINES_HH
#define DEFINES_HH

// Output streamings defines
#define __AUTHOR__	"Gabriel Urbain"
#define __F__		(strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define ERROR		cout << "\033[31m["<<setprecision(3)<<((float)clock())/CLOCKS_PER_SEC<<"] "<<"ERROR in "<<__F__<<":"<<__LINE__<<": \033[00m"
#define DEBUG		cout << "\033[33m["<<setprecision(3)<<((float)clock())/CLOCKS_PER_SEC<<"] "<<"DEBUG in "<<__F__<<":"<<__LINE__<<": \033[00m"
#define INFO		cout << "["<<setprecision(3)<<((float)clock())/CLOCKS_PER_SEC<<"] "<<"INFO: "  


// Orf cameras defines
#define USE_SR4K 1
#define USE_FILTER 1
#define ORF_IMG_DISTANCE   0
#define ORF_IMG_AMPLITUDE  1
#define ORF_IMG_CONFIDENCE 2
#define MODE (AM_CONF_MAP | AM_CONV_GRAY | AM_COR_FIX_PTRN | AM_DENOISE_ANF)


// Stereo camera defines
#define RING_BUFFER_SIZE	10
#define WAIT_TIMEOUT_MS		10000

// Other libs
#include <sys/time.h>
	

// // Global variables
// long timeInit;
// timeval timeValInit;

#endif