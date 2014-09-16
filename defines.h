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

// Orf camera defines
#define USE_SR4K 1
#define USE_FILTER 1
#define ORF_IMG_DISTANCE   0
#define ORF_IMG_AMPLITUDE  1
#define ORF_IMG_CONFIDENCE 2
#define MODE (AM_CONF_MAP | AM_CONV_GRAY | AM_COR_FIX_PTRN | AM_DENOISE_ANF)
#define THRESH_ORF_CONF    230

#define ORF_COLS	176
#define ORF_ROWS	144
#define ORF_IMAGES	3

#define ORF_FOV_H	56
#define ORF_FOV_V	70

#define ORF_CLOUD_DOWNSAMPLING	4

// Stereo cameras defines
#define RING_BUFFER_SIZE	10
#define WAIT_TIMEOUT_MS		10000

#define CAM_FOV_H	35
#define CAM_FOV_V	35

#define TEST_PROJ_CHANNEL_FILE "/opt/GogglesDaemon/TEST_PROJ_CHANNEL"
#define CAMERA_FILE "/opt/GogglesOptics/CAMERA_FILE"


typedef struct _UEYE_IMAGE
{
	char    *pBuf;
	int     img_id;
	int     img_seqNum;
	int     nBufferSize;
} UEYE_IMAGE;

// Halo cameras defines
#define SYNCHRONOUS	1
#define ASYNCHRONOUS	0
#define CALIB_DIR	"calib"

// Calibration defines
#define CALIB_DEBUG

#define ORF_NUMBER_BOARDS	10
#define STEREO_NUMBER_BOARDS	20
#define HALO_NUMBER_BOARDS	5

#define BOARD_WIDTH	6
#define BOARD_HEIGHT	11
#define BOARD_SIZE	Size(BOARD_WIDTH, BOARD_HEIGHT)
#define SQUARE_SIZE	25 //mm

// Save and folder defines
#define SAVE_DIR "/home/gabs48/results"

// Triangulator defines
#define TRI_MONO	0
#define TRI_STEREO	1

// Utils defines
#define DELTATMIN	150
#define DELTATMAX	300

// Fusion
#define PI	3.14159265
#define FUSION_DEBUG
#define SIGMA_T_COEFF	15
#define VAR_KER		2
#define TAD_KER		2
#define TAD_THRESH	128
#define OM_PREC		0.000006
#define OM_FOCUS	0.0028
#define K_INT		4



// Global variables
// long timeInit;
// timeval timeValInit;

#endif