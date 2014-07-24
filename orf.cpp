/*! 
* 	\file    orf.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Sources for optical range finder class
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "orf.h"


using namespace orf;
using namespace std;


//////////////////////////
//////  Constructor //////
//////////////////////////
ORF::ORF(bool use_filter) : 
orfCam_(NULL), imgEntryArray_(NULL), buffer_(NULL),
imgWidth(640), imgHeight(480)
{}

//////////////////////////
//////   Destructor //////
//////////////////////////
ORF::~ORF() 
{
  SafeCleanup();
}

//////////////////////////
//////     Open     //////
//////////////////////////
int ORF::initOrf(bool auto_exposure, int integration_time, int modulation_freq, int amp_threshold, string ether_addr) 
{
	// Open camera handling exceptions
	int res = 0;
	if(ether_addr != "") {
		res = SR_OpenETH (&orfCam_, ether_addr.c_str());
	}
	else
		res = SR_OpenUSB (&orfCam_, 0);	
	if (res <= 0) {
		SafeCleanup();
		ERROR<<"Failed to open ORF device!"<<endl;
		return (-1);
	}

	// Variable creation
	device_id_   = getDeviceString ();
	lib_version_ = getLibraryVersion ();
	int rows_ = SR_GetRows (orfCam_);
	int cols_ = SR_GetCols (orfCam_);

	// Set acquisition mode and verify
	SR_SetMode (orfCam_, MODE);

	int inr_  = SR_GetImageList (orfCam_, &imgEntryArray_);
	char buffer [50];
	sprintf (buffer, "SwissRanger device open. Number of images available: %d", inr_); 
	DEBUG<<buffer<<endl;
	if ( (cols_ != ORF_COLS) || (rows_ != ORF_ROWS) || (inr_ < ORF_IMAGES) || (imgEntryArray_ == 0) ) {
		SafeCleanup();
		char buffer[100];
		sprintf(buffer, "Invalid data images: %d %dx%d images received from camera! Expected %d %dx%d images.", inr_, cols_, rows_, ORF_IMAGES, ORF_COLS, ORF_ROWS);
		DEBUG<<buffer<<endl;
		return (-1);
	}

	// Set every parameters
	if (auto_exposure==true) {
		setAutoExposure(true);
	} else {
		setAutoExposure(false);
	}
	if (integration_time >=0 && integration_time != getIntegrationTime())
		setIntegrationTime(integration_time);
	if (modulation_freq >=0 && modulation_freq != getModulationFrequency())
		setModulationFrequency(modulation_freq);
	if (amp_threshold >=0 && amp_threshold != getAmplitudeThreshold())
		setAmplitudeThreshold(amp_threshold);


	// Create point arrays
	size_t buffer_size = rows_ * cols_ * 3 * sizeof (float);
	buffer_ = (float*)malloc (buffer_size);
	memset (buffer_, 0xaf, buffer_size);

	xp_ = buffer_;
	yp_ = &xp_[rows_*cols_];
	zp_ = &yp_[rows_*cols_];
	
	return 0;
}

//////////////////////////
////// Safe Cleanup //////
//////////////////////////
void ORF::SafeCleanup() {
	if (orfCam_) {
		SR_Close (orfCam_);
	}

	if (buffer_)
		free(buffer_);

	orfCam_ = NULL;
	buffer_ = NULL;

}

//////////////////////////
//////     Close    //////
//////////////////////////
int ORF::closeOrf() 
{
	if (orfCam_)
		if (SR_Close (orfCam_))
			DEBUG<<"Unable to close the camera!"<<endl;

	// Free resources
	SafeCleanup();

	return 0;
}


//////////////////////////
//////   Read Data  //////
//////////////////////////
int ORF::captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame)
{
	// Verify the handle integrity
	SR_SetMode(orfCam_, AM_COR_FIX_PTRN|AM_CONV_GRAY|AM_DENOISE_ANF|AM_CONF_MAP);
	if (orfCam_ == NULL) {
		ERROR<<"Read attempted on NULL SwissRanger port!"<<endl;
		return -1;
	}

	// Acquire data with time stamp
	int res;
	time_t time1 = time(0);
	res = SR_Acquire (orfCam_);
	time_t time2 = time(0);
	if (res < 0) {
		ERROR<<"Unable to capture data"<<endl;
		return -1;
	}
	time_t timestamp = ((int)time1 + (int)time2) / 2;

	// Points array
	res = SR_CoordTrfFlt (orfCam_, xp_, yp_, zp_, sizeof (float), sizeof (float), sizeof (float));  

	// Fill the pictures
	Mat depth(ORF_ROWS, ORF_COLS, CV_16U, SR_GetImage (orfCam_, 0));
	Mat visual(ORF_ROWS, ORF_COLS, CV_16U, SR_GetImage (orfCam_, 1));
	Mat confidence(ORF_ROWS, ORF_COLS, CV_16U, SR_GetImage (orfCam_, 2));
	
	// Image resizing
	Size newSize(imgWidth, imgHeight);
	resize(depth, depthNewImageFrame, newSize);
	resize(visual, visualNewImageFrame, newSize);
	resize(confidence, confidenceNewImageFrame, newSize);
	
	// Image processing
// 	normalize(visualNewImageFrame, visualNewImageFrame, 0, 255, NORM_MINMAX, CV_8UC1);
// 	equalizeHist(visualNewImageFrame, visualNewImageFrame);
	// 	Mat depth2, depth3;
//  	depth.convertTo(depth, CV_8U, 0.00390625);
// 	visual.convertTo(visual, CV_8U, 0.00390625);
// 	confidence.convertTo(confidence, CV_8U, 0.00390625);
	//GaussianBlur(depth, depth2,Size(15,15), 1.0);
	//GaussianBlur(depth, depth3,Size(30,30), 1.0);
	//normalize(depth, depth, 0, 255, NORM_MINMAX, CV_8UC1);
// 	normalize(depth2, depth2, 0, 255, NORM_MINMAX, CV_8UC1);
// 	normalize(depth3, depth3, 0, 255, NORM_MINMAX, CV_8UC1);
	//normalize(confidence, confidence, 0, 255, NORM_MINMAX, CV_8UC1);
// 	equalizeHist(depth, depth);
// 	equalizeHist(depth2, depth2);
// 	equalizeHist(depth3, depth3);

	//equalizeHist(confidence, confidence);
	//blur(depth, depth2,Size(5,5));

// 	imshow("Depth", depth);
//  	//imshow("Depth2", depth2);
// // 	imshow("Depth3", depth3);
// 	imshow("Intensity", visual);
// 	imshow("Confidence", confidence);

	return 1;
}

//////////////////////////
//////Set parameters//////
//////////////////////////
int ORF::setAutoExposure (bool on)
{
	int timemin, timemax, percOverPos, desiredPos;
	if (on==true) {
		timemin = 1;
		timemax = 150;
		percOverPos = 5;
		desiredPos = 70;
	} else {
		timemin = 255;
		timemax = 0;
		percOverPos = 0;
		desiredPos = 0;
	}	
	int res = SR_SetAutoExposure(orfCam_,timemin,timemax,percOverPos,desiredPos);
	INFO<<"Auto exposure parameters have been set to:\n\t\tMin integration time = "<<timemin<<"s\n\t\tMax integration time = "<<timemax<<"s\n\t\tPercentage of the historigram above position = "<<percOverPos<<"\%\n\t\tDesired mean percentage of the historigram = "<<desiredPos<<"\%"<<endl;
	return res;
}
int ORF::setIntegrationTime (int time)
{
	int res = SR_SetIntegrationTime(orfCam_, time);
	return res;
}
int ORF::setAmplitudeThreshold (int thresh)
{
	int res = SR_SetAmplitudeThreshold(orfCam_, thresh);
	return res;
}
int ORF::setModulationFrequency (int freq)
{
	enum ModulationFrq m;
	switch(freq) {
		case 40: m = MF_40MHz;
			break;
		case 30: m = MF_30MHz;
			break;
		case 21: m = MF_21MHz;
			break;
		case 20: m = MF_20MHz;
			break;
		case 19: m = MF_19MHz;
			break;
		case 60: m = MF_60MHz;
			break;
		case 15: m = MF_15MHz;
			break;
		case 10: m = MF_10MHz;
			break;
		case 29: m = MF_29MHz;
			break;
		case 31: m = MF_31MHz;
			break;
		case 14: m = MF_14_5MHz;
			break;
		default : m = MF_LAST;
			break;
	}
	INFO<<"Modulation frequency has been set to "<<freq<<"MHz (num "<<m<<")"<<endl;
	int res = SR_SetModulationFrequency(orfCam_, m);
	return res;
}

//////////////////////////
//////Get parameters//////
//////////////////////////
int ORF::getIntegrationTime ()
{
	return SR_GetIntegrationTime(orfCam_);
}

double ORF::getModulationFrequency ()
{
	double res = 0;
	enum ModulationFrq m = SR_GetModulationFrequency(orfCam_);
	switch(m) {
		case 0: res = MF_40MHz;
			break;
		case 1: res = MF_30MHz;
			break;
		case 2: res = MF_21MHz;
			break;
		case 3: res = MF_20MHz;
			break;
		case 4: res = MF_19MHz;
			break;
		case 5: res = MF_60MHz;
			break;
		case 6: res = MF_15MHz;
			break;
		case 7: res = MF_10MHz;
			break;
		case 8: res = MF_29MHz;
			break;
		case 9: res = MF_31MHz;
			break;
		case 10: res = MF_14_5MHz;
			break;
		case 11: res = MF_15_5MHz;
			break;
		case 12: res = MF_LAST;
			break;
	}
	return res;
}

int ORF::getAmplitudeThreshold ()
{
	return SR_GetAmplitudeThreshold(orfCam_);
}

string ORF::getDeviceString ()
{
	char buff[100];
	SR_GetDeviceString(orfCam_, buff, 100);
	string s(buff);
	device_id_ = s;
	return s;
}

string ORF::getLibraryVersion ()
{
	unsigned short tab[4];
	SR_GetVersion(tab);
	char buf[10];
	sprintf(buf, "%i.%i.%i.%i", tab[3], tab[2], tab[1], tab[0]);
	string str(buf);
	INFO<<"Version "<<str<<endl;
	lib_version_ = str;
	return str;
}
