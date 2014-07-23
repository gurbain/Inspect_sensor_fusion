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
ORF::ORF(bool use_filter) : orfCam_(NULL), imgEntryArray_(NULL), buffer_(NULL)
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
int ORF::initOrf(int auto_exposure, int integration_time, int modulation_freq, int amp_threshold, string ether_addr) 
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
	sprintf (buffer, "SwissRanger device open. Number of images available: %d\n", inr_); 
	DEBUG<<buffer;
	if ( (cols_ != ORF_COLS) || (rows_ != ORF_ROWS) || (inr_ < ORF_IMAGES) || (imgEntryArray_ == 0) ) {
		SafeCleanup();
		char buffer[100];
		sprintf(buffer, "Invalid data images: %d %dx%d images received from camera!\nExpected %d %dx%d images.\n", inr_, cols_, rows_, ORF_IMAGES, ORF_COLS, ORF_ROWS);
		DEBUG<<buffer<<endl;
		return (-1);
	}

	// Set every parameters
	if (auto_exposure >= 0)
		setAutoExposure(auto_exposure);
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
void ORF::captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame)
{
	// Verify the handle integrity
	SR_SetMode(orfCam_, AM_COR_FIX_PTRN|AM_CONV_GRAY|AM_DENOISE_ANF|AM_CONF_MAP);
	if (orfCam_ == NULL) {
		ERROR<<"Read attempted on NULL SwissRanger port!"<<endl;
		return;
	}

	// Acquire data with time stamp
	int res;
	time_t time1 = time(0);
	res = SR_Acquire (orfCam_);
	time_t time2 = time(0);
	if (res < 0) {
		ERROR<<"Unable to capture data"<<endl;
		return;
	}
	time_t timestamp = ((int)time1 + (int)time2) / 2;

	// Points array
	res = SR_CoordTrfFlt (orfCam_, xp_, yp_, zp_, sizeof (float), sizeof (float), sizeof (float));  

	// Fill the pictures
	Mat depth(ORF_ROWS, ORF_COLS, CV_16UC1, SR_GetImage (orfCam_, 0));
	Mat intensity(ORF_ROWS, ORF_COLS, CV_16U, SR_GetImage (orfCam_, 1));
	Mat confidence(ORF_ROWS, ORF_COLS, CV_16U, SR_GetImage (orfCam_, 2));
	Size newSize(ORF_COLS*3, ORF_ROWS*3);
	resize(depth, depth, newSize);
	resize(intensity, intensity, newSize);
	resize(confidence, confidence, newSize);
	imshow("Depth", depth);
	imshow("Intensity", intensity);
	imshow("Confidence", confidence);

	return;
}

//////////////////////////
//////Set parameters//////
//////////////////////////
int ORF::setAutoExposure (bool on)
{
	int res = SR_SetAutoExposure(orfCam_,1,150,5,70);
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
	char *buff;
	SR_GetDeviceString(orfCam_, buff, 100);
	string s(buff);
	device_id_ = s;
	return s;
}

string ORF::getLibraryVersion ()
{
	unsigned short tab[4];
	SR_GetVersion(tab);
	stringstream ss;
	INFO<<"Version "<<tab[3]<<"."<<tab[2]<<"."<<tab[1]<<"."<<tab[0];
	lib_version_ = ss.str();
	return ss.str();
}
