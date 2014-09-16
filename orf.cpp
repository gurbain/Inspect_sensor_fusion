/*! 
* 	\file    orf.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Sources for optical range finder class
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "orf.h"

using namespace std;

//////////////////////////
//////  Constructor //////
//////////////////////////
ORF::ORF() : 
orfCam_(NULL), imgEntryArray_(NULL), buffer_(NULL),
imgWidth(640), imgHeight(480), 
imgNum(0), tslast(0),
timestamps("timestampORF.txt"),
auto_exposure(true), integration_time(100), modulation_freq(15), 
amp_threshold(20), ether_addr("192.168.1.42")
{
	imageSize = Size(imgWidth, imgHeight);
}

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
int ORF::init() 
{
	// We are using the camera
	this->load_image = false;
	
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
	SR_SetMode (orfCam_, AM_COR_FIX_PTRN|AM_CONV_GRAY|AM_DENOISE_ANF|AM_CONF_MAP|AM_SW_TRIGGER);

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
	
	// Set camera mode
	SR_SetMode(orfCam_, AM_COR_FIX_PTRN|AM_CONV_GRAY|AM_DENOISE_ANF|AM_CONF_MAP);

	// Create a new timestamp file and dir
	time_t rawtime;
	struct tm * timeinfo;
	char buffer1[100];
	stringstream buffer2;
	time (&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer1,100,"%d-%m-%Y-%I-%M", timeinfo);
	buffer2<<SAVE_DIR<<"/"<<buffer1;
	string dir = buffer2.str();
	this->save_dir = dir;
	
	struct stat st;
	if(stat(this->save_dir.c_str(),&st) != 0) {
		mkdir(save_dir.c_str(), 0777);
		INFO<<"Creation of the folder "<<save_dir<<endl;
		mkdir(this->save_dir.c_str(), 0777);
		INFO<<"Creation of the folder "<<this->save_dir<<endl;
	}
	stringstream ss;
	ss<<this->save_dir<<"/"<<this->timestamps;
	tsfile.open(ss.str().c_str());
	ss.str("");
	if (tsfile.is_open())
		tsfile<<endl<<endl<<"######################### NEW SESSION #######################"<<endl<<endl;
	return 0;
}

int ORF::init(string dir)
{
	this->load_dir = dir;
	this->load_image = true;
	return 0;
}



//////////////////////////
////// Safe Cleanup //////
//////////////////////////
void ORF::SafeCleanup() {
	if (!load_image) {
		if (orfCam_) {
			SR_Close (orfCam_);
		}

		if (buffer_)
			free(buffer_);

		orfCam_ = NULL;
		buffer_ = NULL;
	}
}

//////////////////////////
//////     Close    //////
//////////////////////////
int ORF::close() 
{
	if (!load_image) {
		// Close file timestamp
		if (tsfile.is_open()) {
			INFO<<"Close "<<timestamps<<" file"<<endl;
			tsfile.close();
		}
		
		// Close camera
		if (orfCam_)
			if (SR_Close (orfCam_))
				DEBUG<<"Unable to close the camera!"<<endl;

		// Free resources
		SafeCleanup();
		
		INFO<<"ORF camera has been closed"<<endl;
	}

	return 0;
}


//////////////////////////
//////   Read Data  //////
//////////////////////////

int ORF::captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, int num)
{
	// Start the timeStamp
	ts.start();
	
	// If enable, load images instead of capturing
	if (load_image) {
		// Set the names
		char buffer[10];
		sprintf(buffer, "%i", num);
		string filenamed = string(this->load_dir) + "/orfDepth" + buffer + ".png";
		string filenamev = string(this->load_dir) + "/orfVisual" + buffer + ".png";
		string filenamec = string(this->load_dir) + "/orfConfidency" + buffer + ".png";
		
		// Recover images
		try {
			depthNewImageFrame = imread(filenamed, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
			visualNewImageFrame = imread(filenamev, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
			confidenceNewImageFrame = imread(filenamec, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
			if (depthNewImageFrame.empty() || visualNewImageFrame.empty() || confidenceNewImageFrame.empty()) {
				ERROR<<"No More ORF file found! If you saw nothing, check directory!"<<endl;
				return -1;
			}
		} catch (int ex) {
			ERROR<<"Exception when reading ORF file "<<ex<<endl;
			return -1;
		}
	// Else, capture images
	} else {
		// Verify handle integrity
		SR_SetMode(orfCam_, MODE);
		if (orfCam_ == NULL) {
			ERROR<<"Read attempted on NULL SwissRanger port!"<<endl;
			return -1;
		}

		// Do the acquisition
		int retVal = SR_Acquire (orfCam_);
		if (retVal < 0) {
			ERROR<<"Unable to capture data"<<endl;
			return -1;
		}
		
		// Points array
		//retVal = SR_CoordTrfFlt (orfCam_, xp_, yp_, zp_, sizeof (float), sizeof (float), sizeof (float));  

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
		normalize(visualNewImageFrame, visualNewImageFrame, 0, 255, NORM_MINMAX, CV_8UC1);
		equalizeHist(visualNewImageFrame, visualNewImageFrame);
	}
	
	// Stop the timeStamp
	ts.stop();
	
	return 0;
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
		timemin = 10;
		timemax = 20;
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
	INFO<<"SwissRanger device version "<<str<<endl;
	lib_version_ = str;
	return str;
}


int ORF::calib(string filename)
{
	orfCalib(filename);
}

int ORF::captureRectifiedOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, int num, string filename)
{
	// Start the timeStamp
	ts.start();
	
	int retVal;
	if (mapx.empty() || mapy.empty()) {
		// Load calibration parameters
		FileStorage storage;
		retVal = storage.open(filename, FileStorage::READ);
		if (retVal==1) {
			INFO<<"ORF Calibration file found! No need to perform calibration!"<<endl;
		} else {
			INFO<<"Calibration file not found! Calibration needed!"<<endl;
			calib(filename);
			retVal = storage.open(filename, FileStorage::READ);
			if (retVal!=1) {
				ERROR<<"File cannot be open or read! Verify user rights"<<endl;
				return -1;
			}
		}
		storage["Intrinsicparameters"]>>intrinsicMatrix;
		storage["Distortioncoefficients"]>>distorsionCoeffs;
		f = intrinsicMatrix.at<double>(0,0);
		cx = intrinsicMatrix.at<double>(0,2);
		cy = intrinsicMatrix.at<double>(1,2);
		storage.release();
		
		// Build the undistort map that we will use for all subsequent frames
		Size imageSize(imgWidth, imgHeight);
		Mat Rect, newCameraMatrix;
		mapx.create(imageSize, CV_32FC1);
		mapy.create(imageSize, CV_32FC1);
		initUndistortRectifyMap(intrinsicMatrix, distorsionCoeffs, Rect, newCameraMatrix, imageSize, CV_32FC1, mapx, mapy);
	}
	
	// Capture an image
	Mat dt, it, ct;
	TimeStamp t;
	retVal = captureOrf(dt, it, ct, t, num);
	if (retVal!=0)
		return -1;
	
	// Remap the image
	remap(dt, depthNewImageFrame, mapx, mapy, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
	remap(it, visualNewImageFrame, mapx, mapy, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
	remap(ct, confidenceNewImageFrame, mapx, mapy, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
	
	// Stop the timeStamp
	ts.stop();

	return 0;
}

int ORF::saveOrf()
{
	if (load_image) {
		ERROR<<"You cannot save images loaded from hard drive!"<<endl;
		return -1;
	}
	
	// Create variables to save
	TimeStamp ts;
	Mat depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
	
	// Capture images
	this->captureOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, ts);
	
	//Save jpg images
	char buffer[10];
	sprintf(buffer, "%i", imgNum);
	mkdir(SAVE_DIR, 0777);
	string filenamed = this->save_dir + "/orfDepth" + buffer + ".png";
	string filenamev = this->save_dir + "/orfVisual" + buffer + ".png";
	string filenamec = this->save_dir + "/orfConfidency" + buffer + ".png";
	
	try {
		imwrite(filenamed, depthNewImageFrame);
		imwrite(filenamev, visualNewImageFrame);
		imwrite(filenamec, confidenceNewImageFrame);
	} catch (int ex) {
		ERROR<<"Exception converting image to jpg format: "<<ex<<endl;
		return -1;
	}
	
	// Save time stamp
	if (!tsfile.is_open()) {
		tsfile.open(timestamps.c_str());
		if (!tsfile.is_open()) {
			ERROR<<"Impossible to open the file"<<endl;
			return -1;
		}
	}
	if (imgNum==0)
		INFO<<"Saving ORF images into folder "<<this->save_dir<<endl;
	tsfile<<"IMAGENUM\t"<<imgNum<<"\tPROCTIME\t"<<ts.getProcTime()<<"\tMEANTIME\t"<<ts.getMeanTime()<<"\tDIFF\t"<<ts.getMeanTime()-tslast<<endl;

	imgNum++;
	tslast = ts.getMeanTime();
	
	return 0;
}

int ORF::saveRectifiedOrf()
{
	if (load_image) {
		ERROR<<"You cannot save images loaded from hard drive!"<<endl;
		return -1;
	}
	
	// Create variables to save
	TimeStamp ts;
	Mat depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
	
	// Capture images
	this->captureRectifiedOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, ts);
	
	//Save jpg images
	char buffer[10];
	sprintf(buffer, "%i", imgNum);
	mkdir(SAVE_DIR, 0777);
	string filenamed = this->save_dir + "/orfDepth" + buffer + ".png";
	string filenamev = this->save_dir + "/orfVisual" + buffer + ".png";
	string filenamec = this->save_dir + "/orfConfidency" + buffer + ".png";
	
	try {
		imwrite(filenamed, depthNewImageFrame);
		imwrite(filenamev, visualNewImageFrame);
		imwrite(filenamec, confidenceNewImageFrame);
	} catch (int ex) {
		ERROR<<"Exception converting image to jpg format: "<<ex<<endl;
		return -1;
	}
	
	// Save time stamp
	if (!tsfile.is_open()) {
		tsfile.open(timestamps.c_str());
		if (!tsfile.is_open()) {
			ERROR<<"Impossible to open the file"<<endl;
			return -1;
		}
	}
	if (imgNum==0)
		INFO<<"Saving ORF images into folder "<<this->save_dir<<endl;
	tsfile<<"IMAGENUM\t"<<imgNum<<"\tPROCTIME\t"<<ts.getProcTime()<<"\tMEANTIME\t"<<ts.getMeanTime()<<"\tDIFF\t"<<ts.getMeanTime()-tslast<<endl;

	imgNum++;
	tslast = ts.getMeanTime();
	
	return 0;
}

int ORF::capture3Dcloud(vector<Point3d>& pointcloud, vector<Vec3b>& rgbcloud, int num, int downsampling, string filename)
{
	// Variables declaration
	Mat dT, iT, cT;
	TimeStamp t;
	Point3d newPoint;
	
	// Capture images
	this->captureRectifiedOrf(dT, iT, cT, t, num, filename);
	OrfTriangulator ORFtriangle(this->intrinsicMatrix); //NB: the matrix always exist since we performed rectification just before
	
	// Down sample
	resize(dT, dT, Size((int)dT.cols/downsampling, (int)dT.rows/downsampling));
	resize(iT, iT, Size((int)iT.cols/downsampling, (int)iT.rows/downsampling));
	resize(cT, cT, Size((int)cT.cols/downsampling, (int)cT.rows/downsampling));
	
	// Add color
	Mat dT2;
	dT.convertTo(dT2, CV_8U, 0.00390625);
	Mat iT_col(iT.size(), CV_8UC3);
	Mat dT_col(dT2.size(), CV_8UC3);
	applyColorMap(dT2, dT_col, COLORMAP_HSV);
	cvtColor(iT, iT_col, CV_GRAY2RGB);
	addWeighted(dT_col,.3,iT_col,.7,0,iT_col);
	vector<Mat> channels;
	split(iT_col, channels);
	
	// For each point, compute 3D coordinates and intensity
	for (int i=0; i<iT.rows; i++) {
		for (int j=0; j<iT.cols; j++) {
			//if ((unsigned short)(cT.at<uchar>(i, j)) > 0) {
				double z = ((dT.at<unsigned short>(i, j)>>2) & 0x3FFF)*0.00061;
				if (z > 0.4 && z < 3) {
					//cout<<"C: "<<(short)cT.at<uchar>(i, j)<<" D: "<<z<<" I: "<<(short)iT.at<uchar>(i, j)<<endl;
					newPoint = ORFtriangle.triangulateOrf(j*downsampling, i*downsampling, z);
					pointcloud.push_back(newPoint);
					rgbcloud.push_back(Vec3b((short)(channels[0].at<uchar>(i, j)), (short)(channels[1].at<uchar>(i, j)), (short)(channels[2].at<uchar>(i, j))));
				}
			//}
		}
	}
	
	return 0;
}