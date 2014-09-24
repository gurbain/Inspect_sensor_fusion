/*! 
* 	\file    camera.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Sources for stereo rig class adapted from VERTIGO project
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "camera.h"


Cameras::Cameras() : 
iterDummy(0),
useSynchCams(true), reduceImageSizeTo320x240(false), useCameras(true),
h_cam1(0), h_cam2(1),
pixelClockFreq(10), frameRate(5),exposureTime(20), flashDelay(0), flashDuration(15000),
imgWidth(640), imgHeight(480), imgBitsPixel(8), imgBufferCount(RING_BUFFER_SIZE),
act_img_buf1(NULL), act_img_buf2(NULL), last_img_buf1(NULL), last_img_buf2(NULL),
timestamps("timestampOpticsMount.txt"),
hwGain(100)
{
	imageSize = Size(imgWidth, imgHeight);
	sprintf(CAMERA_1_SERIAL, "4002795734"); // this is the serial number of the left camera, a standard initialization
}

int Cameras::getImageWidth()
{
	return this->imgWidth;
}

int Cameras::getImageHeight()
{
	return this->imgHeight;
}

double Cameras::getFrameRate()
{
	if (this->useCameras) {
		int retVal;
		double fr;

		retVal = is_SetFrameRate(h_cam1, IS_GET_FRAMERATE, &fr);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Could not read Frame Rate from Camera (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		return fr;
	} else {
		return this->frameRate;
	}
}

void Cameras::setFrameRate(float newFR)
{
	this->frameRate = newFR;
}

double Cameras::getExposureTime()
{
	int retVal;
	double exp_time;

	retVal = is_SetExposureTime(h_cam1, IS_GET_EXPOSURE_TIME, &exp_time);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Could not read Exposure Time from Camera (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	return exp_time;
}

void Cameras::setExposureTime(int newET)
{
	this->exposureTime = newET;
	return;
}

void Cameras::setHWGain(int gain)
{
	this->hwGain = gain;
	return;
}

char* Cameras::getCam1Serial()
{
	return this->CAMERA_1_SERIAL;
}

void Cameras::setCam1Serial(const char* serialNum)
{
	sprintf(this->CAMERA_1_SERIAL, "%s", serialNum);
	return;
}

unsigned int Cameras::init()
{
	parseParameterFile();
	if (this->reduceImageSizeTo320x240) {
		this->numberOfBytesToCopy = 76800;
	} else {
		this->numberOfBytesToCopy = 307200;
	}
	this->load_image = false;
	
	int img_xpos, img_ypos, img_width, img_height;
	int bits_pixel;
	double fps, exp_time;
	int retVal;
	int i;

	h_cam1 = 0;
	h_cam2 = 1;
	memset(&camInfo1,0,sizeof(CAMINFO));
	memset(&camInfo2,0,sizeof(CAMINFO));

	this->maxFrameRate = float(this->pixelClockFreq * 1000000) / float((752*480));
	if (this->frameRate > this->maxFrameRate) {
		INFO<<"Specified FrameRate is higher than camera pixel clock frequency allows! --- frameRate will be set to maximally possible value of "<< this->maxFrameRate<<endl;
		this->frameRate = this->maxFrameRate;
	}

	//initialize camera 1 & 2
	retVal = is_InitCamera(&h_cam1,0);
	if (retVal != IS_SUCCESS) {
		h_cam1 = 0;
		ERROR<<"Init Camera Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_InitCamera(&h_cam2,0);
	if (retVal != IS_SUCCESS) {
		h_cam2 = 0;
		ERROR<<"Init Camera Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//Get camera info 1 & 2
	retVal = is_GetCameraInfo(h_cam1, &camInfo1);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Get Camera Info Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"S/N of CAM 1: "<<camInfo1.SerNo<<" - Select: "<<(float)camInfo1.Select<<endl;
	retVal = is_GetCameraInfo(h_cam2, &camInfo2);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Get Camera Info Failed (Code:  "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"S/N of CAM 2: "<<camInfo2.SerNo<<" - Select: "<<(float)camInfo2.Select<<endl;

	for (int count = 0; count < 16; count++) {
		if (OPTICS_ID[count] == -1) {
			DEBUG<<"CAMERA S/N Not Found"<<endl;
			break;
		}
		if (strcmp(camInfo1.SerNo,LEFT_CAM_SERIAL[count]) == 0) {
			strcpy(this->CAMERA_1_SERIAL, camInfo1.SerNo);
			INFO<< "LEFT CAM S/N: " << this->CAMERA_1_SERIAL << " (Not Swapped)" << endl;
			INFO<< "OPTICS MOUNT S/N: " << OPTICS_ID[count] << endl;
			break;
		} else if (strcmp(camInfo2.SerNo,LEFT_CAM_SERIAL[count]) == 0) {
			strcpy(this->CAMERA_1_SERIAL, camInfo2.SerNo);
			INFO<< "LEFT CAM S/N: " << this->CAMERA_1_SERIAL << " (Swapped)" << endl;
			INFO<< "OPTICS MOUNT S/N: " << OPTICS_ID[count] << endl;
			HIDS h_cam_temp;
			h_cam_temp = h_cam2;
			h_cam2 = h_cam1;
			h_cam1 = h_cam_temp;
			break;
		}
	}

	// set pixelClock
	retVal = is_SetPixelClock (h_cam1, this->pixelClockFreq);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Pixel Clock Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_SetPixelClock (h_cam2, this->pixelClockFreq);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Pixel Clock Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//get color modes 1 & 2
	bits_pixel = this->imgBitsPixel;
	retVal = is_SetColorMode(h_cam1, IS_SET_CM_Y8);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Color Mode Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetColorMode(h_cam2, IS_SET_CM_Y8);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Color Mode Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	if (reduceImageSizeTo320x240) {
		retVal = is_SetBinning(h_cam1, IS_BINNING_2X_VERTICAL || IS_BINNING_2X_HORIZONTAL);
		if (retVal != IS_SUCCESS) {
			DEBUG<<"Set Binning failed for Camera 1 (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		retVal = is_SetBinning(h_cam2, IS_BINNING_2X_VERTICAL || IS_BINNING_2X_HORIZONTAL);
		if (retVal != IS_SUCCESS) {
			DEBUG<<"Set Binning failed for Camera 2 (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		this->imgHeight = int( (this->imgHeight)/2 );
		this->imgWidth = int( (this->imgWidth)/2 );
	}

	//set area of interest 1 & 2
	img_width = this->imgWidth;
	img_height = this->imgHeight;

	if (reduceImageSizeTo320x240) {
		img_ypos = 0;
		img_xpos = 28;	//shifts the area of interest so that the image in the center of the ccd
	} else {
		img_ypos = 0;
		img_xpos = 56;	//shifts the area of interest so that the image in the center of the ccd
	}

	retVal = is_SetAOI(h_cam1, IS_SET_IMAGE_AOI, &img_xpos, &img_ypos, &img_width, &img_height);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"SetAOI Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetAOI(h_cam2, IS_SET_IMAGE_AOI, &img_xpos, &img_ypos, &img_width, &img_height);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"SetAOI Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	for(i = 0; i < this->imgBufferCount; i++) {
		//cam 1
		//allocate image memory
		retVal = is_AllocImageMem(h_cam1, img_width, img_height, bits_pixel, &(this->Cam1_ringbuffer[i].pBuf), &(this->Cam1_ringbuffer[i].img_id) );
		if (retVal != IS_SUCCESS) {
			ERROR<<"AllocImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		//add to sequence
		retVal = is_AddToSequence(h_cam1, this->Cam1_ringbuffer[i].pBuf, this->Cam1_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS) {
			DEBUG<<"AddToSequence Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		//cam 2
		//allocate image memory
		retVal = is_AllocImageMem(h_cam2, img_width, img_height, bits_pixel, &this->Cam2_ringbuffer[i].pBuf, &this->Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS) {
			ERROR<<"AllocImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		//add to sequence
		retVal = is_AddToSequence(h_cam2, this->Cam2_ringbuffer[i].pBuf, this->Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS) {
			DEBUG<<"AddToSequence Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
	}

	//set up frame rate 1 & 2
	retVal = is_SetFrameRate(h_cam1, this->frameRate, &fps);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Frame Rate Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Frame rate: "<<fps<<endl;
	retVal = is_SetFrameRate(h_cam2, this->frameRate, &fps);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Frame Rate Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Frame rate: "<<fps<<endl;

	//set up exposure time 1 & 2
	retVal = is_SetExposureTime(h_cam1, this->exposureTime, &exp_time);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Exposure Time Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Exposure Time: "<<exp_time<<endl;
	retVal = is_SetExposureTime(h_cam2, this->exposureTime, &exp_time);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Exposure Time Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Exposure Time: "<<exp_time<<endl;

	//set gain value
	retVal = is_SetGainBoost(h_cam1, IS_SET_GAINBOOST_OFF);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Enable GainBoost Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetGainBoost(h_cam2, IS_SET_GAINBOOST_OFF);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Enable GainBoost Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetHardwareGain(h_cam1, this->hwGain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set HW Gain Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetHardwareGain(h_cam2, this->hwGain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set HW Gain Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set camera 1 to wait for a trigger on the digital input line
	//from camera 2 (falling edge is all that is available for the hw)
	retVal = is_SetExternalTrigger (h_cam1, IS_SET_TRIGGER_HI_LO);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set External Trigger Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up flash from camera 2 to have a falling edge when there is an exposure
	//this triggers camera 1
	retVal = is_SetFlashStrobe(h_cam2, IS_SET_FLASH_LO_ACTIVE_FREERUN, 0);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Flash Strobe Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up flash delay and duration on camera 2
	//0 us delay and 5 ms duration of the low pulse
	// Cam2 drives Cam1 using the flashStrobe, but the current flash is for the next image capture
	int flashDelayTemp =  int( 1000000 / (this->frameRate) - (this->exposureTime)*1000 );
	retVal = is_SetFlashDelay(h_cam2, flashDelayTemp, 5000);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Flash Delay Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up the flash on camera 1 to drive the led lights
	//high whenever
	retVal = is_SetFlashStrobe(h_cam1, IS_SET_FLASH_HI_ACTIVE, 0);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Flash Strobe Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up LED delay/duration to be customizable in optics2.h
	retVal = is_SetFlashDelay(h_cam1, this->flashDelay, this->flashDuration);
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Set Flash Delay Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//Enable frame event
	retVal = is_EnableEvent(h_cam1, IS_SET_EVENT_FRAME);        // enable frame event
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Enable Event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_EnableEvent(h_cam2, IS_SET_EVENT_FRAME);        // enable frame event
	if (retVal != IS_SUCCESS) {
		DEBUG<<"Enable Event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	
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
	struct stat st;
	if(stat(dir.c_str(),&st) != 0) {
		INFO<<"Creation of the folder "<<this->save_dir<<endl;
		mkdir(this->save_dir.c_str(), 0777);
		INFO<<"Creation of the folder "<<dir<<endl;			
		mkdir(dir.c_str(), 0777);
	}
	this->save_dir = dir;
	stringstream ss;
	ss<<this->save_dir<<"/"<<this->timestamps;
	tsfile.open(ss.str().c_str());
	ss.str("");
	if (tsfile.is_open())
		tsfile<<endl<<endl<<"######################### NEW SESSION #######################"<<endl<<endl;
	

	DEBUG<<"Initialization successfull!"<<endl;
	return 0;
}

unsigned int Cameras::init(string dir)
{
	this->load_dir = dir;
	this->load_image = true;
	
	return 0;
}

unsigned int Cameras::close()
{
	if (!load_image) {
		int i, retVal;

		for(i = 0; i < this->imgBufferCount; i++)
		{
			//deallocate image memory
			retVal = is_FreeImageMem(h_cam1, this->Cam1_ringbuffer[i].pBuf, this->Cam1_ringbuffer[i].img_id);
			if (retVal != IS_SUCCESS)
				DEBUG<<"FreeImageMem Failed (Code: "<<retVal<<")"<<endl;
			
			this->Cam1_ringbuffer[i].pBuf = NULL;
			this->Cam1_ringbuffer[i].img_id = 0;

			retVal = is_FreeImageMem(h_cam2, this->Cam2_ringbuffer[i].pBuf, this->Cam2_ringbuffer[i].img_id);
			if (retVal != IS_SUCCESS)
				DEBUG<<"FreeImageMem Failed (Code: "<<retVal<<")"<<endl;
			
			this->Cam2_ringbuffer[i].pBuf = NULL;
			this->Cam2_ringbuffer[i].img_id = 0;
		}

		//close camera 1 & 2
		if(h_cam1)
			is_ExitCamera(h_cam1);
		h_cam1 = 0;
		if(h_cam2)
			is_ExitCamera(h_cam2);
		h_cam2 = 0;
		INFO<<"Stereo cameras have been closed"<<endl;
	}
	
	return 0;
}

unsigned int Cameras::start()
{
	if (!load_image) {
		int retVal;
		
		//start capture 1 & 2
		retVal = is_CaptureVideo(h_cam2, IS_DONT_WAIT);
		if (retVal != IS_SUCCESS)
		{
			ERROR<<"Capture Video Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		retVal = is_CaptureVideo(h_cam1, IS_DONT_WAIT);
		if (retVal != IS_SUCCESS)
		{
			ERROR<<"Capture Video Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		DEBUG<<"Capture Video successfull!"<<endl;
	}
	return 0;

}

unsigned int Cameras::stop()
{
	if (!load_image) {
		int retVal;

		retVal = is_StopLiveVideo(h_cam1, IS_WAIT);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Stop Live Video Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		retVal = is_StopLiveVideo(h_cam2, IS_WAIT);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Stop Live Video Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
	}
	return 0;
}

unsigned int Cameras::captureTwoImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, TimeStamp& ts, int& synchCheckFlag, int num)
{
	// If we are loading image from file
	if (load_image) {
		// Set the names
		char buffer[10];
		sprintf(buffer, "%i", num);
		string filenamel = string(this->load_dir) + "/leftImg" + buffer + ".png";
		string filenamer = string(this->load_dir) + "/rightImg" + buffer + ".png";
		// Recover images
		try {
			leftNewImageFrame = imread(filenamel, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
			rightNewImageFrame = imread(filenamer, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
			if (leftNewImageFrame.empty() || rightNewImageFrame.empty()) {
				ERROR<<"No more stereo file found! If you saw nothing, check directory!"<<endl;
				return -1;
			}
		} catch (int ex) {
			ERROR<<"Exception when reading OpticsMount file "<<ex<<endl;
			return -1;
		}
	
	// Else, we are capturing images
	} else {
		// Create variables
		int retVal;
		iterDummy++;
		
		// Start the timeStamp
		ts.start();

		retVal = is_WaitEvent(h_cam1, IS_SET_EVENT_FRAME, WAIT_TIMEOUT_MS);
		if (retVal != IS_SUCCESS)
		{
			printf("Is wait event Failed (Code: %d)\n", retVal);
			return retVal;
		}

		if (iterDummy > 1)
		{
			// Unlock active image buffer for camera 2
			retVal = is_UnlockSeqBuf (h_cam2, (rightImgNum), (this->act_img_buf2));
			if (retVal != IS_SUCCESS)
			{
				printf("is_UnlockSeqBuf for active buffer Failed (Code: %d)\n", retVal);
				return retVal;
			}

			prevImgNum = rightImgNum -1;
			if (rightImgNum == 1)
				prevImgNum = imgBufferCount;
		}

		retVal = is_WaitEvent(h_cam2, IS_SET_EVENT_FRAME, WAIT_TIMEOUT_MS);
		if (retVal != IS_SUCCESS)
		{
			printf("Is wait event Failed (Code: %d)\n", retVal);
			return retVal;
		}

		retVal = is_GetActSeqBuf (h_cam2, &rightImgNum, &(this->act_img_buf2), &(this->last_img_buf2));
		if (retVal != IS_SUCCESS)
		{
			printf("GetActSeqBuf Failed (Code: %d)\n", retVal);
			return retVal;
		}
		Mat right(Size(imgWidth, imgHeight), CV_8U, this->last_img_buf2);
		rightNewImageFrame = right;

		// lock active image buffer for camera 2, this particular memory will be skipped while camera 2 is running in freerun mode
		retVal = is_LockSeqBuf (h_cam2, rightImgNum, (this->act_img_buf2));
		if (retVal != IS_SUCCESS)
		{
			printf("is_LockSeqBuf for active buffer Failed (Code: %d)\n", retVal);
			return retVal;
		}

		prevImgNum = rightImgNum -1;
		if (rightImgNum == 1)
			prevImgNum = imgBufferCount;
		
		retVal = is_GetActSeqBuf (h_cam1, &leftImgNum, &(this->act_img_buf1), &(this->last_img_buf1));
		if (retVal != IS_SUCCESS)
		{
			printf("GetActSeqBuf Failed (Code: %d)\n", retVal);
			return retVal;
		}
		Mat left(Size(imgWidth, imgHeight), CV_8U, this->last_img_buf1);
		leftNewImageFrame = left;
		
		// check if last_img_buf position has changed
		// if not, current frames are out of synch --> don't change frames (= use previous frames) and return synchCheckFlag = -1
		if(this->last_img_buf1 == bufDummy1 || this->last_img_buf2 == bufDummy2) {
			synchCheckFlag = -1;
		} else {
			Mat left(Size(imgWidth, imgHeight), CV_8U, this->last_img_buf1);
			Mat right(Size(imgWidth, imgHeight), CV_8U, this->last_img_buf2);
			leftNewImageFrame = left;
			rightNewImageFrame = right;
			synchCheckFlag = 0;

		}

		this->bufDummy1 = this->last_img_buf1;
		this->bufDummy2 = this->last_img_buf2;

		// Stop the timeStamp
		ts.stop();
	}
	return 0;
}

int Cameras::calib(string filename)
{
	stereoCalib(filename);
}

unsigned int Cameras::captureTwoRectifiedImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, TimeStamp& ts, int num, string filename)
{
	// Usefull variables
	int *img_num1, *img_num2;
	int flags = 0;
	int retVal;
	
	// Start the timeStamp
	ts.start();
	
	// Check if calib file already exist
	if (mapxL.empty() || mapyL.empty() || mapxR.empty() || mapyR.empty()) {
		// Load calibration matrices
		FileStorage storage;
		retVal = storage.open(filename, FileStorage::READ);
		if (retVal==1) {
			INFO<<"Optics Mount Calibration file found! No need to perform calibration!"<<endl;
		} else {
			INFO<<"Calibration file not found! Calibration needed!"<<endl;
			calib(filename);
			retVal = storage.open(filename, FileStorage::READ);
			if (retVal!=1) {
				ERROR<<"File cannot be open or read! Verify user rights"<<endl;
				return -1;
			}
		}
		storage["intrinsicMatrixL"]>>intrinsicMatrixL;
		storage["distorsionCoeffsL"]>>distorsionCoeffsL;
		storage["projMatrixL"]>>projMatrixL;
		storage["rotMatrixL"]>>rotMatrixL;
		storage["intrinsicMatrixR"]>>intrinsicMatrixR;
		storage["distorsionCoeffsR"]>>distorsionCoeffsR;
		storage["projMatrixR"]>>projMatrixR;
		storage["rotMatrixR"]>>rotMatrixR;
		storage["R"]>>R;
		storage["T"]>>T;
		storage["E"]>>E;
		storage["F"]>>F;
		storage.release();
		f = projMatrixL.at<double>(0,0);
		Tx = T.at<double>(0,0);
		Ty = T.at<double>(0,1);
		Tz = T.at<double>(0,2);
		cxL = projMatrixL.at<double>(0,2);
		cyL = projMatrixL.at<double>(1,2);
		cxR = projMatrixR.at<double>(0,2);
		cyR = projMatrixR.at<double>(1,2);
		
		// Build the undistort map that we will use for all subsequent frames
		Size imageSize(imgWidth, imgHeight);
		Mat RectL, newCameraMatrixL, RectR, newCameraMatrixR;;
		mapxL.create(imageSize, CV_32FC1);
		mapyL.create(imageSize, CV_32FC1);
		mapxR.create(imageSize, CV_32FC1);
		mapyR.create(imageSize, CV_32FC1);
		initUndistortRectifyMap(intrinsicMatrixL, distorsionCoeffsL, rotMatrixL, projMatrixL, imageSize, CV_16SC2, mapxL, mapyL);
		initUndistortRectifyMap(intrinsicMatrixR, distorsionCoeffsR, rotMatrixR, projMatrixR, imageSize, CV_16SC2, mapxR, mapyR);
	}
	
	// Capture an image
	Mat iL, iR;
	TimeStamp t;
	retVal = captureTwoImages(iL, iR, img_num1, img_num2, t, flags, num);
	if (retVal!=0)
		return -1;
	
	// Remap the image
	remap(iL, leftNewImageFrame, mapxL, mapyL, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
	remap(iR, rightNewImageFrame, mapxR, mapyR, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
	
	// Stop the timeStamp
	ts.stop();
	
	return 0;

}

void Cameras::parseParameterFile() {

	string line;
	string searchString;
	string foundString;
	size_t found;

	char parameterFilePath[200] = "parameterFile1.txt";
	ifstream parameterFile(parameterFilePath);

	if (parameterFile.is_open()) {
		INFO<<endl<<"--- PARAMETERFILE INPUT ---" <<endl;
		while (parameterFile.good()) {

			// Get line by line
			getline (parameterFile, line);
			if (line[0] != '#') {
				// frameRate
				searchString = "FRAME_RATE";
				found = line.find(searchString);
				if (found != string::npos) {
					foundString = line.substr( found+searchString.size()+1, string::npos );
					setFrameRate( atof(foundString.c_str()) );
					cout << "FRAME_RATE " << foundString << endl;
				}
				searchString.clear();
				found = string::npos;

				// exposureTime
				searchString = "EXPOSURE_TIME";
				found = line.find(searchString);
				if (found != string::npos) {
					foundString = line.substr( found+searchString.size()+1, string::npos );
					setExposureTime( atoi(foundString.c_str()) );
					cout << "EXPOSURE_TIME " << foundString << endl;
				}
				searchString.clear();
				found = string::npos;

				// exposureTime
				searchString = "HW_GAIN";
				found = line.find(searchString);
				if (found != string::npos) {
					foundString = line.substr( found+searchString.size()+1, string::npos );
					setHWGain( atoi(foundString.c_str()) );
					cout << "HW_GAIN " << foundString << endl;
				}
				searchString.clear();
				found = string::npos;


				// useSynchCams
				searchString = "USE_SYNCH_CAMS";
				found = line.find(searchString);
				if (found != string::npos) {
					foundString = line.substr( found+searchString.size()+1, string::npos );
					if (foundString == "true")
						useSynchCams = true;
					if (foundString == "false")
						useSynchCams = false;
					cout << "USE_SYNCH_CAMS " << foundString << endl;
				}
				searchString.clear();
				found = string::npos;

				// reduceImageSizeTo320x240
				searchString = "REDUCE_IMAGE_SIZE_TO_320X240";
				found = line.find(searchString);
				if (found != string::npos) {
					foundString = line.substr( found+searchString.size()+1, string::npos );
					if (foundString == "true")
						reduceImageSizeTo320x240 = true;
					if (foundString == "false")
						reduceImageSizeTo320x240 = false;
					cout << "REDUCE_IMAGE_SIZE_TO_320X240 " << foundString << endl;
				}
				searchString.clear();
				found = string::npos;

			}
		}
		parameterFile.close();
		cout << "---------------------------" << endl;


	} else {
		DEBUG<<"Unable to open "<<parameterFilePath<<endl;
	}

	//Open the /opt/GogglesDaemon/CAMERA_FILE
	ifstream cameraFile(CAMERA_FILE);

	if (cameraFile.is_open()) {
		for (int count = 0; count < 16; count++) {
			getline(cameraFile,line);
			if (cameraFile.eof()) {
				OPTICS_ID[count] = -1;
				break;
			}
			found = line.find(":");
			strcpy(LEFT_CAM_SERIAL[count],line.substr(0,found).c_str());
			OPTICS_ID[count] = atoi(line.substr(found+1,string::npos).c_str());
			INFO<<"Cam S/N & ID: "<<LEFT_CAM_SERIAL[count]<< ":"<<OPTICS_ID[count]<<endl;
		}
	} else {
		DEBUG << "Unable to open: " << CAMERA_FILE << endl;
	}
}

unsigned int Cameras::saveTwoRectifiedImages()
{
	if (load_image) {
		ERROR<<"You cannot save images loaded from hard drive!"<<endl;
		return -1;
	}
	
	// Create variables to save
	TimeStamp ts;
	Mat leftImg, rightImg;
	
	// Capture images
	this->captureTwoRectifiedImages(leftImg, rightImg, ts);
	
	//Save jpg images
	stringstream ssl, ssr;
	ssl<<this->save_dir<<"/leftImg"<<imgNum<<".png";
	ssr<<this->save_dir<<"/rightImg"<<imgNum<<".png";
	string filenamel = ssl.str();
	string filenamer = ssr.str();
	ssl.str("");
	ssr.str("");
	
	try {
		imwrite(filenamel, leftImg);
		imwrite(filenamer, rightImg);
	} catch (int ex) {
		ERROR<<"Exception converting image to png format: "<<ex<<endl;
		return -1;
	}
	
	// Save time stamp
	if (!tsfile.is_open()) {
		tsfile.open(this->timestamps.c_str());
		tsfile<<endl<<endl<<"######################### NEW SESSION #######################"<<endl<<endl;
		if (!tsfile.is_open()) {
			ERROR<<"Impossible to open the file"<<endl;
			return -1;
		}
	}
	if (imgNum==0)
		cout<<"Saving Opticsmount images into folder "<<this->save_dir<<endl;
	tsfile<<"IMAGENUM\t"<<imgNum<<"\tPROCTIME\t"<<ts.getProcTime()<<"\tMEANTIME\t"<<ts.getMeanTime()<<"\tDIFF\t"<<ts.getMeanTime()-tslast<<endl;

	imgNum++;
	tslast = ts.getMeanTime();
	
	return 0;
}

unsigned int Cameras::saveTwoImages()
{
	if (load_image) {
		ERROR<<"You cannot save images loaded from hard drive!"<<endl;
		return -1;
	}
	
	// Create variables to save
	TimeStamp ts;
	Mat leftImg, rightImg;
	int *img_num1, *img_num2;
	int flags = 0;
	
	// Capture images
	this->captureTwoImages(leftImg, rightImg, img_num1, img_num2, ts, flags);
	
	//Save jpg images
	stringstream ssl, ssr;
	ssl<<this->save_dir<<"/leftImg"<<imgNum<<".png";
	ssr<<this->save_dir<<"/rightImg"<<imgNum<<".png";
	string filenamel = ssl.str();
	string filenamer = ssr.str();
	ssl.str("");
	ssr.str("");
	
	try {
		imwrite(filenamel, leftImg);
		imwrite(filenamer, rightImg);
	} catch (int ex) {
		ERROR<<"Exception converting image to png format: "<<ex<<endl;
		return -1;
	}
	
	// Save time stamp
	if (!tsfile.is_open()) {
		tsfile.open(this->timestamps.c_str());
		tsfile<<endl<<endl<<"######################### NEW SESSION #######################"<<endl<<endl;
		if (!tsfile.is_open()) {
			ERROR<<"Impossible to open the file"<<endl;
			return -1;
		}
	}
	if (imgNum==0)
		INFO<<"Saving stereo images into folder "<<this->save_dir<<endl;
	tsfile<<"IMAGENUM\t"<<imgNum<<"\tPROCTIME\t"<<ts.getProcTime()<<"\tMEANTIME\t"<<ts.getMeanTime()<<"\tDIFF\t"<<ts.getMeanTime()-tslast<<endl;

	imgNum++;
	tslast = ts.getMeanTime();
	
	return 0;
}


int Cameras::capture3Dcloud(vector<Point3d>& pointcloud, vector<Vec3b>& rgbcloud, int num, int downsampling, string filename)
{
	// Variables declaration
	Mat iL, iR;
	TimeStamp t;
	Point3d newPoint;
	
	// Capture images
	this->captureTwoRectifiedImages(iL, iR, t, num, filename);
	
	// TODO BGM
	
	return 0;
}