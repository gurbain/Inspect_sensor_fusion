/*! 
* 	\file    camera.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Sources for stereo rig class
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "camera.h"

using namespace cam;

Cameras::Cameras() : 
iterDummy(0),
useSynchCams(true), reduceImageSizeTo320x240(true), useCameras(true),
h_cam1(0), h_cam2(1),
pixelClockFreq(10), frameRate(5),exposureTime(20), flashDelay(0), flashDuration(15000),
imgWidth(640), imgHeight(480), imgBitsPixel(8), imgBufferCount(RING_BUFFER_SIZE),
act_img_buf1(NULL), act_img_buf2(NULL), last_img_buf1(NULL), last_img_buf2(NULL),
hwGain(100)
{
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

}

void Cameras::setHWGain(int gain)
{

	this->hwGain = gain;

}


char* Cameras::getCam1Serial()
{

	return this->CAMERA_1_SERIAL;

}

void Cameras::setCam1Serial(const char* serialNum)
{

	sprintf(this->CAMERA_1_SERIAL, "%s", serialNum);

}

unsigned int Cameras::initTwoCameras()
{
	if (this->reduceImageSizeTo320x240)
	{
		this->numberOfBytesToCopy = 76800;
	}
	else
	{
		this->numberOfBytesToCopy = 307200;
	}


	if (this->useSynchCams)
	{
		this->initTwoCamerasSynch();
	}

	else
	{
		this->initTwoCamerasNotSynch();
	}

}

unsigned int Cameras::closeTwoCameras()
{

	if (this->useSynchCams)
	{
		this->closeTwoCamerasSynch();
	}

	else
	{
		this->closeTwoCamerasNotSynch();
	}

}
unsigned int Cameras::startTwoCameras()
{

	if (this->useSynchCams)
	{
		this->startTwoCamerasSynch();
	}

	else
	{
		this->startTwoCamerasNotSynch();
	}

}

unsigned int Cameras::stopTwoCameras()
{

	if (this->useSynchCams)
	{
		this->stopTwoCamerasSynch();
	}

	else
	{
		this->stopTwoCamerasNotSynch();
	}

}

unsigned int Cameras::captureTwoImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, int& synchCheckFlag)
{

	if (this->useSynchCams)
	{
		this->captureTwoImagesSynch(leftNewImageFrame, rightNewImageFrame, img_num1, img_num2, synchCheckFlag);
	}

	else
	{
		this->captureTwoImagesNotSynch(leftNewImageFrame, rightNewImageFrame, img_num1, img_num2);
	}

}

///////////////////////////////////////////////////////////////
//// Begin of Synch functions
unsigned int Cameras::initTwoCamerasSynch()
{
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
	if (this->frameRate > this->maxFrameRate)
	{
		INFO<<"Specified FrameRate is higher than camera pixel clock frequency allows! --- frameRate will be set to maximally possible value of "<< this->maxFrameRate<<endl;
		this->frameRate = this->maxFrameRate;
	}

	//initialize camera 1 & 2
	retVal = is_InitCamera(&h_cam1,0);
	if (retVal != IS_SUCCESS)
	{
		h_cam1 = 0;
		ERROR<<"Init Camera Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_InitCamera(&h_cam2,0);
	if (retVal != IS_SUCCESS)
	{
		h_cam2 = 0;
		ERROR<<"Init Camera Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//Get camera info 1 & 2
	retVal = is_GetCameraInfo(h_cam1, &camInfo1);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Get Camera Info Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"S/N of CAM 1: "<<camInfo1.SerNo<<" - Select: "<<(float)camInfo1.Select<<endl;
	retVal = is_GetCameraInfo(h_cam2, &camInfo2);
	if (retVal != IS_SUCCESS)
	{
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
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Pixel Clock Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_SetPixelClock (h_cam2, this->pixelClockFreq);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Pixel Clock Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//get color modes 1 & 2
	bits_pixel = this->imgBitsPixel;
	retVal = is_SetColorMode(h_cam1, IS_SET_CM_Y8);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Color Mode Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetColorMode(h_cam2, IS_SET_CM_Y8);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Color Mode Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	if (reduceImageSizeTo320x240)
	{

		retVal = is_SetBinning(h_cam1, IS_BINNING_2X_VERTICAL || IS_BINNING_2X_HORIZONTAL);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Set Binning failed for Camera 1 (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		retVal = is_SetBinning(h_cam2, IS_BINNING_2X_VERTICAL || IS_BINNING_2X_HORIZONTAL);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Set Binning failed for Camera 2 (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		this->imgHeight = int( (this->imgHeight)/2 );
		this->imgWidth = int( (this->imgWidth)/2 );

	}

	//set area of interest 1 & 2
	img_width = this->imgWidth;
	img_height = this->imgHeight;

	if (reduceImageSizeTo320x240)
	{
		img_ypos = 0;
		img_xpos = 28;	//shifts the area of interest so that the image in the center of the ccd
	}
	else
	{
		img_ypos = 0;
		img_xpos = 56;	//shifts the area of interest so that the image in the center of the ccd
	}


	retVal = is_SetAOI(h_cam1, IS_SET_IMAGE_AOI, &img_xpos, &img_ypos, &img_width, &img_height);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"SetAOI Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetAOI(h_cam2, IS_SET_IMAGE_AOI, &img_xpos, &img_ypos, &img_width, &img_height);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"SetAOI Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	for(i = 0; i < this->imgBufferCount; i++)
	{
		//cam 1
		//allocate image memory
		retVal = is_AllocImageMem(h_cam1, img_width, img_height, bits_pixel, &(this->Cam1_ringbuffer[i].pBuf), &(this->Cam1_ringbuffer[i].img_id) );
		if (retVal != IS_SUCCESS)
		{
			ERROR<<"AllocImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		//add to sequence
		retVal = is_AddToSequence(h_cam1, this->Cam1_ringbuffer[i].pBuf, this->Cam1_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"AddToSequence Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		//printf("Image Buffer Address(%d): 0x%08X\n",i, img_buffer1[i].pBuf);

		//cam 2
		//allocate image memory
		retVal = is_AllocImageMem(h_cam2, img_width, img_height, bits_pixel, &this->Cam2_ringbuffer[i].pBuf, &this->Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			ERROR<<"AllocImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		//add to sequence
		retVal = is_AddToSequence(h_cam2, this->Cam2_ringbuffer[i].pBuf, this->Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"AddToSequence Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		//printf("Image Buffer Address(%d): 0x%08X\n",i, img_buffer2[i].pBuf);
	}

	//set up frame rate 1 & 2
	retVal = is_SetFrameRate(h_cam1, this->frameRate, &fps);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Frame Rate Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Frame rate: "<<fps<<endl;
	retVal = is_SetFrameRate(h_cam2, this->frameRate, &fps);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Frame Rate Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Frame rate: "<<fps<<endl;

	//set up exposure time 1 & 2
	retVal = is_SetExposureTime(h_cam1, this->exposureTime, &exp_time);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Exposure Time Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Exposure Time: "<<exp_time<<endl;
	retVal = is_SetExposureTime(h_cam2, this->exposureTime, &exp_time);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Exposure Time Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Exposure Time: "<<exp_time<<endl;

	//set gain value
	retVal = is_SetGainBoost(h_cam1, IS_SET_GAINBOOST_OFF);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable GainBoost Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_SetGainBoost(h_cam2, IS_SET_GAINBOOST_OFF);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable GainBoost Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_SetHardwareGain(h_cam1, this->hwGain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set HW Gain Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
//	retVal = is_SetHardwareGain(h_cam1, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
//	printf("New Gain Factor: %d\n", retVal);

	retVal = is_SetHardwareGain(h_cam2, this->hwGain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set HW Gain Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
//	retVal = is_SetHardwareGain(h_cam2, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
//	printf("New Gain Factor: %d\n", retVal);


	//set camera 1 to wait for a trigger on the digital input line
	//from camera 2 (falling edge is all that is available for the hw)
	retVal = is_SetExternalTrigger (h_cam1, IS_SET_TRIGGER_HI_LO);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set External Trigger Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up flash from camera 2 to have a falling edge when there is an exposure
	//this triggers camera 1
	retVal = is_SetFlashStrobe(h_cam2, IS_SET_FLASH_LO_ACTIVE_FREERUN, 0);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Strobe Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up flash delay and duration on camera 2
	//0 us delay and 5 ms duration of the low pulse
	// Cam2 drives Cam1 using the flashStrobe, but the current flash is for the next image capture
	int flashDelayTemp =  int( 1000000 / (this->frameRate) - (this->exposureTime)*1000 );
	retVal = is_SetFlashDelay(h_cam2, flashDelayTemp, 5000);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Delay Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up the flash on camera 1 to drive the led lights
	//high whenever
	retVal = is_SetFlashStrobe(h_cam1, IS_SET_FLASH_HI_ACTIVE, 0);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Strobe Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up LED delay/duration to be customizable in optics2.h
	retVal = is_SetFlashDelay(h_cam1, this->flashDelay, this->flashDuration);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Delay Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//Enable frame event
	retVal = is_EnableEvent(h_cam1, IS_SET_EVENT_FRAME);        // enable frame event
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable Event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_EnableEvent(h_cam2, IS_SET_EVENT_FRAME);        // enable frame event
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable Event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	
	DEBUG<<"Fonctionne"<<endl;
	return 0;
}

unsigned int Cameras::closeTwoCamerasSynch()
{
	int i, retVal;

	for(i = 0; i < this->imgBufferCount; i++)
	{
		//deallocate image memory
		retVal = is_FreeImageMem(h_cam1, this->Cam1_ringbuffer[i].pBuf, this->Cam1_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"FreeImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		this->Cam1_ringbuffer[i].pBuf = NULL;
		this->Cam1_ringbuffer[i].img_id = 0;

		retVal = is_FreeImageMem(h_cam2, this->Cam2_ringbuffer[i].pBuf, this->Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"FreeImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		this->Cam2_ringbuffer[i].pBuf = NULL;
		this->Cam2_ringbuffer[i].img_id = 0;

	}

	//close camera 1 & 2
	if(h_cam1)
	{
		is_ExitCamera(h_cam1);
	}
	h_cam1 = 0;
	if(h_cam2)
	{
		is_ExitCamera(h_cam2);
	}
	h_cam2 = 0;

	return 0;
}

unsigned int Cameras::startTwoCamerasSynch()
{
	int retVal;

	//start capture 1 & 2

	retVal = is_CaptureVideo(h_cam2, IS_DONT_WAIT);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Capture Video Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_CaptureVideo(h_cam1, IS_DONT_WAIT);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Capture Video Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	return 0;
}

unsigned int Cameras::stopTwoCamerasSynch()
{
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

	return 0;
}


unsigned int Cameras::captureTwoImagesSynch(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, int& synchCheckFlag)
{
	int retVal;

	iterDummy++;

	//this function is not documented in the programmers manual
	//it is mentioned in README.TXT and used in the demo program...
	//the parameters are (camera_handle, event_type, timeout_in_ms)


	retVal = is_WaitEvent(h_cam1, IS_SET_EVENT_FRAME, WAIT_TIMEOUT_MS);
	if (retVal != IS_SUCCESS)
	{
		ERROR<<"Is wait event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}


	if (iterDummy > 1)
	{

		// Unlock active image buffer for camera 2
		retVal = is_UnlockSeqBuf (h_cam2, (rightImgNum), (this->act_img_buf2));
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"is_UnlockSeqBuf for active buffer Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		prevImgNum = rightImgNum -1;
		if (rightImgNum == 1)
			prevImgNum = imgBufferCount;
/*
		// Unlock last image buffer for camera 2
//		retVal = is_UnlockSeqBuf (h_cam2, IS_IGNORE_PARAMETER, (this->last_img_buf2));
		retVal = is_UnlockSeqBuf (h_cam2, prevImgNum, (this->last_img_buf2));
		if (retVal != IS_SUCCESS)
		{
			printf("is_UnlockSeqBuf for last buffer Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		*/
	}

	retVal = is_WaitEvent(h_cam2, IS_SET_EVENT_FRAME, WAIT_TIMEOUT_MS);
	if (retVal != IS_SUCCESS)
	{
		ERROR<<"Is wait event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_GetActSeqBuf (h_cam2, &rightImgNum, &(this->act_img_buf2), &(this->last_img_buf2));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"GetActSeqBuf Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	rightNewImageFrame.data = (uchar*) this->last_img_buf2;


	// lock active image buffer for camera 2, this particular memory will be skipped while camera 2 is running in freerun mode
	retVal = is_LockSeqBuf (h_cam2, rightImgNum, (this->act_img_buf2));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"is_LockSeqBuf for active buffer Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	prevImgNum = rightImgNum -1;
	if (rightImgNum == 1)
		prevImgNum = imgBufferCount;
/*
	// lock active last buffer for camera 2, this particular memory will be skipped while camera 2 is running in freerun mode
	retVal = is_LockSeqBuf (h_cam2, prevImgNum, (this->last_img_buf2));
//	retVal = is_LockSeqBuf (h_cam2, IS_IGNORE_PARAMETER, (this->last_img_buf2));
	if (retVal != IS_SUCCESS)
	{
		printf("is_LockSeqBuf for last buffer Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
*/

	retVal = is_GetActSeqBuf (h_cam1, &leftImgNum, &(this->act_img_buf1), &(this->last_img_buf1));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"GetActSeqBuf Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	leftNewImageFrame.data = (uchar*) this->last_img_buf1;

	// get current time in ms for timestamp
//	gettimeofday(&this->timeRAW,NULL);
//	imageTimestamp = timeRAW.tv_sec*1000+timeRAW.tv_usec/1000; // ms
//	imageTimestamp = imageTimestamp - zerotime;


	// check if last_img_buf position has changed
	// if not, current frames are out of synch --> don't change frames (= use previous frames) and return synchCheckFlag = -1
	if(this->last_img_buf1 == bufDummy1 || this->last_img_buf2 == bufDummy2)
	{
		synchCheckFlag = -1;
	}
	else
	{
		leftNewImageFrame.data = (uchar*) this->last_img_buf1;
		rightNewImageFrame.data = (uchar*) this->last_img_buf2;

		synchCheckFlag = 0;

	}

	this->bufDummy1 = this->last_img_buf1;
	this->bufDummy2 = this->last_img_buf2;

	return 0;
}

//// End of Synch functions


//// Begin of NotSynch functions
unsigned int Cameras::initTwoCamerasNotSynch()
{
	int img_xpos, img_ypos, imageWidth, imageHeight;
	int bits_pixel;
	double fps, exp_time;
	int retVal;
	int i;

	h_cam1 = 0;
	h_cam2 = 1;
	memset(&camInfo1,0,sizeof(CAMINFO));
	memset(&camInfo2,0,sizeof(CAMINFO));

	//initialize camera 1 & 2
	retVal = is_InitCamera(&h_cam1,0);
	if (retVal != IS_SUCCESS)
	{
		h_cam1 = 0;
		ERROR<<"Init Camera Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_InitCamera(&h_cam2,0);
	if (retVal != IS_SUCCESS)
	{
		h_cam2 = 0;
		ERROR<<"Init Camera Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//Get camera info 1 & 2
	retVal = is_GetCameraInfo(h_cam1, &camInfo1);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Get Camera Info Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"S/N of CAM 1: "<<camInfo1.SerNo<<" - Select: "<<(float)camInfo1.Select<<endl;
	retVal = is_GetCameraInfo(h_cam2, &camInfo2);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Get Camera Info Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"S/N of CAM 2: "<<camInfo2.SerNo<<" - Select: "<<(float)camInfo2.Select<<endl;

	for (int count = 0; count < 16; count++) {
		if (OPTICS_ID[count] == -1) {
			DEBUG << "CAMERA S/N Not Found" << endl;
			break;
		}
		if (strcmp(camInfo1.SerNo,LEFT_CAM_SERIAL[count]) == 0) {
			strcpy(this->CAMERA_1_SERIAL, camInfo1.SerNo);
			INFO << "LEFT CAM S/N: " << this->CAMERA_1_SERIAL << " (Not Swapped)" << endl;
			INFO << "OPTICS MOUNT S/N: " << OPTICS_ID[count] << endl;
			break;
		} else if (strcmp(camInfo2.SerNo,LEFT_CAM_SERIAL[count]) == 0) {
			strcpy(this->CAMERA_1_SERIAL, camInfo2.SerNo);
			INFO << "LEFT CAM S/N: " << this->CAMERA_1_SERIAL << " (Swapped)" << endl;
			INFO << "OPTICS MOUNT S/N: " << OPTICS_ID[count] << endl;
			HIDS h_cam_temp;
			h_cam_temp = h_cam2;
			h_cam2 = h_cam1;
			h_cam1 = h_cam_temp;
			break;
		}
	}

	//get color modes 1 & 2
	bits_pixel = imgBitsPixel;
	retVal = is_SetColorMode(h_cam1, IS_SET_CM_Y8);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Color Mode Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetColorMode(h_cam2, IS_SET_CM_Y8);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Color Mode Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	if (reduceImageSizeTo320x240)
	{

		retVal = is_SetBinning(h_cam1, IS_BINNING_2X_VERTICAL || IS_BINNING_2X_HORIZONTAL);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Set Binning failed for Camera 1 (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		retVal = is_SetBinning(h_cam2, IS_BINNING_2X_VERTICAL || IS_BINNING_2X_HORIZONTAL);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"Set Binning failed for Camera 2 (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		this->imgHeight = int( (this->imgHeight)/2 );
		this->imgWidth = int( (this->imgWidth)/2 );

	}


	//set area of interest 1 & 2
	if (reduceImageSizeTo320x240)
	{
		img_ypos = 0;
		img_xpos = 28;	//shifts the area of interest so that the image in the center of the ccd
	}
	else
	{
		img_ypos = 0;
		img_xpos = 56;	//shifts the area of interest so that the image in the center of the ccd
	}

	INFO<<"img_xpos = "<<img_xpos<<endl;

	retVal = is_SetAOI(h_cam1, IS_SET_IMAGE_AOI, &img_xpos, &img_ypos, &(this->imgWidth), &(this->imgHeight));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"SetAOI Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetAOI(h_cam2, IS_SET_IMAGE_AOI, &img_xpos, &img_ypos, &(this->imgWidth), &(this->imgHeight));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"SetAOI Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	for(i = 0; i < imgBufferCount; i++)
	{
		//cam 1
		//allocate image memory
		retVal = is_AllocImageMem(h_cam1, this->imgWidth, this->imgHeight, bits_pixel, &Cam1_ringbuffer[i].pBuf, &Cam1_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			ERROR<<"AllocImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		//add to sequence
		retVal = is_AddToSequence(h_cam1, Cam1_ringbuffer[i].pBuf, Cam1_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"AddToSequence Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		//printf("Image Buffer Address(%d): 0x%08X\n",i, Cam1_ringbuffer[i].pBuf);

		//cam 2
		//allocate image memory
		retVal = is_AllocImageMem(h_cam2, this->imgWidth, this->imgHeight, bits_pixel, &Cam2_ringbuffer[i].pBuf, &Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			ERROR<<"AllocImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}

		//add to sequence
		retVal = is_AddToSequence(h_cam2, Cam2_ringbuffer[i].pBuf, Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"AddToSequence Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		//printf("Image Buffer Address(%d): 0x%08X\n",i, Cam2_ringbuffer[i].pBuf);
	}

	//set up frame rate 1 & 2
	retVal = is_SetFrameRate(h_cam1, frameRate, &fps);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Frame Rate Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Frame rate: "<<fps<<endl;
	retVal = is_SetFrameRate(h_cam2, frameRate, &fps);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Frame Rate Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Frame rate: "<<fps<<endl;

	//set gain value
	retVal = is_SetGainBoost(h_cam1, IS_SET_GAINBOOST_OFF);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable GainBoost Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_SetGainBoost(h_cam2, IS_SET_GAINBOOST_OFF);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable GainBoost Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_SetHardwareGain(h_cam1, this->hwGain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set HW Gain Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
//	retVal = is_SetHardwareGain(h_cam1, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
//	printf("New Gain Factor: %d\n", retVal);

	retVal = is_SetHardwareGain(h_cam2, this->hwGain, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set HW Gain Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
//	retVal = is_SetHardwareGain(h_cam2, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
//	printf("New Gain Factor: %d\n", retVal);


	//set up exposure time 1 & 2
	retVal = is_SetExposureTime(h_cam1, exposureTime, &exp_time);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Exposure Time Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Exposure Time: "<<exp_time<<endl;
	retVal = is_SetExposureTime(h_cam2, exposureTime, &exp_time);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Exposure Time Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	INFO<<"New Exposure Time: "<<exp_time<<endl;

	//set up flash strobe 1 & 2
	retVal = is_SetFlashStrobe(h_cam1, IS_SET_FLASH_HI_ACTIVE_FREERUN, 0);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Strobe Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetFlashStrobe(h_cam2, IS_SET_FLASH_HI_ACTIVE_FREERUN, 0);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Strobe Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//set up flash delay 1 & 2
	retVal = is_SetFlashDelay(h_cam1, flashDelay, flashDuration);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Delay Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_SetFlashDelay(h_cam2, flashDelay, flashDuration);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Set Flash Delay Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	//Enable frame event
	retVal = is_EnableEvent(h_cam1, IS_SET_EVENT_FRAME);        // enable frame event
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable Event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_EnableEvent(h_cam2, IS_SET_EVENT_FRAME);        // enable frame event
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Enable Event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	return 0;
}


unsigned int Cameras::closeTwoCamerasNotSynch()
{
	int i, retVal;

	for(i = 0; i < imgBufferCount; i++)
	{
		//deallocate image memory
		retVal = is_FreeImageMem(h_cam1, Cam1_ringbuffer[i].pBuf, Cam1_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"FreeImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		Cam1_ringbuffer[i].pBuf = NULL;
		Cam1_ringbuffer[i].img_id = 0;

		retVal = is_FreeImageMem(h_cam2, Cam2_ringbuffer[i].pBuf, Cam2_ringbuffer[i].img_id);
		if (retVal != IS_SUCCESS)
		{
			DEBUG<<"FreeImageMem Failed (Code: "<<retVal<<")"<<endl;
			return retVal;
		}
		Cam2_ringbuffer[i].pBuf = NULL;
		Cam2_ringbuffer[i].img_id = 0;

	}

	//close camera 1 & 2
	if(h_cam1)
	{
		is_ExitCamera(h_cam1);
	}
	h_cam1 = 0;
	if(h_cam2)
	{
		is_ExitCamera(h_cam2);
	}
	h_cam2 = 0;

	return 0;
}

unsigned int Cameras::startTwoCamerasNotSynch()
{
	int retVal;

	//start capture 1 & 2
	retVal = is_CaptureVideo(h_cam1, IS_DONT_WAIT);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Capture Video Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_CaptureVideo(h_cam2, IS_DONT_WAIT);
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"Capture Video Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	
	return 0;
}

unsigned int Cameras::stopTwoCamerasNotSynch()
{
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

	return 0;
}

unsigned int Cameras::captureTwoImagesNotSynch(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2)
{

	int retVal;

	//this function is not documented in the programmers manual
	//it is mentioned in README.TXT and used in the demo program...
	//the parameters are (camera_handle, event_type, timeout_in_ms)

	retVal = is_WaitEvent(h_cam1, IS_SET_EVENT_FRAME, WAIT_TIMEOUT_MS);
	if (retVal != IS_SUCCESS)
	{
		ERROR<<"Is wait event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}
	retVal = is_WaitEvent(h_cam2, IS_SET_EVENT_FRAME, WAIT_TIMEOUT_MS);
	if (retVal != IS_SUCCESS)
	{
		ERROR<<"Is wait event Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_GetActSeqBuf(h_cam1, img_num1, &(this->act_img_buf1), &(this->last_img_buf1));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"GetActSeqBuf Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	retVal = is_GetActSeqBuf(h_cam2, img_num2, &(this->act_img_buf2), &(this->last_img_buf2));
	if (retVal != IS_SUCCESS)
	{
		DEBUG<<"GetActSeqBuf Failed (Code: "<<retVal<<")"<<endl;
		return retVal;
	}

	leftNewImageFrame.data = (uchar*) this->last_img_buf1;
	rightNewImageFrame.data = (uchar*) this->last_img_buf2;

	// get current time in ms for timestamp
//	gettimeofday(&this->timeRAW,NULL);
//	imageTimestamp = timeRAW.tv_sec*1000+timeRAW.tv_usec/1000; // ms
//	imageTimestamp = imageTimestamp - zerotime;

	return 0;
}
//// end of NotSynch cameras functions


Rectifier::Rectifier()
{
	rectifierOn = true;
}

int Rectifier::calcRectificationMaps(int imgwidth, int imgheight, const char calibParamDir[200])
{
	// file names and paths to intrinsics and extrinsics
	// files are read by calcRectificationMaps
	sprintf(intrinsic_filename, "%s/intrinsics.yml", calibParamDir);
	sprintf(extrinsic_filename, "%s/extrinsics.yml", calibParamDir);

	INFO<<"Opening files: " << intrinsic_filename << ", " << extrinsic_filename << endl;

    //read intrinsics and extrinsics
	cv::FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        ERROR<<"Failed to open file "<<intrinsic_filename<<endl;
        INFO<<"Check camera calibration parameter directory environment variable and correct calibration set number" << endl;
        return -1;
    }

    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(extrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        ERROR<<"Failed to open file "<<extrinsic_filename<<endl;
        return -1;
    }

    fs["R"] >> R;
    fs["T"] >> T;

    //compute rectification matriciesC
    stereoRectify( M1, D1, M2, D2, cv::Size(imgwidth, imgheight), R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY,
    		0, cv::Size(640,480), &roi1, &roi2 );

    //compute rect maps
    cv::initUndistortRectifyMap(M1, D1, R1, P1, cv::Size(imgwidth, imgheight), CV_16SC2, this->LeftRectMap1, this->LeftRectMap2);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, cv::Size(imgwidth, imgheight), CV_16SC2, this->RightRectMap1, this->RightRectMap2);

	f = P1.at<double>(0,0);
	Tx = T.at<double>(0,0)*0.0254/(6*1.0e-6);
	Ty = T.at<double>(0,1)*0.0254/(6*1.0e-6);
	Tz = T.at<double>(0,2)*0.0254/(6*1.0e-6);
	cx = P2.at<double>(0,2);
	cy = P2.at<double>(1,2);

	return 0;
}


int Rectifier::rectifyImages(cv::Mat& leftImgFrame, cv::Mat& rightImgFrame)
{

	cv::Mat leftDummyImage;
	cv::remap(leftImgFrame, leftDummyImage, this->LeftRectMap1, this->LeftRectMap2, CV_INTER_LINEAR);
    leftImgFrame = leftDummyImage;

    cv::Mat rightDummyImage;
    cv::remap(rightImgFrame, rightDummyImage, this->RightRectMap1, this->RightRectMap2, CV_INTER_LINEAR);
    rightImgFrame = rightDummyImage;

	return 0;

}


void Rectifier::getCameraParameters(cv::Mat& Qin, cv::Mat& Rin, cv::Mat& Tin, cv::Mat& R1in, cv::Mat& P1in,
		cv::Mat& R2in, cv::Mat& P2in, cv::Mat& M1in, cv::Mat& D1in, cv::Mat& M2in, cv::Mat& D2in,
		double& Txin, double& Tyin, double& Tzin, double& fin, double& cxin, double& cyin)
{

    Qin = this->Q;
    Rin = this->R;
    Tin = this->T;
    R1in = this->R1;
    P1in = this->P1;
    R2in = this->R2;
    P2in = this->P2;
    M1in = this->M1;
    D1in = this->D1;
    M2in = this->M2;
    D2in = this->D2;
    Txin = this->Tx;
    Tyin = this->Ty;
    Tzin = this->Tz;
    fin = this->f;
    cxin = this->cx;
    cyin = this->cy;

}

void Rectifier::getCameraParameters(cv::Mat& Qin)
{
    Qin = this->Q;
}


void Rectifier::getCameraParameters(cv::Mat& Rin, cv::Mat& Tin, cv::Mat& M1in, cv::Mat& D1in, cv::Mat& M2in, cv::Mat& D2in) {
    Rin = this->R;
    Tin = this->T;
    M1in = this->M1;
    D1in = this->D1;
    M2in = this->M2;
    D2in = this->D2;
}

void Rectifier::getCameraParameters(double & cx_left,double &  cy_left,double &  cx_right,double &  cy_right,double &  f_left,double &  f_right,double &  Tx,double &  Ty,double &  Tz) {
	cx_left = this->M1.at<double>(0,2);
	cy_left = this->M1.at<double>(1,2);
	cx_right = this->M2.at<double>(0,2);
	cy_right = this->M2.at<double>(1,2);
	f_left = this->M1.at<double>(0,0);
	f_right = this->M2.at<double>(0,0);
	Tx = this->Tx;
	Ty = this->Ty;
	Tz = this->Tz;
}
 
