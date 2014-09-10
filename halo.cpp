/*! 
* 	\file    orf.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Sources for Halo class
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "halo.h"

using namespace std;

Halo::Halo() :
isCamOpen(false), isOrfOpen(false),
imgWidth(640), imgHeight(480), imgNum(0)
{}

Halo::~Halo()
{}

int Halo::init()
{
	// We are capturing images
	this->load_image = false;
	
	int retVal;
	
	retVal = orf.init();
	if (retVal!=0) {
		orf.close();
		return -1;
	}
	isOrfOpen = true;
	retVal = stereo.init();
	if (retVal!=0) {
		orf.close();
		stereo.close();
		return -1;
	}
	retVal = stereo.start();
	if (retVal!=0) {
		orf.close();
		stereo.close();;
		return -1;
	}
	isCamOpen = true;
	
	return 0;
}

int Halo::init(string directory)
{	
	// We are using saved data
	this->load_image = true;

	// Verify the directory and set the name
	struct stat st;;
	if(stat(directory.c_str(),&st) != 0) {
		ERROR<<"The directory to load images does not exist!"<<endl;
		return -1;
	}
	this->load_directory = directory;
	orf.init(directory);
	stereo.init(directory);

	// Load the timestamp
	string tsOM = directory + "/timestampOpticsMount.txt";
	string tsORF = directory + "/timestampORF.txt";
	if(stat(tsOM.c_str(),&st) != 0) {
		ERROR<<"The OpticsMount timestamp file does not exist !"<<endl;
		return -1;
	}
	if(stat(tsORF.c_str(),&st) != 0) {
		ERROR<<"The ORF timestamp file does not exist !"<<endl;
		return -1;
	}
	
	string line;
	this->omFile.open(tsOM.c_str());
	this->orfFile.open(tsORF.c_str());
	
	if (this->omFile.is_open()) {
		for(int i=0; i<4; i++) {
			getline(this->omFile,line);
		}
	} else {
		ERROR<<"Unable to open OpticsMount timstamp file"<<endl;
		return -1;
	}
	if (orfFile.is_open()) {
		for(int i=0; i<4; i++) {
			getline(this->orfFile,line);
		}
	} else {
		ERROR<<"Unable to open  timestamp file"<<endl;
		return -1;
	}
	
	isCamOpen = true;
	isOrfOpen = true;
	this->load_num = 0;
	
	return 0;	
}

int Halo::close()
{
	int retVal;
	
	if (!load_image) {
		if (isOrfOpen) {
			retVal = orf.close();
		}
		if (isCamOpen){
			retVal = stereo.close();
		}
	}
	
	return 0;
}


int Halo::captureAllImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag)
{
	int retVal;
	int flags;

	// Create TimeStamp
	TimeStamp tsOM, tsORF, dummy;
	
	// If needed, load timestamps
	if (load_image) {
		string line;
		int retVal;
		if (this->orfFile.is_open()) {
			getline(this->orfFile,line);
			if (line=="")
				return -1;
			tsORF.load(line);
		}
		if (this->omFile.is_open()) {
			getline(this->omFile,line);
			if (line=="")
				return -1;
			tsOM.load(line);
		}
	}
	
	// If we load from file
	if (load_image) {
		// Load ORF images
		retVal = orf.captureOrf(dT, vT, cT, dummy, this->load_num);
		if (retVal!=0)
			return -1;
	
		// Load OpticsMount images
		retVal = stereo.captureTwoImages(iL, iR, &stereo.rightImgNum, &stereo.leftImgNum, dummy, flags, this->load_num);
		if (retVal!=0)
			return -1;
		
		this->load_num++;
		
	// Else, if we capture images
	} else {
		// Capture non rectified ToF images
		if (isOrfOpen) {
			retVal = orf.captureOrf(dT, vT, cT, tsORF);
			if (retVal!=0)
				return -1;
		}

		// Capture frames from stereo cameras
		if (isCamOpen){
			retVal = stereo.captureTwoImages(iL, iR, &stereo.rightImgNum, &stereo.leftImgNum, tsOM, flags);
			if (retVal!=0)
				return -1;
		}
		
		imgNum++;
	}
	
	// Check synchronicity
	if (flag==SYNCHRONOUS) {
		int del = abs(tsORF.getMeanTime() - tsOM.getMeanTime());
		if (!tsOM.isSynchro(tsORF)) {
			DEBUG<<"Images "<<this->load_num<<" are not synchronous! Delay between stereo cams and orf cam is : "<<del<<" ms"<<endl;
			this->load_num++;
			return abs(del);
		}
		DEBUG<<"Images "<<this->load_num<<" are synchronous! Delay between stereo cams and orf cam is : "<<del<<" ms"<<endl;
	}
	
	return 0;
}


int Halo::captureAllRectifiedImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag)
{
	int retVal;
	int flags;

	// Create TimeStamp
	TimeStamp tsOM, tsORF, dummy;
	
	// If needed, load timestamps
	if (load_image) {
		string line;
		int retVal;
		if (this->orfFile.is_open()) {
			getline(this->orfFile,line);
			if (line=="")
				return -1;
			tsORF.load(line);
		}
		if (this->omFile.is_open()) {
			getline(this->omFile,line);
			if (line=="")
				return -1;
			tsOM.load(line);
		}
	}
	
	// If we load from file
	if (load_image) {
		// Load ORF images
		retVal = orf.captureRectifiedOrf(dT, vT, cT, dummy, this->load_num);
		if (retVal!=0)
			return -1;
	
		// Load OpticsMount images
		retVal = stereo.captureTwoRectifiedImages(iL, iR, dummy, this->load_num);
		if (retVal!=0)
			return -1;
		
		this->load_num++;
		
	// Else, if we capture images
	} else {
		// Capture non rectified ToF images
		if (isOrfOpen) {
			retVal = orf.captureRectifiedOrf(dT, vT, cT, tsORF);
			if (retVal!=0)
				return -1;
		}

		// Capture frames from stereo cameras
		if (isCamOpen){
			retVal = stereo.captureTwoRectifiedImages(iL, iR, tsOM);
			if (retVal!=0)
				return -1;
		}
		
		imgNum++;
	}
	
	// Check synchronicity
	if (flag==SYNCHRONOUS) {
		int del = abs(tsORF.getMeanTime() - tsOM.getMeanTime());
		if (!tsOM.isSynchro(tsORF)) {
			DEBUG<<"Images "<<this->load_num<<" are not synchronous! Delay between stereo cams and orf cam is : "<<del<<" ms"<<endl;
			this->load_num++;
			return abs(del);
		}
		DEBUG<<"Images "<<this->load_num<<" are synchronous! Delay between stereo cams and orf cam is : "<<del<<" ms"<<endl;
	}
	
	return 0;
}

int Halo::saveAllImages()
{
	if (load_image) {
		ERROR<<"You cannot save images loaded from hard drive!"<<endl;
		return -1;
	}
	
	this->stereo.saveTwoImages();
	this->orf.saveOrf();
	
	return 0;
}

int Halo::capture3Dcloud(vector<Point3d>& pointcloud, vector<Vec3b>& rgbcloud)
{
	// Capture images to merge
	//Mat iL, iR, dT, iT, cT;
	//this->captureAllRectifiedImages(iL, iR, dT, iT, cT);
	
	// If we load from file
	if (load_image) {
		orf.capture3Dcloud(pointcloud, rgbcloud, this->load_num, ORF_CLOUD_DOWNSAMPLING);
		this->load_num++;
	}
	
	
	return 0;
}


int Halo::calib(string filename)
{
	haloCalib(filename);
}

// For each point:

// Compute sigmaT from Ct

// Compute sigmaS = variance from the second order neighborhood around p

// sigmaW = min(sigmaT, sigmaS)

// 