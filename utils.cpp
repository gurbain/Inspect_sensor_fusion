/*! 
* 	\file    timeStamp.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Sources for software tools
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "utils.h" 

using namespace std;

TimeStamp::TimeStamp()
{
	isRunning = false;
	timeInit = timeValInit;
	meanTime = 0;
	procTime = 0;
	initTime = 0;//timeInit.tv_sec*1000 + timeInit.tv_usec/1000;
}

TimeStamp::~TimeStamp() 
{}

int TimeStamp::load(string line_)
{
	// Parse line
	string proctime, meantime;
	istringstream iss(line_);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
	
	// Set TimeStamp
	int proc = atoi(tokens.at(3).c_str());
	int mean = atoi(tokens.at(5).c_str());
	this->setTime(mean, proc);
// 	cout<<"PROC "<<this->procTime<<endl;
// 	cout<<"MEAN "<<this->meanTime<<endl;
// 	cout<<"START "<<this->startTime<<endl;
// 	cout<<"STOP "<<this->stopTime<<endl;
	
	return 0;
}

void TimeStamp::start()
{
	if (isRunning==true) {
		DEBUG<<"You must stop a time stamp defore starting it again"<<endl;
	} else {
		isRunning = true;
		gettimeofday(&timeBegin, NULL);
	}
	
}

void TimeStamp::stop()
{
	if (isRunning==false) {
		DEBUG<<"You must start a time stamp defore stopping it"<<endl;
	} else {
		gettimeofday(&timeEnd, NULL);
		startTime = timeBegin.tv_sec*1000 + timeBegin.tv_usec/1000 - initTime;
		stopTime = timeEnd.tv_sec*1000 + timeEnd.tv_usec/1000 - initTime;
		procTime = stopTime - startTime;
		meanTime = (startTime + stopTime)/2;
		isRunning = false;
	}
}

int TimeStamp::getProcTime()
{
	if (isRunning==true) {
		return -1;
	} else {
		return procTime;
	}
}

int TimeStamp::getMeanTime()
{
	if (isRunning==true) {
		return -1;
	} else {
		return meanTime;
	}
}

int TimeStamp::getStartTime()
{
	if (isRunning==true) {
		return -1;
	} else {
		return startTime;
	}
}

int TimeStamp::getStopTime()
{
	if (isRunning==true) {
		return -1;
	} else {
		return stopTime;
	}
}

int TimeStamp::setTime(int meanTime_, int procTime_)
{
	this->procTime = procTime_;
	this->meanTime = meanTime_;
	this->startTime = meanTime_ - int(procTime_/2);
	this->stopTime = meanTime_ + int(procTime_/2);	
}

bool TimeStamp::isSynchro(TimeStamp t_)
{
	if (isRunning==true) {
		return false;
	} else {
		int proc1 = t_.getProcTime();
		int mean1 = t_.getMeanTime();
		if (proc1==-1 || mean1==-1) {
			return false;
		} else {
			if (abs(mean1-meanTime)<DELTATMIN && abs(mean1+proc1/2-meanTime+procTime/2)<DELTATMAX && abs(meanTime+procTime/2-mean1+proc1/2)<DELTATMAX)
				return true;
		}
	}
	return false;
}

void init()
{
        cout<<"\n===================== Inspect Linux Software =====================\n"<<endl;
	cout<<"This standalone software aims at data fusion from an ethernet"<<endl;
	cout<<"SwissRanger SR4000 Optical Range Finder and a stereo Optics Mount"<<endl;
	cout<<"(VERTIGO). A thermocamshould be added afterwards. The main features"<<endl;
	cout<<"are calibration, feature matching, 3D registration and fusion"<<endl;
	cout<<"and speed, inertia and pose estimation of a moving target\n"<<endl;
	cout<<"PID of the process: "<<(int)getpid()<<endl;
	cout<<"Compiled  On: \t"<<__DATE__<<" at "<<__TIME__<<" UTC"<<endl;
	cout<<"          With:\tGCC: "<< __VERSION__<<endl;
	struct utsname ver;
	uname(&ver);
	cout<<"\t\tUbuntu: Trusty 14.04 - kernel "<<ver.release<<endl;
	cout<<"\t\tOpenCV: "<<CV_VERSION<<endl;
	cout<<"Author: "<<__AUTHOR__<<endl;
	cout<<"Space Systems Lab (SSL) - INSPECT project"<<endl;
	cout<<"July 2014 - Massachussets Institute of Technology (MIT)\n"<<endl;
	cout<<"=====================================================================\n"<<endl;
	gettimeofday(&timeValInit, NULL);
}

vector<cv::Point3f> create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
	vector<cv::Point3f> corners;
 
	for( int i = 0; i < boardSize.height; i++ ) {
		for( int j = 0; j < boardSize.width; j++ ) {
			corners.push_back(cv::Point3f(float(j*squareSize),
			float(i*squareSize), 0));
		}
	}
	return corners;
}


