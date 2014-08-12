/*! 
* 	\file    timeStamp.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
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
	initTime = timeInit.tv_sec*1000 + timeInit.tv_usec/1000;
}

TimeStamp::~TimeStamp() 
{}

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

bool TimeStamp::isSynchro(TimeStamp t)
{
	if (isRunning==true) {
		return false;
	} else {
		int proc1 = t.getProcTime();
		int mean1 = t.getMeanTime();
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
	
        cout<<"\n=============== SwissRanger SR4000 Acquisition Software ===============\n"<<endl;
	cout<<"This software allows the user to take and save visual images and depth"<<endl;
	cout<<"information given by a SwissRanger SR4000 connected by ethernet with"<<endl;
	cout<<"a static IP."<<endl<<endl;
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
	cout<<"=======================================================================\n"<<endl;
	gettimeofday(&timeValInit, NULL);
}


