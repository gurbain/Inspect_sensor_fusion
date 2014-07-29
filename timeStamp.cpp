/*! 
* 	\file    timeStamp.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Sources for timeStamp utilities
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "timeStamp.h" 


using namespace std;


TimeStamp::TimeStamp()
{
	isRunning = false;
	meanTime = 0;
	procTime = 0;
	gettimeofday(&timeBegin, NULL);
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
		int init = 0;
		int start = timeBegin.tv_sec*1000 + timeBegin.tv_usec/1000 - init;
		int end = timeEnd.tv_sec*1000 + timeEnd.tv_usec/1000 - init;
		procTime = end - start;
		meanTime = (start + end)/2;
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

bool TimeStamp::isSynchro(TimeStamp t)
{
	if (isRunning==true) {
		return false;
	} else {
		int proc1 = t.getProcTime();
		int mean1 = t.getProcTime();
		if (proc1==-1 || mean1==-1) {
			return false;
		} else {
			if (abs(mean1-meanTime)<DELTATMIN && abs(mean1+proc1/2-meanTime+procTime/2)<DELTATMAX && (meanTime+procTime/2-mean1+proc1/2)<DELTATMAX)
				return true;
		}
	}
	return false;
}

