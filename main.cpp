/*! 
* 	\file    main.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
* 	\date    July 2014
* 	\version 0.1
* 	\brief   Main program for optical range finder and stereo cameras acquisition
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

// Project libs
#include "main.h"
#include "utils.h"
#include "defines.h"
#include "camera.h"
#include "orf.h"
#include "timeStamp.h"


using namespace std;
using namespace cv;
using namespace orf;
using namespace cam;

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
// 	gettimeofday(&timeValInit, NULL);
// 	timeInit = timeValInit.tv_sec*1000 + timeValInit.tv_usec/1000;
}


int main(int argc, char **argv) {
	
	init();

	// Allocate variables
	ORF tof;
	
	// Initialize
	tof.initOrf();
	//stereo.initTwoCameras();

	while(true) {
		
		// Create TimeStamp
		TimeStamp ts, tsr;
		
		//Mat leftNewImageFrame, rightNewImageFrame;
		//cam2.captureTwoImages(leftNewImageFrame, rightNewImageFrame, &cam2.rightImgNum, &cam2.leftImgNum, flag);

		// Capture rectified ToF images
		Mat depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
		int retVal = tof.captureOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame, ts);
		if (retVal==-1)
			return 0;
		
		// Capture non rectified ToF images
		Mat depthNewImageFrame2, visualNewImageFrame2, confidenceNewImageFrame2;
		retVal = tof.captureRectifiedOrf(depthNewImageFrame2, visualNewImageFrame2, confidenceNewImageFrame2, tsr);
		if (retVal==-1)
			return 0;
		
		imshow("Depth", depthNewImageFrame);
		//imshow("Intensity", visualNewImageFrame);
		//imshow("Confidency", confidenceNewImageFrame);
		
		imshow("DepthR", depthNewImageFrame2);
		//imshow("IntensityR", visualNewImageFrame2);
		//imshow("ConfidencyR", confidenceNewImageFrame2);
		
		// Display timeStamps
		INFO<<"Time stamp non-synch: "<<ts.getProcTime()<<"ms of processing at "<<ts.getMeanTime()<<"ms"<<endl;
		INFO<<"Time stamp synch: "<<tsr.getProcTime()<<"ms of processing at "<<tsr.getMeanTime()<<"ms"<<endl;
		
		// Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if(c == 'p') {
			DEBUG<<"Acquisition is now paused"<<endl;
			c = 0;
			while(c != 'p' && c != 27){
				c = cvWaitKey(250);
			}
			DEBUG<<"Acquisition is now unpaused"<<endl;
		}
		if(c == 27) {
			DEBUG<<"Acquisition has been stopped by user"<<endl;
			return 0;
		}
 	}
 	
	return 0;
}