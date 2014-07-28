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
#include "utils.h"
#include "defines.h"
#include "camera.h"
#include "orf.h"


using namespace std;
using namespace cv;
using namespace orf;
using namespace cam;


int main(int argc, char **argv) {
	
	intro();

	// Create
	ORF tof;
	
	// Initialize
	tof.initOrf();
	//stereo.initTwoCameras();
	

	while(true) {
		//Mat leftNewImageFrame, rightNewImageFrame;
		//cam2.captureTwoImages(leftNewImageFrame, rightNewImageFrame, &cam2.rightImgNum, &cam2.leftImgNum, flag);

		// Capture rectified ToF images
		Mat depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame;
		int retVal = tof.captureOrf(depthNewImageFrame, visualNewImageFrame, confidenceNewImageFrame);
		if (retVal==-1)
			return 0;
		
		// Capture non rectified ToF images
		Mat depthNewImageFrame2, visualNewImageFrame2, confidenceNewImageFrame2;
		retVal = tof.captureRectifiedOrf(depthNewImageFrame2, visualNewImageFrame2, confidenceNewImageFrame2);
		if (retVal==-1)
			return 0;
		
		imshow("Depth", depthNewImageFrame);
		//imshow("Intensity", visualNewImageFrame);
		//imshow("Confidency", confidenceNewImageFrame);
		
		imshow("DepthR", depthNewImageFrame2);
		//imshow("IntensityR", visualNewImageFrame2);
		//imshow("ConfidencyR", confidenceNewImageFrame2);
		
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