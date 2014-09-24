/*! 
* 	\file    orfMeasdepth.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Example showing how to capture ORF images and compute the depth for a given point in the picture
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

// Project libs
#include "../main.h"
#include "../utils.h"
#include "../defines.h"
#include "../halo.h"
#include "../triangulator.h"

using namespace std;
using namespace cv;


// Allocate variables
ORF orf;

// Create a CTRL-C handler
void exit_handler(int s) {
	orf.close();
	exit(1);
}

// Create a mouse clicks handler
int x_img, y_img;
void mouse_handler(int event, int x, int y, int flags, void* userdata) {
	if  ( event == EVENT_LBUTTONDOWN ) {
		x_img = x;
		y_img = y;
	}
}

// Main program
int main(int argc, char **argv) {
	
	init();
	
	// For catching a CTRL-C
	signal(SIGINT,exit_handler);
	
	// Initialize
	int retVal = orf.init();
	if (retVal !=0 )
		exit_handler(0);
	
	// Initialize mouse clicks
	x_img = 0;
	y_img = 0;
	namedWindow("Depth", 1);
	namedWindow("Intensity", 1);
	namedWindow("Confidency", 1);
	setMouseCallback("Depth", mouse_handler, NULL);
	setMouseCallback("Intensity", mouse_handler, NULL);
	setMouseCallback("Confidency", mouse_handler, NULL);
	
	// Main loop
	while(true) {
		// Capture orf images
		Mat dT, iT, cT;
		TimeStamp t;
		retVal = orf.captureRectifiedOrf(dT, iT, cT, t);
		if (retVal !=0 )
			exit_handler(0);
		
		// Create a triangulator
		OrfTriangulator triangle(orf.intrinsicMatrix);
		
		// Create a point where mouse has clicked
		Point p(x_img, y_img);
		
		// Triangulate this point
		Point3d P = triangle.triangulateOrf(x_img, y_img, ((dT.at<unsigned short>(p.y, p.x)>>2) & 0x3FFF)*0.00061);
		
		// Compute the depth from depth map
		char buffX[10], buffY[10], buffZ[10];
		sprintf(buffX, "%.3f", P.x);
		sprintf(buffY, "%.3f", P.y);
		sprintf(buffZ, "%.3f", P.z);
		string disp = String("X: ") + buffX + String("m  Y: ") + buffY + String("m  Z: ") + buffZ + String("m");
		
		// Add the point and the depth value to the images
		Mat dT_txt(dT.size(), CV_8UC3), iT_txt(iT.size(), CV_8UC3), cT_txt(cT.size(), CV_8UC3);
		cvtColor(dT.clone(), dT_txt, CV_GRAY2RGB);
		cvtColor(iT, iT_txt, CV_GRAY2RGB);
		cvtColor(cT, cT_txt, CV_GRAY2RGB);
		circle (dT_txt, p, 5, Scalar(0, 0, 255), 3);
		circle (iT_txt, p, 5, Scalar(0, 0, 255), 3);
		circle (cT_txt, p, 5, Scalar(0, 0, 255), 3);
		putText(dT_txt, disp, Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 0, 0), 1, CV_AA);
		putText(iT_txt, disp, Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 0, 0), 1, CV_AA);
		putText(cT_txt, disp, Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 0, 0), 1, CV_AA);
		
		// Display images
		imshow("Depth", dT_txt);
		imshow("Intensity", iT_txt);
		imshow("Confidency", cT_txt);
		
		// Handle pause/unpause and ESC
		int c = cvWaitKey(1);
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
			break;
		}
	}
	
 	// Close camera
  	orf.close();
	
	return 0;
}