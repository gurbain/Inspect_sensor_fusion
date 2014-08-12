/*! 
* 	\file    orf.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
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
imgWidth(640), imgHeight(480), 
boardWidth (6), boardHeight (11),
numberBoards (1), squareSize (250),
acqStep (5), imgNum(0)
{
	boardSize = Size(boardWidth, boardHeight);
}

Halo::~Halo()
{}

int Halo::init()
{
	int retVal;
	
	retVal = orf.initOrf();
	if (retVal!=0) {
		orf.closeOrf();
		return -1;
	}
	isOrfOpen = true;
	retVal = stereo.initTwoCameras();
	if (retVal!=0) {
		orf.closeOrf();
		stereo.closeTwoCameras();
		return -1;
	}
	retVal = stereo.startTwoCameras();
	if (retVal!=0) {
		orf.closeOrf();
		stereo.closeTwoCameras();;
		return -1;
	}
	isCamOpen = true;
	//rect.calcRectificationMaps(imgWidth, imgHeight, CALIB_DIR);
	
	return 0;
}

int Halo::close()
{
	int retVal;
	
	if (isOrfOpen) {
		retVal = orf.closeOrf();
	}
	if (isCamOpen){
		retVal = stereo.closeTwoCameras();
	}
	
	return 0;
}


int Halo::captureAllImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag)
{
	int retVal;
	int flags;

	// Create TimeStamp
	TimeStamp ts, tsr;
	
	// Capture non rectified ToF images
	if (isOrfOpen) {
		retVal = orf.captureRectifiedOrf(dT, vT, cT, tsr);
		if (retVal!=0) {
			return -1;
		}
	}
	
	// Capture frames from stereo cameras
	if (isCamOpen){
		retVal = stereo.captureTwoImages(iL, iR, &stereo.rightImgNum, &stereo.leftImgNum, ts, flags);
		if (retVal!=0) {
			INFO<<"c "<<imgNum<<endl;
			return -1;
		}
		
// 		retVal = rect.rectifyImages(iL, iR);
// 		if (retVal!=0) {
// 			DEBUG<<"Rectification error!"<<endl;
// 			return retVal;
// 		}
		
		if (flag==SYNCHRONOUS) {
			int del = abs(ts.getMeanTime() - tsr.getMeanTime());
			if (!ts.isSynchro(tsr)) {
				DEBUG<<"Images "<<imgNum<<" are not synchronous! Delay between stereo cams and orf cam is : "<<del<<" ms"<<endl;
				return -del;
			}
			//DEBUG<<"Images are synchronous! t1="<<tsr.getStartTime()<<" t2="<<tsr.getStopTime()<<" t3="<<ts.getStartTime()<<" t4="<<ts.getStopTime()<<endl;
		}
		imgNum++;
	}
	
	return 0;
}


int Halo::capture3Dcloud()
{
	return 0;
}

/*
int Halo::calib()
{
	// STEP 1 :: Detect checkerboard in iL, iR and vT
	
	// Capture images
	Mat iL, iR, dT, vT, cT;
	vector<vector<Point2f> > imagePointsL(numberBoards), imagePointsR(numberBoards), imagePointsT(numberBoards);
	vector<vector<Point3f> > objectPoints(numberBoards);
	
	// Usefull variables
	int retVal;
	int successes = 0;
	int step, frame = 0;
	bool foundT = false, foundR = false, foundL = false;
	
	// Capture images
	retVal = captureAllImages(iL, iR, dT, vT, cT);
	//INFO<<"images: "<<vT.size()<<","<<iL.size()<<","<<iR.size()<< " et boardSize: "<<boardSize<<" et imagePoints: "<<imagePointsT[0]<<";"<<imagePointsL[0]<<";"	<<imagePointsR[0]<<endl;
	
	// Capture numberBoards images
	while(successes < numberBoards) {
		if((frame++ % acqStep)==0){
			// Find chessboard corners:
			foundT = findChessboardCorners(vT, boardSize, imagePointsT[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			if (foundT) {
				foundL = findChessboardCorners(iL, boardSize, imagePointsL[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
				if (foundL) {
					foundR = findChessboardCorners(iR, boardSize, imagePointsR[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
				}
			}
			//INFO<<"Checkerboard found state : ["<<foundL<<", "<<foundR<<", "<<foundT<<" ]"<<endl; 

			// If sth found
			if(foundL && foundR && foundT){	
				// Draw it if applicable
				drawChessboardCorners(iL, boardSize, Mat(imagePointsL[successes]), foundL);
				drawChessboardCorners(iR, boardSize, Mat(imagePointsR[successes]), foundR);
				drawChessboardCorners(vT, boardSize, Mat(imagePointsT[successes]), foundT);
				
				// Add point
				objectPoints[successes] = create3DChessboardCorners(boardSize, squareSize);
				successes++;
				INFO<<"Checkerboard found : "<<successes<<endl; 
			}
		}
		
		// Show the result
		imshow("Calib left", iL);
		imshow("Calib right", iR);
		imshow("Calib orf", vT);

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
		
		// Get next image
		retVal = captureAllImages(iL, iR, dT, vT, cT);
	}
	
	INFO<<"yooh: "<<Mat(imagePointsR[0])<<endl;
	
	// STEP 2 :: For each point, compute depth from stereo cameras in L reference
	FileStorage storage("test.xml", FileStorage::WRITE);
	storage<<"imagePointsL"<<imagePointsL;
	storage<<"imagePointsR"<<imagePointsR;
	storage<<"imagePointsT"<<imagePointsT;
	storage.release();
	
	
	// For each point, acquire depth in T reference
	
	// Minimize the distances between them to find the MLT rotation matrix
	
	return 0;
}

vector<Point3f> Halo::create3DChessboardCorners(Size boardSize, float squareSize)
{
	vector<Point3f> corners;
 
	for( int i = 0; i < boardSize.height; i++ ) {
		for( int j = 0; j < boardSize.width; j++ ) {
			corners.push_back(cv::Point3f(float(j*squareSize),
			float(i*squareSize), 0));
		}
	}
	return corners;
}*/