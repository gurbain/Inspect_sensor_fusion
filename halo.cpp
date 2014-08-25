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
// using namespace pcl;

Halo::Halo() :
isCamOpen(false), isOrfOpen(false),
imgWidth(640), imgHeight(480), 
boardWidth (6), boardHeight (11),
numberBoards (1), squareSize (250),
acqStep (5), imgNum(0)
{
	boardSize = Size(boardWidth, boardHeight);
	rect.calcRectificationMaps(this->imgWidth, this->imgHeight, "/opt/GogglesOptics/Calib_Params/defaultSet");
}

Halo::~Halo()
{}

int Halo::init()
{
	// We are capturing images
	this->load_image = false;
	
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
	orf.initOrf(directory);
	stereo.initTwoCameras(directory);

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
	
}

int Halo::close()
{
	int retVal;
	
	if (!load_image) {
		if (isOrfOpen) {
			retVal = orf.closeOrf();
		}
		if (isCamOpen){
			retVal = stereo.closeTwoCameras();
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
		retVal = orf.captureRectifiedOrf(dT, vT, cT, dummy, this->load_num);
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
			retVal = orf.captureRectifiedOrf(dT, vT, cT, tsORF);
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


int Halo::capture3Dcloud()
{
	return 0;
}


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
	Mat *essai;
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
				essai = &iL;
				drawChessboardCorners(iL, boardSize, Mat(imagePointsL[successes]), foundL);
				drawChessboardCorners(iR, boardSize, Mat(imagePointsR[successes]), foundR);
				drawChessboardCorners(vT, boardSize, Mat(imagePointsT[successes]), foundT);
				
				// Add point
				objectPoints[successes] = create3DChessboardCorners(boardSize, squareSize);
				successes++;
				INFO<<"Checkerboard found : "<<successes<<" numImg: "<<this->load_num<<endl; 
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
	circle(*essai, imagePointsL[0][0], 5, Scalar(255,0,0));
	circle(*essai, imagePointsL[0][1], 5, Scalar(0,255,0));
	circle(*essai, imagePointsL[0][2], 5, Scalar(0,0,255));
	circle(*essai, imagePointsL[0][3], 5, Scalar(255,255,0));
	imshow("Reperage", *essai);
	cvWaitKey(0);
	
	// Save data
	FileStorage storage("test.xml", FileStorage::WRITE);
	storage<<"imagePointsL"<<imagePointsL;
	storage<<"imagePointsR"<<imagePointsR;
	storage<<"imagePointsT"<<imagePointsT;
	storage.release();	

	// STEP 2 :: For each point, compute depth from stereo cameras in L reference
	
	// Triangulate stereo points
	cv::Mat distcoeff;
	vector<KeyPoint> correspRPt;
	vector<Point3d> pointcloud;
	Mat cloudp;
// 	vector<KeyPoint> ptSetL, ptSetR;
// 	PointsToKeyPoints(imagePointsL[0], ptSetL);
// 	PointsToKeyPoints(imagePointsR[0], ptSetR);
// 	Matx34d M1(rect.R1.at<double>(0,0), rect.R1.at<double>(0,1), rect.R1.at<double>(0, 2), 0,
// 		   rect.R1.at<double>(1,0), rect.R1.at<double>(1,1), rect.R1.at<double>(1, 2), 0,
// 		   rect.R1.at<double>(2,0), rect.R1.at<double>(2,1), rect.R1.at<double>(2, 2), 0);
// 	Matx34d M2(rect.R2.at<double>(0,0), rect.R2.at<double>(0,1), rect.R2.at<double>(0, 2), rect.T.at<double>(0, 0),
// 		   rect.R2.at<double>(1,0), rect.R2.at<double>(1,1), rect.R2.at<double>(1, 2), rect.T.at<double>(0, 1),
// 		   rect.R2.at<double>(2,0), rect.R2.at<double>(2,1), rect.R2.at<double>(2, 2), rect.T.at<double>(0, 2));
// 	
	cout<<"P1 "<<rect.P1<<endl;
	cout<<"P2 "<<rect.P2<<endl;
	//triangulatePoints(rect.P1, rect.P2, imagePointsL[0], imagePointsR[0], cloudp);
	SimpleTriangulator triangle(rect);
	Point3d newPoint;
	DEBUG<<"f "<<rect.f<<" Tx: "<<rect.Tx<<" cx: "<<rect.cx<<" cy: "<<rect.cy<<endl;
	for (int i=0; i<66; i++) {
		newPoint = triangle.triangulate((double)imagePointsL[0][i].y, (double)imagePointsR[0][i].y, (double)imagePointsL[0][i].x);
// 		newPoint.x = cloudp.at<float>(4*i);
// 		newPoint.y = cloudp.at<float>(4*i+1);
// 		newPoint.z = cloudp.at<float>(4*i+2);
		DEBUG<<imagePointsL[0][i].x<<" et "<<imagePointsR[0][i].x<<endl;
		DEBUG<<"Point "<<i+1<<": X="<<newPoint.x<<" ; Y="<<newPoint.y<<" ; Z="<<newPoint.z<<endl;
		pointcloud.push_back(newPoint);
	}

	
	//TriangulatePoints(ptSetL, ptSetR, rect.M1, rect.M1.inv(), rect.D1, M1, M2, pointcloud, correspRPt);
	//DEBUG<<"cloud "<<CloudPointsToPoints(pointcloud)<<endl;
	
	//visualizerShowCamera(rect.R1, Vec3f(M1(0, 3), M1(1,3), M1(2,3)), 255,0,0,0.2,"Left camera");
	//visualizerShowCamera(rect.R2, Vec3f(M2(0, 3), M2(1,3), M2(2,3)), 0,0,255,0.2,"Right camera");
	RunVisualization(pointcloud);
	// Comments on the function:
	//This function takes as parameters : set of Keypoint in the left camera system; set of keypoint in the right camera system;
	//intrinsics parameters from a single camera; their inverse; Distorsion correction for a single camera; the R|T for left camera;
	//the R|T for right camera (as we consider left camera as the origin of the world, we assume that T is 0 for it. To be coherent with the 
	//paper, those one has been called M and rect.M should be called K...
	
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
}
