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
acqStep (3), imgNum(0)
{
	boardSize = Size(boardWidth, boardHeight);
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

int Halo::capture3Dcloud()
{
	return 0;
}


int Halo::calib()
{
	// -- STEP 1: Detect checkerboard in iL, iR and vT --
	
	// Capture images
	Mat iL, iR, dT, vT, cT;
	vector<vector<Point2f> > imagePointsL(numberBoards), imagePointsR(numberBoards), imagePointsT(numberBoards);
	vector<vector<Point3f> > objectPoints(numberBoards);
	vector<Mat> savedDepth, savedConf;
	
	// Usefull variables
	int retVal;
	int successes = 0;
	int step, frame = 0;
	bool foundT = false, foundR = false, foundL = false;
	
	
	// Capture numberBoards images
	while(successes < numberBoards) {
		
		// Capture images
		retVal = captureAllRectifiedImages(iL, iR, dT, vT, cT);
	
		if((frame++ % acqStep)==0){
			// Find chessboard corners:
			foundT = findChessboardCorners(vT, boardSize, imagePointsT[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			if (foundT) {
				foundL = findChessboardCorners(iL, boardSize, imagePointsL[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
				if (foundL) {
					foundR = findChessboardCorners(iR, boardSize, imagePointsR[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
				}
			}

			// If sth found
			if(foundL && foundR && foundT){	
				// Draw it if applicable
				drawChessboardCorners(iL, boardSize, Mat(imagePointsL[successes]), foundL);
				drawChessboardCorners(iR, boardSize, Mat(imagePointsR[successes]), foundR);
				drawChessboardCorners(vT, boardSize, Mat(imagePointsT[successes]), foundT);
				
				// Save depth and conf map of the ORF
				savedDepth.push_back(dT.clone());
				cvtColor(cT, cT, CV_BGR2GRAY);
				savedConf.push_back(cT.clone());
				
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
	}

	// -- STEP 2: For each point, compute 3D coordinates from stereo cameras and ORF cameras --
	
	// Variable declaration
	vector<Point3d> pointcloudOM, pointcloudORF, pointcloudTot;
	SimpleTriangulator OMtriangle(stereo.intrinsicMatrixL, stereo.projMatrixR);
	SimpleTriangulator ORFtriangle(orf.intrinsicMatrix);
	Point3d newPointORF, newPointOM;
	Point3d prevPointOM, prevPointORF;
	
	// For each point on each checkerboard checkerboard
	for (int i=0; i<numberBoards; i++) {
		for (int j=0; j<boardSize.area(); j++) {
			
			// If confidency of ORF is good enough
			if ((short)(savedConf[i].at<uchar>(imagePointsT[i][j].y, imagePointsT[i][j].x)) > THRESH_ORF_CONF) {
				
				// Triangulate and add stereo points
				newPointOM = OMtriangle.triangulate((double)imagePointsL[i][j].x, (double)imagePointsR[i][j].x, (double)imagePointsL[i][j].y);
				newPointOM = Point3d(newPointOM.x/1000, newPointOM.y/1000, newPointOM.z/1000); // convert in meters
				pointcloudOM.push_back(newPointOM);
				
				// Compute ORF coordinates (in meters) with depth map and inverting the pinhole equation
				double z = ((savedDepth[i].at<unsigned short>(imagePointsT[i][j].y, imagePointsT[i][j].x)>>2) & 0x3FFF)  * 0.00061 / 1.15;
				newPointORF = ORFtriangle.invTriangulate(imagePointsT[i][j].x, imagePointsT[i][j].y, z);
				pointcloudORF.push_back(newPointORF);
				
				// Print results in case of debug
// 	 			DEBUG<<"X1: "<<newPointOM.x<<"\tX2: "<<newPointORF.x<<"\tY1: "<<newPointOM.y<<"\tY2: "<<newPointORF.y<<"\tZ1: "<<newPointOM.z<<"\tZ2: "<<newPointORF.z<<endl;
// 				DEBUG<<"Interpoint ["<<i+1<<","<<j+1<<"]: Distance OM: "<<sqrt(pow((newPointOM.x - prevPointOM.x), 2) + pow((newPointOM.y - prevPointOM.y), 2) + pow((newPointOM.y - prevPointOM.y), 2))<<"m\tand distance ORF: "<<sqrt(pow((newPointORF.x - prevPointORF.x), 2) + pow((newPointORF.y - prevPointORF.y), 2) + pow((newPointORF.y - prevPointORF.y), 2))<<"m."<<endl;
// 				prevPointOM = newPointOM;
// 				prevPointORF = newPointORF;
			}
		}
	}
	
	
	// Display 3D results
	// Show camera invert XY axes but we should invert T (because it corresponds to translation from right to left) -> nothing to do
// 	visualizerShowCamera(stereo.rotMatrixL, Vec3f(0,0,0), 255,0,0,0.02,"Left or ORF camera"); 
// 	visualizerShowCamera(stereo.rotMatrixR, Vec3f(stereo.T.at<double>(0)/1000, stereo.T.at<double>(1)/1000, stereo.T.at<double>(2)/1000), 0,0,255,0.02,"Right camera");
// 	pointcloudTot.insert(pointcloudTot.end(), pointcloudOM.begin(), pointcloudOM.end());
// 	pointcloudTot.insert(pointcloudTot.end(), pointcloudORF.begin(), pointcloudORF.end());
// 	RunVisualization(pointcloudTot);
	
	// -- STEP 3: Minimize the distances between them to find the MLT rotation matrix --
	
	// Test RANSAC accuracy
	pointcloudOM.push_back(Point3d(0.0,0.0,0.1));
	pointcloudORF.push_back(Point3d(10.0, 0.4, 0.1));

	
	// Transfer key points from pointcloud to vector<Eigen::Vector3d>>
	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pointT, pointS, pointT2, pointS2;
	for (int i=0; i<pointcloudORF.size(); i++) {
		pointT.push_back(Vector3d(pointcloudORF[i].x, pointcloudORF[i].y, pointcloudORF[i].z));
		pointS.push_back(Vector3d(pointcloudOM[i].x, pointcloudOM[i].y, pointcloudOM[i].z));
	}
	
	// Compute absolute orientation
	AbsoluteOrientation abs;
// 	pointT2 = pointT;
// 	pointS2 = pointS;
	Matrix4d M = abs.orientationRANSAC(pointT, pointS);
	DEBUG<<"M:\n"<<M<<endl;
// 	M = abs.absoluteOrientation(pointT2, pointS2);
// 	DEBUG<<"M "<<M<<endl;
	
	// -- STEP 4: Fill each rototranslation matrices --
	
	Mat MLT = Mat::zeros(4,4,CV_64F);
	Mat MLR = Mat::zeros(4,4,CV_64F);
	Mat MRL = Mat::zeros(4,4,CV_64F);
	Mat MRT = Mat::zeros(4,4,CV_64F);
	Mat MTR = Mat::zeros(4,4,CV_64F);
	Mat MTL = Mat::zeros(4,4,CV_64F);
	MTL = Mat(M.rows(), M.cols(), CV_64F, M.data()).t();
	MLT = MTL.inv();
	Matrix4d M1;
	M1(0,0) = stereo.R.at<double>(0,0);
	M1(1,0) = stereo.R.at<double>(0,1);
	M1(2,0) = stereo.R.at<double>(0,2);
	M1(3,0) = stereo.T.at<double>(0)/1000;
	M1(0,1) = stereo.R.at<double>(1,0);
	M1(1,1) = stereo.R.at<double>(1,1);
	M1(2,1) = stereo.R.at<double>(1,2); 
	M1(3,1) = stereo.T.at<double>(1)/1000;
	M1(0,2) = stereo.R.at<double>(2,0);
	M1(2,1) = stereo.R.at<double>(2,1);
	M1(2,2) = stereo.R.at<double>(2,2);
	M1(3,2) = stereo.T.at<double>(2)/1000;
	M1(0,3) = 0.0;
	M1(1,3) = 0.0;
	M1(2,3) = 0.0;
	M1(3,3) = 1.0;
	MLR = Mat(M1.rows(), M1.cols(), CV_64F, M1.data());
	MRL = MLR.inv();
	MRT = MRL * MLT;
	MTR = MRT.inv();
	
	DEBUG<<"MLT: "<<MLT<<"\nMLR: "<<MLR<<"\nMRL: "<<MRL<<"\nMRT: "<<MRT<<"\nMTR: "<<MTR<<"\nMTL: "<<MTL<<endl;
	
	return 0;
	
	/* COMMENTS: 
	 * - Triangulation should be more accurate, using Hartley and Sturm Triangulation linear methods
	 * - The ORF depth map hsould be calibrated more accurately and one should understand where the coefficient 1.15 comes from
	 * - One should verify if the ORF depth map gives Z or radial depth and ameliorate the coordinates computation.
	 */
}