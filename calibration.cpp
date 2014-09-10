/*! 
* 	\file    calibration.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    September 2014
* 	\version 0.1
* 	\brief   Tools for ORF, Stereo Mount and Optics system calibration
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "calibration.h"

using namespace cv;
using namespace std;

int Calibration::orfCalib(string filename)
{
	// CV Matrix storage
	Mat dt, it, ct;
	vector<vector<Point2f> > imagePoints;
	vector<Point2f> corners;
	vector<vector<Point3f> > objectPoints;
	vector<Mat> rotationVectors;
	vector<Mat> translationVectors;
	Mat intrinsicMatrix, distorsionCoeffs;
	
	// Usefull variables
	int successes = 0;
	int num = 0;
	int step;
	bool found;
	
	// Capture first image
	TimeStamp t;
	int retVal = captureOrf(dt, it, ct, t);
	if (retVal!=0)
		return -1;
	
	
	// Capture ORF_NUMBER_BOARDS images
	INFO<<" ---- ORF CAM CALIBRATION:                                        ----"<<endl;
	INFO<<" ---- Each time a keyboard is detected, the image freezes.        ----"<<endl;
	INFO<<" ---- Press space to keep the image or any other key to throw it. ----"<<endl;
	INFO<<" ---- Press esc to quit and p to pause/unpause the calibration    ----"<<endl;
	while(successes < ORF_NUMBER_BOARDS) {
		// Find chessboard corners:
		found = findChessboardCorners(it, BOARD_SIZE, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

		// Display points if we find them
		if(found){
			drawChessboardCorners(it, BOARD_SIZE, Mat(corners), found);
		}
		
		// Show the result
		imshow("Calibration", it);

		// Handle keyboard
		int c = cvWaitKey(15);
		if (found) {
			c = cvWaitKey(0);
		}
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
		if (c == ' ' && found !=0) {
			// Add points
			imagePoints.push_back(corners);
			objectPoints.push_back(create3DChessboardCorners(BOARD_SIZE, SQUARE_SIZE));
			successes++;
			INFO<<"Checkerboard found : "<<successes<<endl;
		}
		
		// Get next image
		num++;
		int retVal = captureOrf(dt, it, ct, t, num);
		if (retVal!=0)
			return -1;
	}
	
	cout<<imagePoints.size()<<"  "<<imagePoints[1].size()<<"  "<<objectPoints.size()<<"  "<<objectPoints[0].size()<<endl;
	
	// Compute calibration matrixes
	double rms = calibrateCamera(objectPoints, imagePoints, it.size(), intrinsicMatrix, distorsionCoeffs, rotationVectors, translationVectors, 0|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
	INFO<<"ORF Calibration done! RMS reprojection error: "<<rms<<endl;

	// Save the intrinsics and distorsions
	FileStorage storage(filename, FileStorage::WRITE);
	storage<<"Intrinsicparameters"<<intrinsicMatrix;
	storage<<"Distortioncoefficients"<<distorsionCoeffs;
	storage.release();
	
	// Print saving info
	INFO<<"Calibration matrixes has been saved in "<<filename<<endl;
	
	return 0;
}


int Calibration::stereoCalib(string filename)
{
	// Capture images
	Mat iL, iR;
	vector<vector<Point2f> > imagePointsL, imagePointsR;
	vector<Point2f> cornersL, cornersR;
	vector<vector<Point3f> > objectPoints;
	
	Mat distorsionCoeffsL, distorsionCoeffsR;
	Mat rotMatrixL, rotMatrixR, projMatrixL, projMatrixR;
	Mat intrinsicMatrixL = Mat::eye(3, 3, CV_64F);
	Mat intrinsicMatrixR = Mat::eye(3, 3, CV_64F);
	Mat Q, R, T, E, F;

	// Usefull variables
	int retVal;
	int num = 0;
	int *dummy1, *dummy2, dummy3;
	int successes = 0;
	int step;
	bool foundR = false, foundL = false;
	
	// Capture images
	TimeStamp t;
	retVal = captureTwoImages(iL, iR, dummy1, dummy2, t, dummy3, 0);
	
	// Capture STEREO_NUMBER_BOARDS images
	INFO<<" ---- STEREO CAMS CALIBRATION:                                    ----"<<endl;
	INFO<<" ---- Each time a keyboard is detected, the image freezes.        ----"<<endl;
	INFO<<" ---- Press space to keep the image or any other key to throw it. ----"<<endl;
	INFO<<" ---- Press esc to quit and p to pause/unpause the calibration    ----"<<endl;
	while(successes < STEREO_NUMBER_BOARDS) {
		foundL = findChessboardCorners(iL, BOARD_SIZE, cornersL, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
		if (foundL) {
			foundR = findChessboardCorners(iR, BOARD_SIZE, cornersR, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
		}
		//INFO<<"Checkerboard found state : ["<<foundL<<", "<<foundR<<", "<<foundT<<" ]"<<endl; 

		// Display points if we find them
		if(foundL && foundR){
			drawChessboardCorners(iL, BOARD_SIZE, Mat(cornersL), foundL);
			drawChessboardCorners(iR, BOARD_SIZE, Mat(cornersR), foundR);
		}
		
		// Show the result
		imshow("Calib left", iL);
		imshow("Calib right", iR);

		// Handle keyboard
		int c = cvWaitKey(15);
		if (foundL && foundR) {
			c = cvWaitKey(0);
		}
		
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
		if (c == ' ' && foundL !=0 && foundR != 0) {
			// Add points
			imagePointsL.push_back(cornersL);
			imagePointsR.push_back(cornersR);
			objectPoints.push_back(create3DChessboardCorners(BOARD_SIZE, SQUARE_SIZE));
			successes++;
			INFO<<"Checkerboard found : "<<successes<<endl;
		}

		// Get next image
		num++;
		retVal = captureTwoImages(iL, iR, dummy1, dummy2, t, dummy3, num);
	}

	// Compute intrinsic and extrinsic calibration matrices
	double rms = stereoCalibrate(objectPoints, imagePointsL, imagePointsR, intrinsicMatrixL, distorsionCoeffsL, intrinsicMatrixR, distorsionCoeffsR, iL.size(), R, T, E, F, TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	INFO << "Optics Mount calibration done! RMS reprojection error: " << rms << endl;
	
	// Compute reprojection matrices
	Rect roi1, roi2;
	Mat RL, PL, RR, PR;
	stereoRectify(intrinsicMatrixL, distorsionCoeffsL, intrinsicMatrixR, distorsionCoeffsR, iL.size(), R, T, rotMatrixL, rotMatrixR, projMatrixL, projMatrixR, Q, CV_CALIB_ZERO_DISPARITY, 0, iL.size(), &roi1, &roi2);
	
	// Save the calibration parameters
	FileStorage storage(filename, FileStorage::WRITE);
	storage<<"intrinsicMatrixL"<<intrinsicMatrixL;
	storage<<"distorsionCoeffsL"<<distorsionCoeffsL;
	storage<<"projMatrixL"<<projMatrixL;
	storage<<"rotMatrixL"<<rotMatrixL;
	storage<<"intrinsicMatrixR"<<intrinsicMatrixR;
	storage<<"distorsionCoeffsR"<<distorsionCoeffsR;
	storage<<"projMatrixR"<<projMatrixR;
	storage<<"rotMatrixR"<<rotMatrixR;
	storage<<"R"<<R;
	storage<<"T"<<T;
	storage<<"E"<<E;
	storage<<"F"<<F;
	storage.release();
	
	return 0;
}


int Calibration::acqHaloCheckPoints(vector<vector<Point2f> >& imagePointsL, vector<vector<Point2f> >& imagePointsR, vector<vector<Point2f> >& imagePointsT, vector<Mat>& savedConf, vector<Mat>& savedDepth)
{
	// Capture images
	Mat iL, iR, dT, vT, cT;
	Mat dTSaved, cTSaved;
	vector<vector<Point3f> > objectPoints;
	vector<Point2f> cornersT, cornersL, cornersR;
	
	// Usefull variables
	int retVal;
	int successes = 0;
	int step, frame = 0;
	bool foundT = false, foundR = false, foundL = false;
	
	// Capture images
	retVal = captureAllRectifiedImages(iL, iR, dT, vT, cT);
	
	// Capture HALO_NUMBER_BOARDS images
	INFO<<" ---- HALO CAMS CALIBRATION:                                      ----"<<endl;
	INFO<<" ---- Each time a keyboard is detected, the image freezes.        ----"<<endl;
	INFO<<" ---- Press space to keep the image or any other key to throw it. ----"<<endl;
	INFO<<" ---- Press esc to quit and p to pause/unpause the calibration    ----"<<endl;
	while(successes < HALO_NUMBER_BOARDS) {
		frame++;
		// Find chessboard corners:
		foundT = findChessboardCorners(vT, BOARD_SIZE, cornersT, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
		if (foundT) {
			foundL = findChessboardCorners(iL, BOARD_SIZE, cornersL, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			if (foundL) {
				foundR = findChessboardCorners(iR, BOARD_SIZE, cornersR, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			}
		}

		// Display points if we find them
		if(foundL && foundR && foundT){
			drawChessboardCorners(iL, BOARD_SIZE, Mat(cornersL), foundL);
			drawChessboardCorners(iR, BOARD_SIZE, Mat(cornersR), foundR);
			dTSaved = dT.clone();
			cTSaved = cT.clone();
			drawChessboardCorners(dT, BOARD_SIZE, Mat(cornersT), foundT);
			drawChessboardCorners(cT, BOARD_SIZE, Mat(cornersT), foundT);
		}
		
		// Show the result
		imshow("Calib left", iL);
		imshow("Calib right", iR);
		imshow("Calib orf", vT);
		imshow("Calib orf d", dT);
		imshow("Calib orf c", cT);
		
		// Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if (foundL && foundR && foundT) {
			c = cvWaitKey(0);
		}
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
		if (c == ' ' && foundL !=0 && foundR != 0 && foundT !=0) {
			// Add points
			imagePointsL.push_back(cornersL);
			imagePointsR.push_back(cornersR);
			imagePointsT.push_back(cornersT);
					
			// Save depth and conf map of the ORF
			savedDepth.push_back(dTSaved.clone());
			savedConf.push_back(cTSaved.clone());
					
			// Add objects poitns
			objectPoints.push_back(create3DChessboardCorners(BOARD_SIZE, SQUARE_SIZE));
			successes++;
			INFO<<"Checkerboard found : "<<successes<<endl;
		}
		
		// Get next image
		retVal = captureAllRectifiedImages(iL, iR, dT, vT, cT);
	}
	
	return 0;
}


int Calibration::computeHalo3DPoints(vector<vector<Point2f> >& imagePointsL, vector<vector<Point2f> >& imagePointsR, vector<vector<Point2f> >& imagePointsT, vector<Mat>& savedConf, vector<Mat>& savedDepth, vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, vector<cv::Vec3b>& rgbcloudOM, vector<cv::Vec3b>& rgbcloudORF, Mat& RRL, Mat& TRL)
{
	// Load calibration matrices
	int retVal;
	Mat intrinsicMatrixL, intrinsicMatrixT, projMatrixR, rotMatrixR, rotMatrixL;
	FileStorage storage;
	string OMfilename = "OM_calib.xml";
	string ORFfilename = "ORF_calib.xml";
	
	retVal = storage.open(OMfilename, FileStorage::READ);
	if (retVal!=1) {
		INFO<<"Calibration file not found! Calibration needed!"<<endl;
		stereoCalib(OMfilename);
		retVal = storage.open(OMfilename, FileStorage::READ);
		if (retVal!=1) {
			ERROR<<"File cannot be open or read! Verify user rights"<<endl;
			return -1;
		}
	}
	storage["intrinsicMatrixL"]>>intrinsicMatrixL;
	storage["projMatrixR"]>>projMatrixR;
	storage["T"]>>TRL;
	storage["R"]>>RRL;
	storage["rotMatrixL"]>>rotMatrixR;
	storage["rotMatrixR"]>>rotMatrixL;
	storage.release();
	
	retVal = storage.open(ORFfilename, FileStorage::READ);
	if (retVal!=1) {
		INFO<<"Calibration file not found! Calibration needed!"<<endl;
		orfCalib(ORFfilename);
		retVal = storage.open(ORFfilename, FileStorage::READ);
		if (retVal!=1) {
			ERROR<<"File cannot be open or read! Verify user rights"<<endl;
			return -1;
		}
	}
	storage["Intrinsicparameters"]>>intrinsicMatrixT;
	storage.release();
	
	// Variable declaration
	vector<Point3d> pointcloudTot;
	Triangulator OMtriangle(intrinsicMatrixL, projMatrixR);
	InverseTriangulator ORFtriangle(intrinsicMatrixT);
	Point3d newPointORF, newPointOM, newPointORF2;
#ifdef	CALIB_DEBUG
	Point3d prevPointOM, prevPointORF;
	vector<cv::Vec3b> rgbcloudTot;
	vector<cv::Vec3b> col;
	col.push_back(Vec3b(255,0,0));
	col.push_back(Vec3b(255,120,120));
	col.push_back(Vec3b(0,255,0));
	col.push_back(Vec3b(0,255,150));
	col.push_back(Vec3b(0,0,255));
	col.push_back(Vec3b(120,120,255));
	col.push_back(Vec3b(0,255,255));
	col.push_back(Vec3b(100,200,255));
	col.push_back(Vec3b(255,0,255));
	col.push_back(Vec3b(255,100,200));
	col.push_back(Vec3b(100,100,100));
	col.push_back(Vec3b(250,250,250));
#endif
	
	// For each point on each checkerboard checkerboard
	for (int i=0; i<HALO_NUMBER_BOARDS; i++) {
		for (int j=0; j<BOARD_SIZE.area(); j++) {
			
			// If confidency of ORF is good enough
			if ((short)(savedConf[i].at<uchar>(imagePointsT[i][j].y, imagePointsT[i][j].x)) > THRESH_ORF_CONF) {
				
				// Triangulate and add stereo points
				newPointOM = OMtriangle.triangulate((double)imagePointsL[i][j].x, (double)imagePointsR[i][j].x, (double)imagePointsL[i][j].y);
				newPointOM = Point3d(newPointOM.x/1000, newPointOM.y/1000, newPointOM.z/1000); // convert in meters
				pointcloudOM.push_back(newPointOM);
				
				// Compute ORF coordinates (in meters) with depth map and inverting the pinhole equation
				double r = ((savedDepth[i].at<unsigned short>(imagePointsT[i][j].y, imagePointsT[i][j].x)>>2) & 0x3FFF)  * 0.00061;
				newPointORF = ORFtriangle.triangulate(imagePointsT[i][j].x, imagePointsT[i][j].y, r);
				pointcloudORF.push_back(newPointORF);
#ifdef	CALIB_DEBUG
				//Print results in case of debug
				cout.precision(3);
				DEBUG<<"COORD: ORF: ["<<newPointORF.x<<";"<<newPointORF.y<<";"<<newPointORF.z<<"]\tStereo: ["<<newPointOM.x<<";"<<newPointOM.y<<";"<<newPointOM.z<<"]\tINTER: ORF: "<<sqrt(pow((newPointORF.x - prevPointORF.x), 2) + pow((newPointORF.y - prevPointORF.y), 2) + pow((newPointORF.y - prevPointORF.y), 2))<<"m\tStereo: "<<sqrt(pow((newPointOM.x - prevPointOM.x), 2) + pow((newPointOM.y - prevPointOM.y), 2) + pow((newPointOM.y - prevPointOM.y), 2))<<"m"<<endl;
				prevPointOM = newPointOM;
				prevPointORF = newPointORF;
				rgbcloudOM.push_back(col[2*(i%(col.size()/2))]);
				rgbcloudORF.push_back(col[2*(i%(col.size()/2))+1]);
#endif
			}
		}
	}
	
#ifdef	CALIB_DEBUG	
	// Display 3D results
	// Show camera invert XY axes but we should invert T (because it corresponds to translation from right to left) -> nothing to do
	visualizerShowCamera(rotMatrixL, Vec3f(0,0,0), 255,0,0,0.02,"Left or ORF camera"); 
	visualizerShowCamera(rotMatrixR, Vec3f(TRL.at<double>(0)/1000, TRL.at<double>(1)/1000, TRL.at<double>(2)/1000), 0,0,255,0.02,"Right camera");
	pointcloudTot.insert(pointcloudTot.end(), pointcloudOM.begin(), pointcloudOM.end());
	pointcloudTot.insert(pointcloudTot.end(), pointcloudORF.begin(), pointcloudORF.end());
	rgbcloudTot.insert(rgbcloudTot.end(), rgbcloudOM.begin(), rgbcloudOM.end());
	rgbcloudTot.insert(rgbcloudTot.end(), rgbcloudORF.begin(), rgbcloudORF.end());
	RunVisualization(pointcloudTot, rgbcloudTot);
#endif
	
	return 0;
}


int Calibration::computeHaloMinimization(vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, Mat& MLT)
{
// 	// Test RANSAC accuracy
// 	pointcloudOM.push_back(Point3d(0.0,0.0,0.1));
// 	pointcloudORF.push_back(Point3d(10.0, 0.4, 0.1));
	
	// Transfer key points from pointcloud to vector<Eigen::Vector3d>>
	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pointT, pointS, pointT2, pointS2;
	for (int i=0; i<pointcloudORF.size(); i++) {
		pointT.push_back(Vector3d(pointcloudORF[i].x, pointcloudORF[i].y, pointcloudORF[i].z));
		pointS.push_back(Vector3d(pointcloudOM[i].x, pointcloudOM[i].y, pointcloudOM[i].z));
	}
	
	// Compute absolute orientation
	AbsoluteOrientation abs;
#ifdef	CALIB_DEBUG
	pointT2 = pointT;
	pointS2 = pointS;
#endif
	Matrix4d M1 = abs.orientationRANSAC(pointT, pointS);
#ifdef	CALIB_DEBUG
	DEBUG<<"M_TL with RANSAC:\n"<<M1<<endl;
	M1 = abs.absoluteOrientation(pointT2, pointS2);
	DEBUG<<"M_TL without RANSAC:\n"<<M1<<endl;
#endif
	MLT = Mat(M1.rows(), M1.cols(), CV_64F, M1.data()).t();
	
	return 0;
}


int Calibration::fillHaloMatrices(vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, vector<cv::Vec3b>& rgbcloudOM, vector<cv::Vec3b>& rgbcloudORF, Mat& RRL, Mat& TRL, Mat& MLT, string filename)
{
	Mat MTL = Mat::zeros(4,4,CV_64F);
	Mat MLR = Mat::zeros(4,4,CV_64F);
	Mat MRL = Mat::zeros(4,4,CV_64F);
	Mat MRT = Mat::zeros(4,4,CV_64F);
	Mat MTR = Mat::zeros(4,4,CV_64F);
	MTL = MLT.inv();
	Matrix4d MRL_eig;
	MRL_eig(0,0) = RRL.at<double>(0,0);
	MRL_eig(1,0) = RRL.at<double>(0,1);
	MRL_eig(2,0) = RRL.at<double>(0,2);
	MRL_eig(3,0) = TRL.at<double>(0)/1000;
	MRL_eig(0,1) = RRL.at<double>(1,0);
	MRL_eig(1,1) = RRL.at<double>(1,1);
	MRL_eig(2,1) = RRL.at<double>(1,2); 
	MRL_eig(3,1) = TRL.at<double>(1)/1000;
	MRL_eig(0,2) = RRL.at<double>(2,0);
	MRL_eig(2,1) = RRL.at<double>(2,1);
	MRL_eig(2,2) = RRL.at<double>(2,2);
	MRL_eig(3,2) = TRL.at<double>(2)/1000;
	MRL_eig(0,3) = 0.0;
	MRL_eig(1,3) = 0.0;
	MRL_eig(2,3) = 0.0;
	MRL_eig(3,3) = 1.0;
	MRL = Mat(MRL_eig.rows(), MRL_eig.cols(), CV_64F, MRL_eig.data());
	MLR = MRL.inv();
	MRT = MRL * MLT;
	MTR = MRT.inv();
	
	// Save the calibration matrices
	FileStorage storage(filename, FileStorage::WRITE);
	storage<<"MTL"<<MTL;
	storage<<"MTR"<<MTR;
	storage<<"MLT"<<MLT;
	storage<<"MLR"<<MLR;
	storage<<"MRT"<<MRT;
	storage<<"MRL"<<MRL;
	storage.release();
	
#ifdef CALIB_DEBUG
	DEBUG<<"\nMTL: "<<MTL<<"\nMLR: "<<MLR<<"\nMRL: "<<MRL<<"\nMRT: "<<MRT<<"\nMTR: "<<MTR<<"\nMLT: "<<MLT<<endl;

	// Reconstruction and displayMatrix
	vector<Point3d> newPointcloudOM, pointcloudTot;
	vector<Vec3b> rgbcloudTot;
	Point3d newPointOM;
	for (int i=0; i< pointcloudORF.size(); i++) {
		Mat PL = Mat::zeros(4, 1, CV_64F);
		PL.at<double>(0,0) = pointcloudOM[i].x;
		PL.at<double>(1,0) = pointcloudOM[i].y;
		PL.at<double>(2,0) = pointcloudOM[i].z;
		PL.at<double>(3,0) = 1;
 		Mat PT = MTL * PL;
		DEBUG<<"Point "<<i+1<<":\tORF: ["<<setprecision(3)<<pointcloudORF[i].x<<","<<setprecision(3)<<pointcloudORF[i].y<<","<<setprecision(3)<<pointcloudORF[i].z<<"]  \tL: ["<<setprecision(3)<<PL.at<double>(0,0)<<","<<setprecision(3)<<PL.at<double>(1,0)<<","<<setprecision(3)<<PL.at<double>(2,0)<<"]    \tT: ["<<setprecision(3)<<PT.at<double>(0,0)<<","<<setprecision(3)<<PT.at<double>(1,0)<<","<<setprecision(3)<<PT.at<double>(2,0)<<"]"<<endl;
		newPointOM = Point3d(PT.at<double>(0,0), PT.at<double>(1,0), PT.at<double>(2,0));
		newPointcloudOM.push_back(newPointOM);
	}

	Mat dummy = Mat::zeros(3, 3, CV_64F);
	dummy.at<double>(0,0) = 1.0;
	dummy.at<double>(1,1) = 1.0;
	dummy.at<double>(2,2) = 1.0;
	visualizerShowCamera(dummy, Vec3f(0,0,0),0,255,0,0.02,"ORF camera");
	dummy.at<double>(0,0) = MLT.at<double>(0,0);
	dummy.at<double>(0,1) = MLT.at<double>(0,1);
	dummy.at<double>(0,2) = MLT.at<double>(0,2);
	dummy.at<double>(1,0) = MLT.at<double>(1,0);
	dummy.at<double>(1,1) = MLT.at<double>(1,1);
	dummy.at<double>(1,2) = MLT.at<double>(1,2);
	dummy.at<double>(2,0) = MLT.at<double>(2,0);
	dummy.at<double>(2,1) = MLT.at<double>(2,1);
	dummy.at<double>(2,2) = MLT.at<double>(2,2);
	visualizerShowCamera(dummy, Vec3f(MLT.at<double>(0,3), MLT.at<double>(1,3), MLT.at<double>(2,3)),255,0,0,0.02,"Left camera");
	dummy.at<double>(0,0) = MRT.at<double>(0,0);
	dummy.at<double>(0,1) = MRT.at<double>(0,1);
	dummy.at<double>(0,2) = MRT.at<double>(0,2);
	dummy.at<double>(1,0) = MRT.at<double>(1,0);
	dummy.at<double>(1,1) = MRT.at<double>(1,1);
	dummy.at<double>(1,2) = MRT.at<double>(1,2);
	dummy.at<double>(2,0) = MRT.at<double>(2,0);
	dummy.at<double>(2,1) = MRT.at<double>(2,1);
	dummy.at<double>(2,2) = MRT.at<double>(2,2);
	visualizerShowCamera(dummy, Vec3f(MRT.at<double>(0,3), MRT.at<double>(1,3), MRT.at<double>(2,3)),0,0,255,0.02,"Right camera");
 	pointcloudTot.insert(pointcloudTot.end(), newPointcloudOM.begin(), newPointcloudOM.end());
 	pointcloudTot.insert(pointcloudTot.end(), pointcloudORF.begin(), pointcloudORF.end());
	rgbcloudTot.insert(rgbcloudTot.end(), rgbcloudOM.begin(), rgbcloudOM.end());
	rgbcloudTot.insert(rgbcloudTot.end(), rgbcloudORF.begin(), rgbcloudORF.end());
	RunVisualization(pointcloudTot, rgbcloudTot);
#endif
	
	return 0;
}


int Calibration::haloCalib(string filename)
{
	/* COMMENTS: 
	 * - Triangulation should be more accurate, using Hartley and Sturm Triangulation linear methods
	 * - The ORF depth map hsould be calibrated more accurately and one should understand where the coefficient 1.15 comes from
	 * - One should verify if the ORF depth map gives Z or radial depth and ameliorate the coordinates computation.
	 */	
	
	vector<vector<Point2f> > imagePointsL, imagePointsR, imagePointsT;
	vector<Mat> savedDepth, savedConf;
	vector<Point3d> pointcloudOM, pointcloudORF;
	vector<cv::Vec3b> rgbcloudOM, rgbcloudORF;
	Mat RRL, TRL, MLT;
	
	// -- STEP 1: Detect checkerboard in iL, iR and vT --
	this->acqHaloCheckPoints(imagePointsL, imagePointsR, imagePointsT, savedConf, savedDepth);
	
	// -- STEP 2: For each point, compute 3D coordinates from stereo cameras and ORF cameras --
	this->computeHalo3DPoints(imagePointsL, imagePointsR, imagePointsT, savedConf, savedDepth, pointcloudOM, pointcloudORF, rgbcloudOM, rgbcloudORF, RRL, TRL);
	
	// -- STEP 3: Minimize the distances between them to find the MLT rotation matrix --
	this->computeHaloMinimization(pointcloudOM, pointcloudORF, MLT);
	
	// -- STEP 4: Fill each rototranslation matrices --
	this->fillHaloMatrices(pointcloudOM, pointcloudORF, rgbcloudOM, rgbcloudORF, RRL, TRL, MLT, filename);
	
	return 0;
}


// ----------- Virtual capture functions ---------------

int Calibration::captureOrf(Mat& depthNewImageFrame, Mat& visualNewImageFrame, Mat& confidenceNewImageFrame, TimeStamp& ts, int num)
{
	// The capture function for ORF is defined in orf.cpp
}


unsigned int Calibration::captureTwoImages(cv::Mat& leftNewImageFrame, cv::Mat& rightNewImageFrame, int* img_num1, int* img_num2, TimeStamp& ts, int& synchCheckFlag, int num)
{
	// The capture function for Cameras is defined in camera.cpp
}


int Calibration::captureAllRectifiedImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag)
{
	// The capture function for Halo is defined in halo.cpp
}

int Calibration::captureAllImages(Mat& iL, Mat& iR, Mat& dT, Mat& vT, Mat& cT, int flag)
{
	// The capture function for Halo is defined in halo.cpp
}