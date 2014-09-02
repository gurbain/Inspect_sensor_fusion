/*! 
* 	\file    calibration.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visitor student at MIT SSL
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
	vector<vector<Point2f> > imagePoints(ORF_NUMBER_BOARDS);
	vector<vector<Point3f> > objectPoints(ORF_NUMBER_BOARDS);
	vector<Mat> rotationVectors;
	vector<Mat> translationVectors;
	Mat intrinsicMatrix, distorsionCoeffs;
	
	// Usefull variables
	int successes = 0;
	int num = 0;
	int step, frame = 0;
	bool found;
	
	// Capture first image
	TimeStamp t;
	int retVal = captureOrf(dt, it, ct, t);
	if (retVal!=0)
		return -1;
	// Capture ORF_NUMBER_BOARDS images
	while(successes < ORF_NUMBER_BOARDS) {
		if((frame++ % ORF_ACQ_STEP)==0){
			// Find chessboard corners:
			found = findChessboardCorners(it, BOARD_SIZE, imagePoints[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

			// Draw it if applicable
			drawChessboardCorners(it, BOARD_SIZE, Mat(imagePoints[successes]), found);

			// Add point if we find them
			if(found){
				objectPoints[successes] = create3DChessboardCorners(BOARD_SIZE, SQUARE_SIZE);
				successes++;
				INFO<<"Checkerboard found : "<<successes<<endl; 
			}
		}
		
		// Show the result
		imshow("Calibration", it);
		
		// Handle a timeout
		if (frame > 3000) {
			DEBUG<<"Timeout! Checkerboard not found! Please, restart calibration process"<<endl;
		}

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
		num++;
		int retVal = captureOrf(dt, it, ct, t, num);
		if (retVal!=0)
			return -1;
	}
	
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
	vector<vector<Point2f> > imagePointsL(STEREO_NUMBER_BOARDS), imagePointsR(STEREO_NUMBER_BOARDS);
	vector<vector<Point3f> > objectPoints(STEREO_NUMBER_BOARDS);
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
	int step, frame = 0;
	bool foundR = false, foundL = false;
	
	// Capture images
	TimeStamp t;
	retVal = captureTwoImages(iL, iR, dummy1, dummy2, t, dummy3, 0);
	
	// Capture STEREO_NUMBER_BOARDS images
	while(successes < STEREO_NUMBER_BOARDS) {
		if((frame++ % STEREO_ACQ_STEP)==0) {
			foundL = findChessboardCorners(iL, BOARD_SIZE, imagePointsL[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			if (foundL) {
				foundR = findChessboardCorners(iR, BOARD_SIZE, imagePointsR[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			}
			//INFO<<"Checkerboard found state : ["<<foundL<<", "<<foundR<<", "<<foundT<<" ]"<<endl; 

			// If sth found
			if(foundL && foundR){
				// Draw it if applicable
				drawChessboardCorners(iL, BOARD_SIZE, Mat(imagePointsL[successes]), foundL);
				drawChessboardCorners(iR, BOARD_SIZE, Mat(imagePointsR[successes]), foundR);
				
				// Add point
				objectPoints[successes] = create3DChessboardCorners(BOARD_SIZE, SQUARE_SIZE);
				successes++;
				INFO<<"Checkerboard found : "<<successes<<endl; 
			}
		}
		// Show the result
		imshow("Calib left", iL);
		imshow("Calib right", iR);

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
	vector<vector<Point3f> > objectPoints(HALO_NUMBER_BOARDS);
	
	// Usefull variables
	int retVal;
	int successes = 0;
	int step, frame = 0;
	bool foundT = false, foundR = false, foundL = false;
	
	
	// Capture HALO_NUMBER_BOARDS images
	while(successes < HALO_NUMBER_BOARDS) {
		
		// Capture images
		retVal = captureAllRectifiedImages(iL, iR, dT, vT, cT);
	
		if((frame++ % HALO_ACQ_STEP)==0){
			// Find chessboard corners:
			foundT = findChessboardCorners(vT, BOARD_SIZE, imagePointsT[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
			if (foundT) {
				foundL = findChessboardCorners(iL, BOARD_SIZE, imagePointsL[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
				if (foundL) {
					foundR = findChessboardCorners(iR, BOARD_SIZE, imagePointsR[successes], CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
				}
			}

			// If sth found
			if(foundL && foundR && foundT){	
				// Draw it if applicable
				drawChessboardCorners(iL, BOARD_SIZE, Mat(imagePointsL[successes]), foundL);
				drawChessboardCorners(iR, BOARD_SIZE, Mat(imagePointsR[successes]), foundR);
				drawChessboardCorners(vT, BOARD_SIZE, Mat(imagePointsT[successes]), foundT);
				
				// Save depth and conf map of the ORF
				savedDepth.push_back(dT.clone());
				cvtColor(cT, cT, CV_BGR2GRAY);
				savedConf.push_back(cT.clone());
				
				// Add point
				objectPoints[successes] = create3DChessboardCorners(BOARD_SIZE, SQUARE_SIZE);
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
	}
	
	return 0;
}

int Calibration::computeHalo3DPoints(vector<vector<Point2f> >& imagePointsL, vector<vector<Point2f> >& imagePointsR, vector<vector<Point2f> >& imagePointsT, vector<Mat>& savedConf, vector<Mat>& savedDepth, vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, Mat& RLR, Mat& TLR)
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
	storage["T"]>>TLR;
	storage["R"]>>RLR;
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
	SimpleTriangulator OMtriangle(intrinsicMatrixL, projMatrixR);
	SimpleTriangulator ORFtriangle(intrinsicMatrixT);
	Point3d newPointORF, newPointOM;
#ifdef	CALIB_DEBUG
	Point3d prevPointOM, prevPointORF;
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
				double z = ((savedDepth[i].at<unsigned short>(imagePointsT[i][j].y, imagePointsT[i][j].x)>>2) & 0x3FFF)  * 0.00061 / 1.15;
				newPointORF = ORFtriangle.invTriangulate(imagePointsT[i][j].x, imagePointsT[i][j].y, z);
				pointcloudORF.push_back(newPointORF);
#ifdef	CALIB_DEBUG
				//Print results in case of debug
	 			DEBUG<<"X1: "<<newPointOM.x<<"\tX2: "<<newPointORF.x<<"\tY1: "<<newPointOM.y<<"\tY2: "<<newPointORF.y<<"\tZ1: "<<newPointOM.z<<"\tZ2: "<<newPointORF.z<<endl;
				DEBUG<<"Interpoint ["<<i+1<<","<<j+1<<"]: Distance OM: "<<sqrt(pow((newPointOM.x - prevPointOM.x), 2) + pow((newPointOM.y - prevPointOM.y), 2) + pow((newPointOM.y - prevPointOM.y), 2))<<"m\tand distance ORF: "<<sqrt(pow((newPointORF.x - prevPointORF.x), 2) + pow((newPointORF.y - prevPointORF.y), 2) + pow((newPointORF.y - prevPointORF.y), 2))<<"m."<<endl;
				prevPointOM = newPointOM;
				prevPointORF = newPointORF;
#endif
			}
		}
	}
	
#ifdef	CALIB_DEBUG	
	// Display 3D results
	// Show camera invert XY axes but we should invert T (because it corresponds to translation from right to left) -> nothing to do
	visualizerShowCamera(rotMatrixL, Vec3f(0,0,0), 255,0,0,0.02,"Left or ORF camera"); 
	visualizerShowCamera(rotMatrixR, Vec3f(TLR.at<double>(0)/1000, TLR.at<double>(1)/1000, TLR.at<double>(2)/1000), 0,0,255,0.02,"Right camera");
	pointcloudTot.insert(pointcloudTot.end(), pointcloudOM.begin(), pointcloudOM.end());
	pointcloudTot.insert(pointcloudTot.end(), pointcloudORF.begin(), pointcloudORF.end());
	RunVisualization(pointcloudTot);
#endif
	
	return 0;
}

int Calibration::computeHaloMinimization(vector<Point3d>& pointcloudOM, vector<Point3d>& pointcloudORF, Mat& MTL)
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
	DEBUG<<"M_TL with RANSAC:\n"<<M<<endl;
	M1 = abs.absoluteOrientation(pointT2, pointS2);
	DEBUG<<"M_TL without RANSAC"<<M<<endl;
#endif
	MTL = Mat(M1.rows(), M1.cols(), CV_64F, M1.data()).t();
	
	return 0;
}

int Calibration::fillHaloMatrices(Mat& RLR, Mat& TLR, Mat& MTL)
{
	Mat MLT = Mat::zeros(4,4,CV_64F);
	Mat MLR = Mat::zeros(4,4,CV_64F);
	Mat MRL = Mat::zeros(4,4,CV_64F);
	Mat MRT = Mat::zeros(4,4,CV_64F);
	Mat MTR = Mat::zeros(4,4,CV_64F);
	MLT = MTL.inv();
	Matrix4d MLR_eig;
	MLR_eig(0,0) = RLR.at<double>(0,0);
	MLR_eig(1,0) = RLR.at<double>(0,1);
	MLR_eig(2,0) = RLR.at<double>(0,2);
	MLR_eig(3,0) = TLR.at<double>(0)/1000;
	MLR_eig(0,1) = RLR.at<double>(1,0);
	MLR_eig(1,1) = RLR.at<double>(1,1);
	MLR_eig(2,1) = RLR.at<double>(1,2); 
	MLR_eig(3,1) = TLR.at<double>(1)/1000;
	MLR_eig(0,2) = RLR.at<double>(2,0);
	MLR_eig(2,1) = RLR.at<double>(2,1);
	MLR_eig(2,2) = RLR.at<double>(2,2);
	MLR_eig(3,2) = TLR.at<double>(2)/1000;
	MLR_eig(0,3) = 0.0;
	MLR_eig(1,3) = 0.0;
	MLR_eig(2,3) = 0.0;
	MLR_eig(3,3) = 1.0;
	MLR = Mat(MLR_eig.rows(), MLR_eig.cols(), CV_64F, MLR_eig.data());
	MRL = MLR.inv();
	MRT = MRL * MLT;
	MTR = MRT.inv();
	
	DEBUG<<"MLT: "<<MLT<<"\nMLR: "<<MLR<<"\nMRL: "<<MRL<<"\nMRT: "<<MRT<<"\nMTR: "<<MTR<<"\nMTL: "<<MTL<<endl;

	// Reconstruction and displayMatrix
	
	return 0;
}

int Calibration::haloCalib(string filename)
{
	/* COMMENTS: 
	 * - Triangulation should be more accurate, using Hartley and Sturm Triangulation linear methods
	 * - The ORF depth map hsould be calibrated more accurately and one should understand where the coefficient 1.15 comes from
	 * - One should verify if the ORF depth map gives Z or radial depth and ameliorate the coordinates computation.
	 */	
	
	vector<vector<Point2f> > imagePointsL(HALO_NUMBER_BOARDS), imagePointsR(HALO_NUMBER_BOARDS), imagePointsT(HALO_NUMBER_BOARDS);
	vector<Mat> savedDepth, savedConf;
	vector<Point3d> pointcloudOM, pointcloudORF;
	Mat RLR, TLR, MTL;
	
	// -- STEP 1: Detect checkerboard in iL, iR and vT --
	this->acqHaloCheckPoints(imagePointsL, imagePointsR, imagePointsT, savedConf, savedDepth);
	
	// -- STEP 2: For each point, compute 3D coordinates from stereo cameras and ORF cameras --
	this->computeHalo3DPoints(imagePointsL, imagePointsR, imagePointsT, savedConf, savedDepth, pointcloudOM, pointcloudORF, RLR, TLR);
	
	// -- STEP 3: Minimize the distances between them to find the MTL rotation matrix --
	this->computeHaloMinimization(pointcloudOM, pointcloudORF, MTL);
	
	// -- STEP 4: Fill each rototranslation matrices --
	this->fillHaloMatrices(RLR, TLR, MTL);
	
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