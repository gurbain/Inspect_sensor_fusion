/*! 
* 	\file    haloTriangulator.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    S 2014
* 	\version 0.1
* 	\brief   Triangulation and fusion from stereo and orf images
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "haloTriangulator.h"

using namespace cv;
using namespace std;


HaloTriangulator::HaloTriangulator(string filenameORF, string filenameOM, string filenameHALO)
{
	if (MLT.empty()) {
		FileStorage storage;
		int retVal = storage.open(filenameHALO, FileStorage::READ);
		if (retVal == 1) {
			INFO<<"HALO Calibration file found! No need to perform calibration!"<<endl;
		} else {
			INFO<<"HALO Calibration file not found! Calibration needed!"<<endl;
			exit(1);
		}
		storage["MLT"]>>MLT;
		storage["MTL"]>>MTL;
		storage["MRT"]>>MRT;
		storage.release();
	}
	
	if (intrinsicL.empty()) {
		FileStorage storage;
		int retVal = storage.open(filenameOM, FileStorage::READ);
		if (retVal != 1) {
			INFO<<"STEREO Calibration file not found! Calibration needed!"<<endl;
			exit(1);
		}
		storage["intrinsicL"]>>intrinsicL;
		storage["intrinsicR"]>>intrinsicR;
		storage["projMatrixR"]>>projMatrixStereo;
		storage.release();
	}
	
	if (intrinsicT.empty()) {
		FileStorage storage;
		int retVal = storage.open(filenameORF, FileStorage::READ);
		if (retVal != 1) {
			INFO<<"HALO Calibration file not found! Calibration needed!"<<endl;
			exit(1);
		}
		storage["intrinsicT"]>>intrinsicT;
		storage.release();
	}
	
	this->computeInterParams();
	//stereo = new StereoTriangulator(intrinsicL, intrinsicR);
	orf = new OrfTriangulator(intrinsicT);
}


HaloTriangulator::HaloTriangulator(Mat& projMatrixR_, Mat& intrinsicT_, Mat& intrinsicL_, Mat& intrinsicR_, string filenameHALO)
{
	if (MLT.empty()) {
		FileStorage storage;
		int retVal = storage.open(filenameHALO, FileStorage::READ);
		if (retVal == 1) {
			INFO<<"HALO Calibration file found! No need to perform calibration!"<<endl;
		} else {
			INFO<<"HALO Calibration file not found! Calibration needed!"<<endl;
			exit(1);
		}
		storage["MLT"]>>MLT;
		storage["MRT"]>>MRT;
		storage["MTL"]>>MTL;
		storage.release();
	}
	
	intrinsicT = intrinsicT_;
	intrinsicL = intrinsicL_;
	intrinsicR = intrinsicR_;
	projMatrixStereo = projMatrixR_;
	
	this->computeInterParams();
	//stereo = new StereoTriangulator(intrinsicL, intrinsicR);
	stereo2 = new StereoTriangulator(projMatrixStereo);
	orf = new OrfTriangulator(intrinsicT);
}


HaloTriangulator::HaloTriangulator(Mat& intrinsicT_, Mat& intrinsicL_, Mat& intrinsicR_, Mat& MLT_, Mat& MTL_, Mat& MRT_)
{
	intrinsicT = intrinsicT_;
	intrinsicL = intrinsicL_;
	intrinsicR = intrinsicR_;
	MLT = MLT_;
	MTL = MTL_;
	MRT = MRT_;
	
	this->computeInterParams();
	//stereo = new StereoTriangulator(intrinsicL, intrinsicR);
	orf = new OrfTriangulator(intrinsicT);
}


vector<Point3d> HaloTriangulator::fusion(Mat& dT, Mat& cT, Mat& iL, Mat& iR, vector<Vec3b>& rgbcloud, int flag)
{	
	#define ORF_CLOUD_DOWNSAMPLING	40
	// TODO Correct downsampling
	resize(dT, dT, Size((int)dT.cols/ORF_CLOUD_DOWNSAMPLING, (int)dT.rows/ORF_CLOUD_DOWNSAMPLING));
	resize(cT, cT, Size((int)cT.cols/ORF_CLOUD_DOWNSAMPLING, (int)cT.rows/ORF_CLOUD_DOWNSAMPLING));
	
#ifdef FUSION_DEBUG
	// DEBUG parameters
	Mat dF = Mat::zeros(dT.size(), dT.type());
	Mat meanILm = Mat::zeros(dT.size(), CV_8U);
	Mat meanIRm = Mat::zeros(dT.size(), CV_8U);
	Mat sigma_w_m = Mat::zeros(cT.size(), cT.type());
	Mat sigma_t_m = Mat::zeros(cT.size(), cT.type());
	Mat sigma_s_m = Mat::zeros(cT.size(), cT.type());
	Mat cim = Mat::zeros(dT.size(), CV_8U);
	int num = 0;
	Mat iR2 = iR.clone();
	Mat iL2 = iL.clone();
	double sigma_s_max = 0;
	double sigma_t_max = 0;
	double sigma_w_max = 0;
	vector<KeyPoint> keyL, keyR;
	vector<double> dist;
	Mat newPointStereoL, newPointStereoT;
	Point3d newPointStereoL2;
#endif
	
	// Probability parameters
	double sigma_i= estimateNoiseVariance(iL, iR);
	double meanD, sigma_t, sigma_s, sigma_w;
	int meanIL, meanIR;
	vector<double> ci;
	double prob_stereo_sum;
	vector<double> prob_orf, prob_stereo_temp, prob_stereo, prob_joint;
	
	// Algorithm parameters
	vector<Point3d> pointcloud, pointcloudf;
	vector<Vec3b> rgbcloudf;
	double d, di, step, m;
	int sum, sum1, sum2;
	Point3d newPointT, newPointFus;
	Mat newPointLi, newPointTi, newPointRi;
	Point2d camCoordL, camCoordR;

	
	// For each points of the image
	for (int i=0; i<dT.rows; i++) {
		for (int j=0; j<dT.cols; j++) {
			
			// -- STEP 0 : Compute point coordinates --
			d = ((dT.at<unsigned short>(i, j)>>2) & 0x3FFF)*0.00061;
			newPointT = orf->triangulateOrf(j*ORF_CLOUD_DOWNSAMPLING, i*ORF_CLOUD_DOWNSAMPLING, d);
			
			// -- STEP 1: if the point is framed by the three cameras --
			if ( newPointT.z > (b *a1)/(a1 + a2) && (newPointT.z > 0.4 && newPointT.z < 3)) {
				if ( -(newPointT.z - b)/a2 < newPointT.x){
					if ( abs(newPointT.y) < newPointT.z/max(a3, a4)) {
						//if (newPointT.x>0.05 && newPointT.x<0.45 && newPointT.y>0.05 && newPointT.y<0.20) {
						
						// -- STEP 2: Calculate variances --
						
						// 2.1 Compute sigma_t as a function of confidency
						sigma_t = (((65535 - cT.at<unsigned short>(i,j))>>2) & 0x3FFF)*0.00061/10;
						
						// 2.2 Compute sigma_s as a function of neighborhood
						if ( i > VAR_KER && j > VAR_KER && i < (dT.rows - VAR_KER) && j < (dT.cols - VAR_KER) ) {
							// 2.2.1 Compute map of mean values with kernel
							sum = 0.0;
							for(int k=-VAR_KER; k<VAR_KER+1; k++) {
								for(int l=-VAR_KER; l<VAR_KER+1; l++) {
									sum = sum + dT.at<unsigned short>(i+k,j+l);
								}
							}
							meanD = sum/((2*VAR_KER+1)*(2*VAR_KER+1));
							
							// 2.2.2 Compute map of variance values with kernel
							sum = 0.0;
							for(int k=-VAR_KER; k<VAR_KER+1; k++) {
								for(int l=-VAR_KER; l<VAR_KER+1; l++) {
									sum = sum + pow((dT.at<unsigned short>(i+k,j+l) - meanD), 2);
									//cout<<j<<". Sum: "<<sum<<" and dist: "<<(dT.at<unsigned short>(i+k,j+l) - meanD)<<endl;
								}
							}
							sigma_s = (((unsigned short)sqrt(sum/((2*VAR_KER+1)*(2*VAR_KER+1)))>>2) & 0x3FFF)*0.00061/4;
							
						}
						
						// 2.3 Compute sigma_w as the max of sigma_s and sigma_t
						sigma_w = max(sigma_t, sigma_s) / 4;
#ifdef FUSION_DEBUG
						//DEBUG<<"Point ("<<i<<","<<j<<"):  \tSigma_t: "<<sigma_t<<"    \tSigma_s: "<<sigma_s<<"\t\tSigma_w: "<<sigma_w<<endl;
						sigma_s_max = max(sigma_s_max, sigma_s);
						sigma_t_max = max(sigma_t_max, sigma_t);
						sigma_w_max = max(sigma_w_max, sigma_w);
						sigma_t_m.at<unsigned short>(i, j) = ((unsigned short)(sigma_t/0.00061*2))<<2;
						sigma_s_m.at<unsigned short>(i, j) = ((unsigned short)(sigma_s/0.00061*2))<<2;
						sigma_w_m.at<unsigned short>(i, j) = max(sigma_t_m.at<unsigned short>(i, j), sigma_s_m.at<unsigned short>(i, j));
#endif
						// -- STEP 3: Create a discrete interval --
						// 3.1 Initialize
						prob_orf.clear();
						prob_stereo_temp.clear();
						prob_stereo.clear();
						prob_joint.clear();
						ci.clear();
						keyL.clear();
						keyR.clear();
						dist.clear();
						m = 0;
						
						// 3.2 Create the interval and begin a loop
						step= pow(d, 2) / (baseline * OM_FOCUS) * OM_PREC / K_INT;

						di = d - 3 * sigma_w;
						do {
							// -- STEP 4: Compute ORF probability --
							prob_orf.push_back(exp(- pow((di - d), 2)/(2*pow(sigma_w, 2))) / sqrt(2 * PI * pow(sigma_w, 2)));
							
							// -- STEP 5: Recover Stereo coordinates --
							
							// 5.1 Get values in T coordinate system
							newPointTi = orf->triangulateOrfMat(j*ORF_CLOUD_DOWNSAMPLING, i*ORF_CLOUD_DOWNSAMPLING, di);
							
							// 5.2 Move from T to L
							newPointLi = MLT * newPointTi;
							
							// 5.3 Project into uv coordinates
							stereo2->reprojectStereo(camCoordL, camCoordR, newPointLi.at<double>(0,0)*1000, newPointLi.at<double>(0,1)*1000, newPointLi.at<double>(0,2)*1000);
#ifdef FUSION_DEBUG
							// 5.4 Reproject all stereo points in L coordinates
							newPointStereoL = stereo2->triangulateStereoMat(camCoordL.x, camCoordR.x, camCoordL.y);
							
							// 5.5 Move from L to T
							newPointStereoL.at<double>(0,0) = newPointStereoL.at<double>(0,0) / 1000;
							newPointStereoL.at<double>(0,1) = newPointStereoL.at<double>(0,1) / 1000;
							newPointStereoL.at<double>(0,2) = newPointStereoL.at<double>(0,2) / 1000;
							newPointStereoT = MTL * newPointStereoL;
							
							// 5.6 Add in point cloud for comparaison
						//	pointcloud.push_back(Point3d(newPointStereoT.at<double>(0,0), newPointStereoT.at<double>(0,1), newPointStereoT.at<double>(0,2)));
							//rgbcloud.push_back(Vec3b(0, 255, 0));
							//cout<<"(u,v) in L: ("<<camCoordL.x<<", "<<camCoordL.y<<")\t(u,v) in R: ("<<camCoordR.x<<", "<<camCoordR.y<<")"<<endl;
							//cout<<newPointStereoT<<" and "<<newPointTi<<" and "<<norm(newPointStereoT-newPointTi)<<endl;
#endif
							// -- STEP 6: Compute stereo probability --
							//if ( camCoordL.x > TAD_KER && camCoordL.y > TAD_KER && camCoordL.x < (iL.rows - TAD_KER) && camCoordL.y < (iL.cols - TAD_KER) ) {
								
								// 6.1 Compute sum of absolute differences
								double sum3 = 0.0;
								for(int k=-TAD_KER; k<TAD_KER+1; k++) {
									for(int l=-TAD_KER; l<TAD_KER+1; l++) {
										sum3 = sum3 + abs((double)iL.at<uchar>(camCoordL.y+k,camCoordL.x+l)/255.0 - (double)iR.at<uchar>(camCoordL.y+k,camCoordL.x+l)/255.0);
									}
								}
								
								// 6.2 Threshold comparaison 
								ci.push_back(min((int)sum3, 25));///(2*TAD_KER), 0.5));						
							//}
							
							// 6.3 Compute stereo probability
							prob_stereo.push_back(exp(- ci[m] / sigma_i));;
							
							// -- STEP 7: Compute joint probability --
							prob_joint.push_back(prob_stereo[m]*prob_orf[m]);
							cout<<"Prob ORF: "<<prob_orf[m]<<"   \tAnd Prob Stereo: "<<prob_stereo[m]<<"\tAnd Prob Joint: "<<prob_joint[m]<<endl;
							
							m++;
							di = di + step;
						} while (di < d + 3 * sigma_w);
						
						// -- STEP 8: Select depth which maximize probability --
						vector<double>::iterator prob_max = max_element(prob_joint.begin(), prob_joint.end());
						int prob_index = distance(prob_joint.begin(), prob_max);
						double prob_depth = d - 3 * sigma_w + prob_index * step;
#ifdef FUSION_DEBUG
// 						DEBUG<<"INTERVAL: "<<6*sigma_w<<"\tSTEP: "<<step<<endl;
						vector<double>::iterator prob_orf_max = max_element(prob_orf.begin(), prob_orf.end());
						int prob_orf_index = distance(prob_orf.begin(), prob_orf_max);
						vector<double>::iterator prob_stereo_max = max_element(prob_stereo.begin(), prob_stereo.end());
						int prob_stereo_index = distance(prob_stereo.begin(), prob_stereo_max);
						DEBUG<<"Point ("<<i<<","<<j<<"): \tMax joint index: "<<prob_index<<" \tMax orf index: "<<prob_orf_index<<"\tMax stereo index: "<<prob_stereo_index<<endl;
 						//cout<<"Point ("<<i<<","<<j<<"):\tMax joint prob: "<<*prob_max<<"    \tPosition: "<<prob_index<<"\tCorresponding depth: "<<prob_depth<<endl;
#endif
						// -- STEP 9: Transform to orthonormal coordinate system --
						newPointFus = orf->triangulateOrf(j*ORF_CLOUD_DOWNSAMPLING, i*ORF_CLOUD_DOWNSAMPLING, prob_depth);
						dF.at<unsigned short>(i,j) = ((unsigned short)(prob_depth / 0.00061))<<2;
						
						pointcloud.push_back(newPointT);
						rgbcloud.push_back(Vec3b(255,0,0));
						pointcloud.push_back(newPointFus);
						rgbcloud.push_back(Vec3b(0,0,255));
					
					} 
// 					else {
// 						pointcloud.push_back(newPointT);
// 						rgbcloud.push_back(Vec3b(0,0,255));
// 					}
				} 
// 				else {
// 					pointcloud.push_back(newPointT);
// 					rgbcloud.push_back(Vec3b(0,0,255));
// 				}
			} 
// 			else {
// 				pointcloud.push_back(newPointT);
// 				rgbcloud.push_back(Vec3b(0,0,255));
// 			}
		}
	}
// 	DEBUG<<"Sigmaw max: "<<sigma_w_max<<"\tSigma_t max: "<<sigma_t_max<<"\tSigma_s max: "<<sigma_s_max<<endl;
// 	resize(sigma_s_m, sigma_s_m, Size((int)sigma_s_m.cols*ORF_CLOUD_DOWNSAMPLING, (int)sigma_s_m.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("sigma_s_m", sigma_s_m);
// 	cvWaitKey(0);

	
	
	// Display point cloud
	Mat R = Mat::zeros(3, 3, CV_64F);
	R.at<double>(0,0) = 1;
	R.at<double>(1,1) = 1;
	R.at<double>(2,2) = 1;
// 	visualizerShowCamera(R, Vec3f(0,0,0), 255,0,0,0.02,"ORF camera"); 
// 	visualizerShowCamera(R, Vec3f(-0.25, 0, 0), 0,255,0,0.02,"Stereo camera");
// 	RunVisualization(pointcloudf, rgbcloudf);
	visualizerShowCamera(R, Vec3f(0,0,0), 255,0,0,0.02,"ORF camera"); 
	visualizerShowCamera(R, Vec3f(-0.25, 0, 0), 0,255,0,0.02,"Stereo camera");
	RunVisualization(pointcloud, rgbcloud);
	return pointcloud;
}


Point2d HaloTriangulator::reprojectT(double x, double y, double z)
{
	Point2d p;
	
	return p;
}


Point2d HaloTriangulator::reprojectL(double x, double y, double z)
{
	Point2d p;
	
	return p;
}


Point2d HaloTriangulator::reprojectR(double x, double y, double z)
{
	Point2d p;
	
	return p;
}


void HaloTriangulator::computeInterParams()
{
	baseline = 0.08;//-projMatrixStereo.at<double>(0,3) / 1000;
	
	a1 = tan(PI/2 - PI*ORF_FOV_H/360);
	a2 = tan(PI/2 - PI*CAM_FOV_H/360);
	a3 = tan(PI/2 - PI*ORF_FOV_V/360);
	a4 = tan(PI/2 - PI*CAM_FOV_V/360);
	b = -MRT.at<double>(0,3) * a2;
}


double HaloTriangulator::estimateNoiseVariance(Mat& iL, Mat& iR) 
{
	int widthL = iL.cols;
	int heightL = iL.rows;
	int widthR = iR.cols;
	int heightR = iR.rows;
	
	Mat iLdst, iRdst;
	
	double sigmaL, sigmaR;
	
	Point anchor = Point( -1, -1 );
	double delta = 0;
	int ddepth = -1;
	
	Mat kernel = Mat::zeros(3, 3, CV_8S);
	kernel.at<char>(0,0) = 1;
	kernel.at<char>(0,1) = -2;
	kernel.at<char>(0,2) = 1;
	kernel.at<char>(1,0) = -2;
	kernel.at<char>(1,1) = 4;
	kernel.at<char>(1,2) = -2;
	kernel.at<char>(2,0) = 1;
	kernel.at<char>(2,1) = -2;
	kernel.at<char>(2,2) = 1;

	filter2D(iL, iLdst, ddepth , kernel, anchor, delta, BORDER_DEFAULT);
	sigmaL = sum(abs(iLdst))[0];
	sigmaL = sigmaL * sqrt(0.5 * PI) / (6 * (widthL-2) * (heightL-2));
	
	filter2D(iR, iRdst, ddepth , kernel, anchor, delta, BORDER_DEFAULT);
	sigmaR = sum(abs(iRdst))[0];
	sigmaR = sigmaR * sqrt(0.5 * PI) / (6 * (widthR-2) * (heightR-2));
	
	return max(sigmaL, sigmaR);	
}

// #ifdef FUSION_DEBUG
	
								// Add keypoints
// 								keyL.push_back(KeyPoint(camCoordL.x, camCoordL.y, 15));
// 								keyR.push_back(KeyPoint(camCoordR.x, camCoordR.y, 15));
// 						// computing descriptors
// 						SurfDescriptorExtractor extractor;
// 						Mat descriptorsL, descriptorsR;
// 						extractor.compute(iL, keyL, descriptorsL);
// 						extractor.compute(iR, keyR, descriptorsR);
// 						
// 						// matching descriptors
// 						BruteForceMatcher<L2<float> > matcher;
// 						vector<DMatch> matches;
// 						cout<<matches[1]<<endl;
// 						matcher.match(descriptorsL, descriptorsR, matches);
// 						cout<<matches.size()<<endl;
// 						
// 
// 						// Display
// 						DEBUG<<"DISTANCES: [";
// 						for( int v = 0; v < descriptorsL.rows; v++ ) { 
// 							dist.push_back((double)matches[v].distance);
// 							cout<<dist[v]<<" ";
// 						}
// 						cout<<"]"<<endl;
// 						DEBUG<<"PROB: [";
// 						for( int v = 0; v < m+1; v++ ) { 
// 							cout<<prob_stereo_temp[m]<<" ";
// 						}
// 						cout<<"]"<<endl;
						
// 						Mat img_matches;
// 						drawMatches(iL2, keyL, iR2, keyR, matches, img_matches);
// 						imshow("matches", img_matches);
// 						waitKey(0);
// #endif

								
// 								if (newPointT.x>0.05 && newPointT.x<0.45 && newPointT.y>0.05 && newPointT.y<0.20) {
// 								circle(iL2, Point(camCoordL.x, camCoordL.y), 10, (255,255,255));
// 								circle(iR2, Point(camCoordR.x, camCoordR.y), 10, (255,255,255));
// 								imshow("aa", iL2);
// 								imshow("bb", iR2);
// 								cvWaitKey(0);
// 								}

	//
	
// 	resize(iR2, iR2, Size((int)iR2.cols*2, (int)iR2.rows*2));
// 	imshow("aa", iR2);
// 	cvWaitKey(0);
// 	resize(sigma_t_m, sigma_t_m, Size((int)sigma_t_m.cols*ORF_CLOUD_DOWNSAMPLING, (int)sigma_t_m.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("sigma_t_m", sigma_t_m);
// 	resize(sigma_w_m, sigma_w_m, Size((int)sigma_w_m.cols*ORF_CLOUD_DOWNSAMPLING, (int)sigma_w_m.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("sigma_w_m", sigma_w_m);
// 	
// 	resize(cT, cT, Size((int)cT.cols*ORF_CLOUD_DOWNSAMPLING, (int)cT.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("Confidency", cT);
// 	resize(cim, cim, Size((int)cim.cols*ORF_CLOUD_DOWNSAMPLING, (int)cim.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("cim ", cim);
// 	resize(meanILm, meanILm, Size((int)meanILm.cols*ORF_CLOUD_DOWNSAMPLING, (int)meanILm.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("IL mean ", meanILm);
// 	resize(meanIRm, meanIRm, Size((int)meanIRm.cols*ORF_CLOUD_DOWNSAMPLING, (int)meanIRm.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("IR mean ", meanIRm);
// 	resize(dT, dT, Size((int)dT.cols*ORF_CLOUD_DOWNSAMPLING, (int)dT.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("Prev depth ", dT);
// 	resize(dF, dF, Size((int)dF.cols*ORF_CLOUD_DOWNSAMPLING, (int)dF.rows*ORF_CLOUD_DOWNSAMPLING));
// 	imshow("New depth ", dF);
// 	cvWaitKey(0);

// 
// 						DEBUG<<"Prob ORF: [";
// 						copy(prob_orf.begin(), prob_orf.end(),ostream_iterator<double>(cout, "; "));
// 						cout<<"]\nAnd Prob Stereo: [";
// 						copy(prob_stereo.begin(), prob_stereo.end(),ostream_iterator<double>(cout, "; "));
// 						cout<<"]\nAnd Prob Joint: [";
// 						copy(prob_joint.begin(), prob_joint.end(),ostream_iterator<double>(cout, "; "));
// 						cout<<"]\nAnd CI: [";
// 						copy(ci.begin(), ci.end(),ostream_iterator<double>(cout, "; "));
// 						cout<<"]"<<endl;
