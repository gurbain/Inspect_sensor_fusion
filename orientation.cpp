/*! 
* 	\file    orientation.cpp
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Absolute orientation solver. Adapted from Muggler sources in VERTIGO Project
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#include "orientation.h"

using namespace std;
using namespace Eigen;

AbsoluteOrientation::AbsoluteOrientation()
{
	cameraPoseEstimationRANSACMaxDistanceForInliers = 0.03;
	cameraPoseEstimationRANSACBreakRatio = 0.87;
	cameraPoseEstimationRANSACMinInliers = 10;
}

// According to:
// Berthold K. P. Horn: Closed-form solution of absolute orientation using unit quaternions (1986)
// Implementation adapted from Brent Tweddle/Konrad Makowka (2011)
Matrix4d AbsoluteOrientation::absoluteOrientation(vector<Vector3d, aligned_allocator<Vector3d> > & left, vector<Vector3d, aligned_allocator<Vector3d> > & right)
{
	// set right dimensions of output matrix
	Matrix4d pose = Matrix4d::Zero();

	// compute the mean of the left and right set of points
	Vector3d leftMean = Vector3d::Zero();
	Vector3d rightMean = Vector3d::Zero();

	for (int i = 0; i < (int)left.size(); i++) {
		leftMean += left[i];
		rightMean += right[i];
	}

	leftMean /= left.size();
	rightMean /= right.size();

	// Move all points to the "center"
	for (int i = 0; i < (int)left.size(); i++) {
		left[i] -= leftMean;
		right[i] -= rightMean;
	}
	//create M matrix
	Matrix3d M = Matrix3d::Zero();

	for (int i = 0; i < (int)left.size(); i++) {
		M(0, 0) += left[i](0) * right[i](0);
		M(0, 1) += left[i](0) * right[i](1);
		M(0, 2) += left[i](0) * right[i](2);
		M(1, 0) += left[i](1) * right[i](0);
		M(1, 1) += left[i](1) * right[i](1);
		M(1, 2) += left[i](1) * right[i](2);
		M(2, 0) += left[i](2) * right[i](0);
		M(2, 1) += left[i](2) * right[i](1);
		M(2, 2) += left[i](2) * right[i](2);
	}

	//create N matrix
	Matrix4d N = Matrix4d::Zero();
	N(0, 0) = M(0, 0) + M(1, 1) + M(2, 2);
	N(0, 1) = M(1, 2) - M(2, 1);
	N(0, 2) = M(2, 0) - M(0, 2);
	N(0, 3) = M(0, 1) - M(1, 0);

	N(1, 0) = M(1, 2) - M(2, 1);
	N(1, 1) = M(0, 0) - M(1, 1) - M(2, 2);
	N(1, 2) = M(0, 1) + M(1, 0);
	N(1, 3) = M(2, 0) + M(0, 2);

	N(2, 0) = M(2, 0) - M(0, 2);
	N(2, 1) = M(0, 1) + M(1, 0);
	N(2, 2) = -M(0, 0) + M(1, 1) - M(2, 2);
	N(2, 3) = M(1, 2) + M(2, 1);

	N(3, 0) = M(0, 1) - M(1, 0);
	N(3, 1) = M(2, 0) + M(0, 2);
	N(3, 2) = M(1, 2) + M(2, 1);
	N(3, 3) = -M(0, 0) - M(1, 1) + M(2, 2);


	// Compute eigenvalues and eigenvectors (normalized, but unordered):
	// "The eigenvectors are normalized to have (Euclidean) norm equal to one."
	// "The eigenvalues are not sorted in any particular order."
	// See: http://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
	EigenSolver<Matrix4d> es(N, true);

	int maxEigenValueCol;
	Vector4cd ev_complex = es.eigenvalues();
	Vector4d ev = ev_complex.real();

	if (ev(0) > ev(1) && ev(0) > ev(2) && ev(0) > ev(3)) maxEigenValueCol = 0;
	else if (ev(1) > ev(2) && ev(1) > ev(3)) maxEigenValueCol = 1;
	else if (ev(2) > ev(3)) maxEigenValueCol = 2;
	else maxEigenValueCol = 3;

	// Quaternion is maximum eigenvector
	Vector4cd q_complex = es.eigenvectors().col(maxEigenValueCol);
	Vector4d q = q_complex.real();

	// Compute Rotation matrix
	pose(0,0) = q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3);
	pose(0,1) = 2*(q(1)*q(2) - q(0)*q(3));
	pose(0,2) = 2*(q(1)*q(3) + q(0)*q(2));

	pose(1,0) = 2*(q(2)*q(1) + q(0)*q(3));
	pose(1,1) = q(0)*q(0) - q(1)*q(1) + q(2)*q(2) - q(3)*q(3);
	pose(1,2) = 2*(q(2)*q(3) - q(0)*q(1));

	pose(2,0) = 2*(q(3)*q(1) - q(0)*q(2));
	pose(2,1) = 2*(q(2)*q(3) + q(0)*q(1));
	pose(2,2) = q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3);

	// Calculate translation: TMat = rightmeanMat - scale*RMat*leftmeanMat (scale=1)
	pose.block<3,1>(0, 3) = rightMean - pose.block<3, 3>(0, 0) * leftMean;

	pose(3, 3) = 1;

	return pose;
}

Matrix4d AbsoluteOrientation::orientationRANSAC(vector<Vector3d, aligned_allocator<Vector3d> > & pointT, vector<Vector3d, aligned_allocator<Vector3d> > & pointS)
{
	Eigen::Matrix4d pose = Eigen::Matrix4d::Zero();
	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > selecT, selecS;
	
	vector<int> supportingMatchesId, maxSupportingMatchesId;
	
	int iterationNo = 0;

	maxSupportingMatchesId.clear();

	while (iterationNo < RANSAC_MAX_ITER) {
		
		// Select 4 random initial points
		int pointIndex1 = rand() % pointT.size();
		int pointIndex2 = rand() % pointT.size();
		int pointIndex3 = rand() % pointT.size();
		int pointIndex4 = rand() % pointT.size();
		
		// Unique indices / distinct points?
		if (pointIndex1 == pointIndex2 || pointIndex1 == pointIndex3 || pointIndex1 == pointIndex4 ||
				pointIndex2 == pointIndex3 || pointIndex2 == pointIndex4 || pointIndex3 == pointIndex4)
			continue;

		// Count only "real" iterations
		iterationNo++;
		
		// Calculate guess
		selecT.clear();
		selecT.push_back(pointT[pointIndex1]);
		selecT.push_back(pointT[pointIndex2]);
		selecT.push_back(pointT[pointIndex3]);
		selecT.push_back(pointT[pointIndex4]);
		
		selecS.clear();
		selecS.push_back(pointS[pointIndex1]);
		selecS.push_back(pointS[pointIndex2]);
		selecS.push_back(pointS[pointIndex3]);
		selecS.push_back(pointS[pointIndex4]);

		pose = AbsoluteOrientation::absoluteOrientation(selecT, selecS);

		supportingMatchesId.clear();
		
		// Count supporting matches for this guess
		for (int i=0; i < pointT.size(); i++) {
			Eigen::Vector4d oldLocationH;
			oldLocationH << pointT[i], 1;
			

			Eigen::Vector4d newLocationH = pose * oldLocationH;
			Eigen::Vector3d newLocation(newLocationH(0), newLocationH(1), newLocationH(2));
// #ifdef	CALIB_DEBUG
// 			DEBUG<<"NORM "<<cameraPoseEstimationRANSACMaxDistanceForInliers<<" BUT "<<(newLocation - pointS[i]).norm()<<endl;
// #endif
			if ((newLocation - pointS[i]).norm() < cameraPoseEstimationRANSACMaxDistanceForInliers) {
				supportingMatchesId.push_back(i);
			}
		}
#ifdef	CALIB_DEBUG
		DEBUG<<"Size: "<<supportingMatchesId.size()<<" ET "<<maxSupportingMatchesId.size()<<" OR "<<cameraPoseEstimationRANSACBreakRatio * pointT.size()<<endl;
#endif
		if (supportingMatchesId.size() > maxSupportingMatchesId.size())
			maxSupportingMatchesId = supportingMatchesId;

		if (maxSupportingMatchesId.size() > cameraPoseEstimationRANSACBreakRatio * pointT.size())
			break;

	}
	// if enough inliers
	selecT.clear();
	selecS.clear();
	if ((int)maxSupportingMatchesId.size() >= cameraPoseEstimationRANSACMinInliers) {
		// optimise with all inliers
		for (int i=0; i < (int)maxSupportingMatchesId.size(); i++) {
			selecT.push_back(pointT[i]);
			selecS.push_back(pointS[i]);
		}
		pose = AbsoluteOrientation::absoluteOrientation(selecT, selecS);
	}
	
	return pose;
}