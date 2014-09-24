/*! 
* 	\file    orientation.h
* 	\author  Gabriel Urbain <gurbain@mit.edu> - Visiting student at MIT SSL
* 	\date    August 2014
* 	\version 0.1
* 	\brief   Absolute orientation solver. Adapted from Muggler sources in VERTIGO Project
*
* 	License: The MIT License (MIT)
* 	Copyright (c) 2014, Massachussets Institute of Technology
*/

#ifndef ORIENTATION_HH_
#define ORIENTATION_HH_

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>

#include "utils.h"

#define RANSAC_MAX_ITER	20

using namespace std;
using namespace Eigen;

class AbsoluteOrientation {
	
		double cameraPoseEstimationRANSACMaxDistanceForInliers;
		double cameraPoseEstimationRANSACBreakRatio;
		int cameraPoseEstimationRANSACMinInliers;
		
	public:
		AbsoluteOrientation();
		static Matrix4d absoluteOrientation(vector<Vector3d, aligned_allocator<Vector3d> > & left, vector<Vector3d, aligned_allocator<Vector3d> > & right);
		Matrix4d orientationRANSAC(vector<Vector3d, aligned_allocator<Vector3d> > & left, vector<Vector3d, aligned_allocator<Vector3d> > & right);

		// 32-bit issue
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* ORIENTATION_HH_ */
