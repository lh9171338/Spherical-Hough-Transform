#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "pointcloud.h"

using namespace std;
using namespace cv;

#ifndef PI
#define PI (float)CV_PI
#endif

namespace sphericalcamera {

struct FisheyeIntrinsic {
	Matx33d cameraMatrix;
	Matx14d distCoeffs;
};

struct SphericalIntrinsic {
	FisheyeIntrinsic frontFisheyeIntrinsic;
	FisheyeIntrinsic backFisheyeIntrinsic;
};


/************************** Fisheye **************************/
void FoldFisheyeImage(const Mat& srcImg, Mat& dstImg, const float FOV);
void UnfoldFisheyeImage(const Mat& srcImg, Mat& dstImg, const float FOV, const float radius);

void ProjectFisheyePoints(vector<Point2f>& p2ds, vector<Point3f>& p3ds, FisheyeIntrinsic intrinsic, bool frontFlag = true);
void ReProjectFisheyePoints(vector<Point3f>& p3ds, vector<Point2f>& p2ds, FisheyeIntrinsic intrinsic, bool frontFlag = true);

void ProjectFisheyeImage(const Mat& image, vector<PointCloud>& pointClouds, FisheyeIntrinsic intrinsic, bool frontFlag = true);
void ReProjectFisheyeImage(Mat& image, vector<PointCloud>& pointClouds, FisheyeIntrinsic intrinsic, bool frontFlag = true, 
	Scalar color = Scalar(0, 0, 255), int thickness = 1);

void ProjectFisheyeEdge(const Mat& edge, vector<Point3f>& p3ds, FisheyeIntrinsic intrinsic);

/************************** Spherical Camera **************************/
void FoldSphericalImage(const Mat& srcImg, Mat& dstImg0, Mat& dstImg1, const float FOV);
void UnfoldSphericalImage(const Mat& srcImg0, const Mat& srcImg1, Mat& dstImg, const float FOV);

void ProjectSphericalImage(const Mat& image, vector<PointCloud>& pointClouds, SphericalIntrinsic intrinsic, const float FOV);
void ReProjectSphericalImage(Mat& image, vector<PointCloud>& pointClouds, SphericalIntrinsic intrinsic, const float FOV,
	Scalar color = Scalar(0, 0, 255), int thickness = 1);

void ProjectSphericalPoints(vector<Point2f>& p2ds, vector<Point3f>& p3ds, SphericalIntrinsic intrinsic, const Size imageSize, const float FOV);
void ProjectSphericalEdge(const Mat& edge, vector<Point3f>& p3ds, SphericalIntrinsic intrinsic, const float FOV);

}