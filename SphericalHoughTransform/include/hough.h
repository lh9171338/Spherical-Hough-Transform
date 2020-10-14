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

struct LineSpherical
{
	float phi;
	float theta;
	int count;
};

void getLines(
	Vec3f normal,
	vector<Point3f>& line,
	float deltaAngle = 1.0f,
	float minAngle = 0.0f,
	float maxAngle = 360.0f);

void drawLines(
	vector<PointCloud>& pointClouds,
	const vector<Point3f>& line,
	Scalar color = Scalar(0, 0, 255));

void drawLines(
	vector<PointCloud>& pointClouds,
	const vector<LineSpherical>& lines,
	float deltaAngle = 1.0f,
	Scalar color = Scalar(0, 0, 255));

void drawLines(
	vector<PointCloud>& pointClouds,
	const vector<Vec4f>& lines,
	float deltaAngle = 1.0f,
	Scalar color = Scalar(0, 0, 255));

void HoughLinesStandard(
	const vector<Point3f>& points,
	vector<LineSpherical>& lines,
	float deltaAngle,
	int threshold,
	int linesMax = INT_MAX);

void HoughLinesProbabilistic(
	const vector<Point3f>& points,
	vector<Vec4f>& lines,
	float deltaAngle,
	float deltaPixel,
	int threshold,
	float lineLength, 
	float lineGap = 0,
	int linesMax = INT_MAX);

}

