#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


struct PointCloud {
	Point3f pos;
	Vec3b pixel;

	PointCloud(Point3f _pos, Vec3b _pixel) : pos(_pos), pixel(_pixel) {}
};


void writePLYFile(string filename, vector<PointCloud> pointClouds);