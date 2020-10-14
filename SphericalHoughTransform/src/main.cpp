#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include "SphericalCamera.h"
#include "hough.h"
#include "pointcloud.h"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;


int main()
{
	/******************* Parameters *******************/
	string srcImageFilename = "image/src.jpg";
	string dstImageFilename = "image/dst.jpg";
	String modelFilename = "param/model.yml";
	string intrinsicFilename0 = "param/default.yaml";
	string intrinsicFilename1 = "param/default.yaml";
	string pointclondFilename = "image/dst.ply";
	string outputFilename = "image/arcs.txt";
	bool saveImagePointcloudFlag = false;
	bool saveEdgePointcloudFlag = false;
	bool saveLinePointcloudFlag = true;
	bool saveDstImageFlag = true;
	bool saveOutputFlag = true;
	bool showFlag = true;
	bool thinFlag = false;
	float FOV = 180.0f;

	/******************* Read camera parameter *******************/
	sphericalcamera::SphericalIntrinsic sphericalIntrinsic;
	FileStorage fs0(intrinsicFilename0, FileStorage::READ);
	if (!fs0.isOpened())
		cerr << "Open File Failed!" << endl;
	fs0["K"] >> sphericalIntrinsic.frontFisheyeIntrinsic.cameraMatrix;
	fs0["D"] >> sphericalIntrinsic.frontFisheyeIntrinsic.distCoeffs;

	FileStorage fs1(intrinsicFilename1, FileStorage::READ);
	if (!fs1.isOpened())
		cerr << "Open File Failed!" << endl;
	fs1["K"] >> sphericalIntrinsic.backFisheyeIntrinsic.cameraMatrix;
	fs1["D"] >> sphericalIntrinsic.backFisheyeIntrinsic.distCoeffs;

	/******************* Read image *******************/
	Mat srcImage = imread(srcImageFilename);
	if (srcImage.empty())
		cerr << "Read image Failed!" << endl;

	/******************* Preprocess image *******************/
	// It't not necessary.
	int rows = srcImage.rows;
	int cols = srcImage.cols / 2;
	Mat temp = srcImage.clone();
	temp(Rect(cols / 2, 0, cols * 3 / 2, rows)).copyTo(srcImage(Rect(0, 0, cols * 3 / 2, rows)));
	temp(Rect(0, 0, cols / 2, rows)).copyTo(srcImage(Rect(cols * 3 / 2, 0, cols / 2, rows)));

	/******************* Detect edge *******************/
	Mat image, edge;
	Mat orientation;
	Mat finalEdge;
	srcImage.convertTo(image, DataType<float>::type, 1 / 255.0);
	Ptr<StructuredEdgeDetection> pDollar = createStructuredEdgeDetection(modelFilename);
	pDollar->detectEdges(image, edge);
	if (thinFlag) {
		edge.convertTo(edge, DataType<uchar>::type, 255.0);
		threshold(edge.clone(), edge, 0, 255, THRESH_OTSU);
		thinning(edge, edge, THINNING_ZHANGSUEN);
	}
	else {
		pDollar->computeOrientation(edge, orientation);
		pDollar->edgesNms(edge.clone(), orientation, edge, 2, 0, 1, true);
		edge.convertTo(edge, DataType<uchar>::type, 255.0);
		threshold(edge.clone(), edge, 0, 255, THRESH_OTSU);
	}
	
	/******************* Back-projection of edge points *******************/
	vector<Point3f> points;
	sphericalcamera::ProjectSphericalEdge(edge, points, sphericalIntrinsic, FOV);
	cout << "Number of edge points: " << points.size() << endl;
	
	/******************* Detect line segments *******************/
	vector<Vec4f> lines;
	sphericalcamera::HoughLinesProbabilistic(points, lines, 0.2f, 0.2f, 50, 5, 5, 300);
	cout << "Number of line segments: " << lines.size() << endl;


	/******************* Save pointclouds *******************/
	vector<PointCloud> pointClouds;
	if (saveImagePointcloudFlag)
		sphericalcamera::ProjectSphericalImage(srcImage, pointClouds, sphericalIntrinsic, FOV);
	if (saveEdgePointcloudFlag)
		sphericalcamera::drawLines(pointClouds, points, Scalar(0, 255, 0));
	if (saveLinePointcloudFlag)
		sphericalcamera::drawLines(pointClouds, lines, 0.1f, Scalar(0, 0, 255));
	if (saveImagePointcloudFlag || saveEdgePointcloudFlag || saveLinePointcloudFlag) {
		writePLYFile(pointclondFilename, pointClouds);
		cout << "Number of pointclouds: " << pointClouds.size() << endl;
	}
	
	/******************* Reproject line segments *******************/
	Mat dstImage = srcImage.clone();
	pointClouds.clear();
	sphericalcamera::drawLines(pointClouds, lines, 0.1f, Scalar(0, 0, 255));
	sphericalcamera::ReProjectSphericalImage(dstImage, pointClouds, sphericalIntrinsic, FOV, Scalar(0, 0, 255), 5);
	if (saveDstImageFlag)
		imwrite(dstImageFilename, dstImage);

	/******************* Save line segments *******************/
	if (saveOutputFlag)
	{
		ofstream outputFile(outputFilename.c_str(), ofstream::out);
		if (outputFile.is_open())
		{
			const float scale = PI / 180.0f;
			int numLines = (int)lines.size();
			for (int i = 0; i < numLines; i++)
			{
				float phi = lines[i][0] * scale;
				float theta = lines[i][1] * scale;
				float x = sin(phi) * cos(theta);
				float y = sin(phi) * sin(theta);
				float z = cos(phi);
				Vec3f normal = Vec3f(x, y, z);
				float minAngle = lines[i][2];
				float maxAngle = lines[i][3];
				if (maxAngle < minAngle) {
					maxAngle += 360.0f;
				}
				float deltaAngle = maxAngle - minAngle;
				vector<Point3f> line;
				sphericalcamera::getLines(normal, line, deltaAngle, minAngle, maxAngle + deltaAngle);
				if (line.size() != 2) {
					cout << "error" << endl;
					return -1;
				}
				outputFile << scientific << setprecision(7) <<
					line[0].x << " " << line[0].y << " " << line[0].z << " " <<
					line[1].x << " " << line[1].y << " " << line[1].z << endl;
			}
			outputFile.close();
		}
	}

	/******************* Display result *******************/
	if (showFlag) {
		namedWindow("src image", 0);
		imshow("src image", srcImage);
		namedWindow("edge", 0);
		imshow("edge", edge);
		namedWindow("dst image", 0);
		imshow("dst image", dstImage);
		waitKey();
	}

	return 0;
}