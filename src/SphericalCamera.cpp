#include "SphericalCamera.h"


namespace sphericalcamera {


/************************** Fisheye **************************/

void FoldFisheyeImage(
	const Mat& srcImg,
	Mat& dstImg,
	const float FOV)
{
	int rows0 = srcImg.rows;
	int cols0 = srcImg.cols;
	int rows1, cols1;
	if (FOV <= 180.0f)
	{
		rows1 = cvRound(rows0 * 180.0f / FOV);
		cols1 = cvRound(cols0 * 180.0f / FOV);
	}
	else
	{
		rows1 = rows0;
		cols1 = cvRound(cols0 / 2.0f);
	}
	float cx = cols1 / 2.0f;
	float cy = rows1 / 2.0f;
	float radius = rows1 / 2.0f;
	dstImg = Mat::zeros(rows1, cols1, srcImg.type());

	for (int i = 0; i < rows1; i++)
	{
		for (int j = 0; j < cols1; j++)
		{
			float x = j - cx;
			float y = i - cy;
			float r = (float)sqrt(x * x + y * y);
			float theta = (float)atan2(y, x);
			float ratio = r / radius;
			if (ratio <= 1)
			{
				float phi = ratio * (FOV / 2.0f) * PI / 180.0f;
				float Px = sin(phi) * cos(theta);
				float Py = sin(phi) * sin(theta);
				float Pz = cos(phi);

				float latitude = acos(Py);
				float longitude = atan2(Pz, Px);
				float u = (PI - longitude) * cols1 / PI;
				float v = (PI - latitude) * rows1 / PI;

				/* 双线性插值 */
				int u0 = (int)floor(u);
				int v0 = (int)floor(v);
				if (u0 >= 0 && v0 >= 0 && (u0 + 1) < cols0 && (v0 + 1) < rows0)
				{
					float dx = u - u0;
					float dy = v - v0;
					float w00 = (1 - dx) * (1 - dy);
					float w01 = dx * (1 - dy);
					float w10 = (1 - dx) * dy;
					float w11 = dx * dy;

					if (srcImg.channels() == 3)
					{
						Vec3f pixel = w00 * srcImg.at<Vec3b>(v0, u0) + w01 * srcImg.at<Vec3b>(v0, u0 + 1) +
							w10 * srcImg.at<Vec3b>(v0 + 1, u0) + w11 * srcImg.at<Vec3b>(v0 + 1, u0 + 1);
						dstImg.at<Vec3b>(i, j) = Vec3b(pixel);
					}
					else if (srcImg.channels() == 1)
					{
						float pixel = w00 * srcImg.at<uchar>(v0, u0) + w01 * srcImg.at<uchar>(v0, u0 + 1) +
							w10 * srcImg.at<uchar>(v0 + 1, u0) + w11 * srcImg.at<uchar>(v0 + 1, u0 + 1);
						dstImg.at<uchar>(i, j) = uchar(pixel);
					}
				}
				else
				{
					continue;
				}
			}
		}
	}
}

void UnfoldFisheyeImage(
	const Mat& srcImg,
	Mat& dstImg,
	const float FOV,
	const float radius)
{
	int rows0 = srcImg.rows;
	int cols0 = srcImg.cols;
	int rows1, cols1;
	if (FOV <= 180.0f)
	{
		rows1 = cvRound(rows0 * FOV / 180.0f);
		cols1 = cvRound(cols0 * FOV / 180.0f);
	}
	else
	{
		rows1 = rows0;
		cols1 = 2 * cols0;
	}
	float cx = cols0 / 2.0f;
	float cy = rows0 / 2.0f;
	dstImg = Mat::zeros(rows1, cols1, srcImg.type());

	for (int i = 0; i < rows1; i++)
	{
		float latitude = PI - i * PI / rows0;
		float sinlatitude = sin(latitude);
		float coslatitude = cos(latitude);
		for (int j = 0; j < cols1; j++)
		{
			float longitude = PI - j * PI / cols0;
			float Px = sinlatitude * cos(longitude);
			float Py = coslatitude;
			float Pz = sinlatitude * sin(longitude);

			float phi = acos(Pz);
			float ratio = phi * 180 / PI / (FOV / 2);
			float theta = atan2(Py, Px);
			float r = radius * ratio;
			float u = r * cos(theta) + cx;
			float v = r * sin(theta) + cy;

			/* 双线性插值 */
			int u0 = (int)floor(u);
			int v0 = (int)floor(v);
			if (u0 >= 0 && v0 >= 0 && (u0 + 1) < cols0 && (v0 + 1) < rows0)
			{
				float dx = u - u0;
				float dy = v - v0;
				float w00 = (1 - dx) * (1 - dy);
				float w01 = dx * (1 - dy);
				float w10 = (1 - dx) * dy;
				float w11 = dx * dy;

				if (srcImg.channels() == 3)
				{
					Vec3f pixel = w00 * srcImg.at<Vec3b>(v0, u0) + w01 * srcImg.at<Vec3b>(v0, u0 + 1) +
						w10 * srcImg.at<Vec3b>(v0 + 1, u0) + w11 * srcImg.at<Vec3b>(v0 + 1, u0 + 1);
					dstImg.at<Vec3b>(i, j) = Vec3b(pixel);
				}
				else if (srcImg.channels() == 1)
				{
					float pixel = w00 * srcImg.at<uchar>(v0, u0) + w01 * srcImg.at<uchar>(v0, u0 + 1) +
						w10 * srcImg.at<uchar>(v0 + 1, u0) + w11 * srcImg.at<uchar>(v0 + 1, u0 + 1);
					dstImg.at<uchar>(i, j) = uchar(pixel);
				}
			}
		}
	}
}

void ProjectFisheyePoints(
	vector<Point2f>& p2ds,
	vector<Point3f>& p3ds,
	FisheyeIntrinsic intrinsic,
	bool frontFlag
)
{
	vector<Point2f> undistorted;
	fisheye::undistortPoints(p2ds, undistorted, intrinsic.cameraMatrix, intrinsic.distCoeffs);

	int npts = (int)undistorted.size();
	for (int i = 0; i < npts; i++) {
		Vec3f vec(undistorted[i].x, undistorted[i].y, 1.0f);
		vec = normalize(vec);
		Point3f p3d(vec);
		if (!frontFlag) {
			p3d.x = -p3d.x;
			p3d.z = -p3d.z;
		}
		p3ds.push_back(p3d);
	}
}

void ReProjectFisheyePoints(
	vector<Point3f>& p3ds,
	vector<Point2f>& p2ds,
	FisheyeIntrinsic intrinsic,
	bool frontFlag
)
{
	vector<Point2f> undistorted;
	int npts = (int)p3ds.size();
	for (int i = 0; i < npts; i++) {
		Point3f p3d = p3ds[i];
		if (!frontFlag) {
			p3d.x = -p3d.x;
			p3d.z = -p3d.z;
		}

		if (p3d.z < 0)
			continue;

		float theta = atan2(p3d.y, p3d.x);
		float phi = acos(p3d.z);
		float r = tan(phi);
		float x = r * cos(theta);
		float y = r * sin(theta);

		undistorted.push_back(Point2f(x, y));
	}

	fisheye::distortPoints(undistorted, p2ds, intrinsic.cameraMatrix, intrinsic.distCoeffs);
}

void ProjectFisheyeImage(
	const Mat& image, 
	vector<PointCloud>& pointClouds,
	FisheyeIntrinsic intrinsic,
	bool frontFlag
)
{
	int rows = image.rows;
	int cols = image.cols;
	float cx = cols / 2.0f;
	float cy = rows / 2.0f;
	float radius = rows / 2.0f;

	vector<Point2f> p2ds;
	vector<Vec3b> pixels;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			float x = j - cx;
			float y = i - cy;
			float r = sqrt(x * x + y * y);
			if (r <= radius) {
				p2ds.push_back(Point2f(j, i));
				pixels.push_back(image.at<Vec3b>(i, j));
			}
		}
	}

	vector<Point3f> p3ds;
	ProjectFisheyePoints(p2ds, p3ds, intrinsic, frontFlag);
	int npts = (int)p3ds.size();
	for (int i = 0; i < npts; i++) 
		pointClouds.push_back(PointCloud(p3ds[i], pixels[i]));
}

void ReProjectFisheyeImage(
	Mat& image, 
	vector<PointCloud>& pointClouds, 
	FisheyeIntrinsic intrinsic,
	bool frontFlag,
	Scalar color,
	int thickness
)
{
	int rows = image.rows;
	int cols = image.cols;

	vector<Point2f> p2ds;
	vector<Point3f> p3ds;
	vector<Vec3b> pixels;
	int npts = (int)pointClouds.size();
	for (int i = 0; i < npts; i++) {
		Point3f p3d = pointClouds[i].pos;
		if (!frontFlag) {
			p3d.x = -p3d.x;
			p3d.z = -p3d.z;
		}
		if (p3d.z < 0)
			continue;

		p3ds.push_back(pointClouds[i].pos);
		pixels.push_back(pointClouds[i].pixel);
	}
		
	ReProjectFisheyePoints(p3ds, p2ds, intrinsic, frontFlag);
	
	npts = (int)p3ds.size();
	Mat mask = Mat::zeros(rows, cols, CV_8UC1);
	for (int i = 0; i < npts; i++) {
		int x = cvRound(p2ds[i].x);
		int y = cvRound(p2ds[i].y);
		if (x >= 0 && x < cols && y >= 0 && y < rows)
			mask.at<uchar>(y, x) = 255;
	}

	if (thickness > 1) {
		Mat element = getStructuringElement(MORPH_RECT, Size(thickness, thickness));
		dilate(mask.clone(), mask, element);
	}

	image.setTo(color, mask);
}

/************************** Spherical Camera **************************/
void FoldSphericalImage(
	const Mat& srcImg,
	Mat& dstImg0,
	Mat& dstImg1,
	const float FOV)
{
	int rows = srcImg.rows;
	int cols = srcImg.cols / 2;
	Mat srcImg0 = srcImg(Rect(0, 0, cols, rows)).clone();
	Mat srcImg1 = srcImg(Rect(cols, 0, cols, rows)).clone();

	FoldFisheyeImage(srcImg0, dstImg0, FOV);
	FoldFisheyeImage(srcImg1, dstImg1, FOV);
}

void UnfoldSphericalImage(
	const Mat& srcImg0,
	const Mat& srcImg1,
	Mat& dstImg,
	const float FOV)
{
	int rows = srcImg0.rows;
	int cols = srcImg0.cols;
	float radius = min(rows, cols) / 2.0f;

	Mat dstImg0, dstImg1;
	UnfoldFisheyeImage(srcImg0, dstImg0, FOV, radius);
	UnfoldFisheyeImage(srcImg1, dstImg1, FOV, radius);

	dstImg = Mat::zeros(rows, 2 * cols, srcImg0.type());
	dstImg0.copyTo(dstImg(Rect(0, 0, cols, rows)));
	dstImg1.copyTo(dstImg(Rect(cols, 0, cols, rows)));
}

void ProjectSphericalImage(
	const Mat& image, 
	vector<PointCloud>& pointClouds,
	SphericalIntrinsic intrinsic,
	const float FOV
)
{
	Mat dstImg0, dstImg1;
	FoldSphericalImage(image, dstImg0, dstImg1, FOV);
	ProjectFisheyeImage(dstImg0, pointClouds, intrinsic.frontFisheyeIntrinsic, true);
	ProjectFisheyeImage(dstImg1, pointClouds, intrinsic.backFisheyeIntrinsic, false);
}

void ReProjectSphericalImage(
	Mat& image, 
	vector<PointCloud>& pointClouds, 
	SphericalIntrinsic intrinsic, 
	const float FOV,
	Scalar color,
	int thickness
)
{
	Mat dstImg0, dstImg1;

	FoldSphericalImage(image, dstImg0, dstImg1, FOV);
	ReProjectFisheyeImage(dstImg0, pointClouds, intrinsic.frontFisheyeIntrinsic, true, color, thickness);
	ReProjectFisheyeImage(dstImg1, pointClouds, intrinsic.backFisheyeIntrinsic, false, color, thickness);
	UnfoldSphericalImage(dstImg0, dstImg1, image, FOV);
}


void ProjectSphericalPoints(
	vector<Point2f>& p2ds,
	vector<Point3f>& p3ds,
	SphericalIntrinsic intrinsic,
	const Size imageSize,
	const float FOV)
{
	int rows = imageSize.height;
	int cols = imageSize.width / 2;
	float cx = cols / 2.0f;
	float cy = rows / 2.0f;

	int npts = (int)p2ds.size();
	vector<Point2f> frontP2ds, backP2ds;
	for (int i = 0; i < npts; i++) {
		bool frontFlag = true;
		float x0 = p2ds[i].x;
		float y0 = p2ds[i].y;
		if (x0 >= cols) {
			x0 -= cols;
			frontFlag = false;
		}

		float radius = rows / 2.0f;

		float latitude = PI - y0 * PI / rows;
		float sinlatitude = sin(latitude);
		float coslatitude = cos(latitude);
		float longitude = PI - x0 * PI / cols;
		float Px = sinlatitude * cos(longitude);
		float Py = coslatitude;
		float Pz = sinlatitude * sin(longitude);

		float phi = acos(Pz);
		float ratio = phi * 180 / PI / (FOV / 2);
		float theta = atan2(Py, Px);
		float r = radius * ratio;
		float u = r * cos(theta) + cx;
		float v = r * sin(theta) + cy;
		Point2f p2d(u, v);

		if (frontFlag) {
			frontP2ds.push_back(p2d);
		}
		else {
			backP2ds.push_back(p2d);
		}
	}

	ProjectFisheyePoints(frontP2ds, p3ds, intrinsic.frontFisheyeIntrinsic, true);
	ProjectFisheyePoints(backP2ds, p3ds, intrinsic.backFisheyeIntrinsic, false);
}

void ProjectSphericalEdge(
	const Mat& edge, 
	vector<Point3f>& p3ds, 
	SphericalIntrinsic intrinsic,
	const float FOV
	)
{
	int rows = edge.rows;
	int cols = edge.cols;
	vector<Point2f> p2ds;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			uchar value = edge.at<uchar>(i, j);
			if (value) 
				p2ds.push_back(Point2f(j, i));
		}
	}

	ProjectSphericalPoints(p2ds, p3ds, intrinsic, edge.size(), FOV);
}

}