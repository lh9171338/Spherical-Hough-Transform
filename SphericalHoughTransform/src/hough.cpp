#include "hough.h"


namespace sphericalcamera {


struct HoughCmpFunc
{
	HoughCmpFunc(const int* _ptr) : ptr(_ptr) {}
	inline bool operator()(int l1, int l2) const
	{
		return ptr[l1] > ptr[l2] || (ptr[l1] == ptr[l2] && l1 < l2);
	}
	const int* ptr;
};

static void createTrigTable(
	float deltaAngle,
	float minAngle,
	float maxAngle,
	vector<float>& tabSin,
	vector<float>& tabCos)
{
	const float scale = PI / 180.0f;
	deltaAngle *= scale;
	minAngle *= scale;
	maxAngle *= scale;

	float angle = minAngle;
	float n = (maxAngle - minAngle) / deltaAngle;
	int numAngle = round((maxAngle - minAngle) / deltaAngle);
	tabSin.resize(numAngle);
	tabCos.resize(numAngle);

	for (int n = 0; n < numAngle; n++)
	{
		tabSin[n] = sin(angle);
		tabCos[n] = cos(angle);
		angle += deltaAngle;
	}
}

static void findLocalMaximums(
	int numPhi,
	int numTheta,
	int threshold,
	const int* ptrAccum,
	vector<int>& sortBuffer)
{
	for (int p = 0; p < numPhi; p++) {
		for (int t = 0; t < numTheta; t++) {
			int base = (p + 1) * (numTheta + 2) + t + 1;
			int val = ptrAccum[base];
			if (val > threshold&&
				val > ptrAccum[base - 1] && val > ptrAccum[base + 1] &&
				val > ptrAccum[base - numTheta - 2] && val > ptrAccum[base + numTheta + 2]) {
				sortBuffer.push_back(base);
			}
		}
	}
}

void getLines(
	Vec3f normal,
	vector<Point3f>& line,
	float deltaAngle,
	float minAngle,
	float maxAngle)
{
	normal = normalize(normal);
	float angle = acos(normal[2]);
	Vec3f axes = Vec3f(-normal[1], normal[0], 0.0f);
	axes = normalize(axes);
	Vec3f r = angle * axes;
	Mat R;	// 旋转矩阵
	Rodrigues(r, R);

	vector<float> tabSin, tabCos;
	createTrigTable(deltaAngle, minAngle, maxAngle, tabSin, tabCos);

	int numAngle = tabCos.size();
	line.resize(numAngle);
	for (int n = 0; n < numAngle; n++)
	{
		Vec3f pt0(tabCos[n], tabSin[n], 0.0f);
		Mat temp = R * Mat(pt0);
		Point3f pt(temp.at<float>(0), temp.at<float>(1), temp.at<float>(2));
		line[n] = pt;
	}
}

void drawLines(
	vector<PointCloud>& pointClouds,
	const vector<Point3f>& line,
	Scalar color)
{
	int numPoints = line.size();
	for (int i = 0; i < numPoints; i++) {
		Point3f pos = line[i];
		Vec3b pixel = Vec3b(color[0], color[1], color[2]);
		PointCloud pointCloud(pos, pixel);
		pointClouds.push_back(pointCloud);
	}
}

void drawLines(
	vector<PointCloud>& pointClouds,
	const vector<LineSpherical>& lines,
	float deltaAngle,
	Scalar color)
{
	const float scale = PI / 180.0f;

	int numLines = lines.size();
	for (int i = 0; i < numLines; i++) {
		float phi = lines[i].phi * scale;
		float theta = lines[i].theta * scale;
		float x = sin(phi) * cos(theta);
		float y = sin(phi) * sin(theta);
		float z = cos(phi);
		Vec3f normal = Vec3f(x, y, z);
		vector<Point3f> line;

		getLines(normal, line, deltaAngle, 0.0f, 360.0f);
		drawLines(pointClouds, line, color);
	}
}

void drawLines(
	vector<PointCloud>& pointClouds,
	const vector<Vec4f>& lines,
	float deltaAngle,
	Scalar color)
{
	const float scale = PI / 180.0f;

	int numLines = lines.size();
	for (int i = 0; i < numLines; i++) {
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
		vector<Point3f> line;
		getLines(normal, line, deltaAngle, minAngle, maxAngle + deltaAngle);
		drawLines(pointClouds, line, color);
	}
}

void HoughLinesStandard(
	const vector<Point3f>& points,
	vector<LineSpherical>& lines,
	float deltaAngle,
	int threshold,
	int linesMax)
{
	// 计算三角函数表
	vector<float> tabSin, tabCos;
	createTrigTable(deltaAngle, 0.0f, 360.0f, tabSin, tabCos);
	int numTab = tabSin.size();
	int numAngle = numTab / 2;
	float ideltaAngle = 1.0f / (deltaAngle * PI / 180.0f);

	// 初始化累加器
	Mat accum = Mat::zeros((numAngle + 2), (numAngle + 2), CV_32SC1);
	int* ptrAccum = accum.ptr<int>();
	vector<int> sortBuffer;

	// 累加计数
	int numPoints = points.size();
	for (int i = 0; i < numPoints; i++) {
		Vec3f normal = Vec3f(points[i]);
		normal = normalize(normal);
		float angle = acos(normal[2]);
		Vec3f axes = Vec3f(-normal[1], normal[0], 0.0f);
		axes = normalize(axes);
		Vec3f r = angle * axes;
		Mat R;	
		Rodrigues(r, R);

		for (int n = 0; n < numTab; n++) {
			Vec3f pt0(tabCos[n], tabSin[n], 0.0f);
			Mat temp = R * Mat(pt0);
			Point3f pt1(temp.at<float>(0), temp.at<float>(1), temp.at<float>(2));
			float phi = acos(pt1.z);
			float theta = atan2(pt1.y, pt1.x);
			int p = cvRound(phi * ideltaAngle);
			int t = (p == 0) ? 0 : cvRound(theta * ideltaAngle);
			if (p >= 0 && p < numAngle && t >= 0 && t < numAngle) {
				int base = (p + 1) * (numAngle + 2) + t + 1;
				ptrAccum[base]++;
			}
		}
	}

	// 寻找极大值
	findLocalMaximums(numAngle, numAngle, threshold, ptrAccum, sortBuffer);

	// 排序
	sort(sortBuffer.begin(), sortBuffer.end(), HoughCmpFunc(ptrAccum));

	// 保存直线
	linesMax = min(linesMax, (int)sortBuffer.size());
	lines.resize(linesMax);
	for (int i = 0; i < linesMax; i++) {
		LineSpherical line;
		int base = sortBuffer[i];
		int p = base / (numAngle + 2) - 1;
		int t = base % (numAngle + 2) - 1;
		line.phi = p * deltaAngle;
		line.theta = t * deltaAngle;
		line.count = ptrAccum[base];
		lines[i] = line;
	}
}

void HoughLinesProbabilistic(
	const vector<Point3f>& points,
	vector<Vec4f>& lines,
	float deltaAngle,
	float deltaPixel,
	int threshold,
	float lineLength,
	float lineGap,
	int linesMax)
{
	lines.clear();
	float ideltaAngle = 1.0f / (deltaAngle * PI / 180.0f);
	lineLength /= deltaPixel;

	// 计算三角函数表
	vector<float> tabSin0, tabCos0;
	createTrigTable(deltaAngle, 0.0f, 360.0f, tabSin0, tabCos0);
	int numTab = tabSin0.size();
	int numAngle = numTab / 2;
	
	vector<float> tabSin1, tabCos1;
	createTrigTable(deltaPixel, 0.0f, 360.0f, tabSin1, tabCos1);

	// 初始化累加器
	Mat accum = Mat::zeros(numAngle, numAngle, CV_32SC1);
	int* ptrAccum = accum.ptr<int>();

	// 初始化mask和nzloc
	int rows = cvRound(180.0f / deltaPixel);
	int cols = cvRound(360.0f / deltaPixel);
	Mat mask = Mat::zeros(rows + 1, cols, CV_8UC1);
	uchar* ptrMask = mask.ptr<uchar>();
	vector<Vec2i> pts;
	int numPoints = (int)points.size();
	for (int k = 0; k < numPoints; k++) {
		Vec3f pt = normalize(Vec3f(points[k]));
		float phi = acos(pt[2]);
		float theta = atan2(pt[1], pt[0]);
		if (theta < 0) 
			theta += 2 * PI;
		int p = cvRound(phi * rows / PI);
		int t = cvRound(theta * cols / (2 * PI)) % cols;
		if (ptrMask[p * cols + t] == 0) {
			ptrMask[p * cols + t] = 1;
			pts.push_back(Vec2i(p, t));
		}
	}
	numPoints = (int)pts.size();

	// 随机处理所有点
	RNG rng((uint64)-1);
	while(!pts.empty()) {
		// 随机选择一个点
		int idx = rng.uniform(0, numPoints); 
		Vec2i pt0 = pts[idx];
		pts.erase(pts.begin() + idx);
		numPoints--;

		// 检查是否处理过该点
		int p0 = pt0[0];
		int t0 = pt0[1];
		if (ptrMask[p0 * cols + t0] == 0) 
			continue;
		float phi0 = p0 * PI / rows;
		float theta0 = t0 * 2.0f * PI / cols;
		float x0 = tabSin1[p0] * tabCos1[t0];
		float y0 = tabSin1[p0] * tabSin1[t0];
		float z0 = tabCos1[p0];

		// 更新累加器
		ptrMask[p0 * cols + t0] = 2;

		Vec3f axes0 = Vec3f(-tabSin1[t0], tabCos1[t0], 0.0f);
		axes0 = normalize(axes0);
		Vec3f r0 = phi0 * axes0;
		Mat R0;
		Rodrigues(r0, R0);
		int max_val = threshold - 1;
		int linep = 0, linet = 0;
		for (int n = 0; n < numTab; n++) {
			Vec3f pt1(tabCos0[n], tabSin0[n], 0.0f);
			Mat temp = R0 * Mat(pt1);
			Point3f pt2(temp.at<float>(0), temp.at<float>(1), temp.at<float>(2));
			float phi = acos(pt2.z);
			float theta = atan2(pt2.y, pt2.x);
			int p = cvRound(phi * ideltaAngle);
			int t = (p == 0) ? 0 : cvRound(theta * ideltaAngle);
			if (p >= 0 && p < numAngle && t >= 0 && t < numAngle) {
				ptrAccum[p * numAngle + t]++;
				if (ptrAccum[p * numAngle + t] > max_val) {
					max_val = ptrAccum[p * numAngle + t];
					linep = p;
					linet = t;
				}
			}
		}

		// 检查是否有超过阈值的候选直线
		if (max_val < threshold) 
			continue;
			
		// 从当前的点向两边延伸，寻找端点
		float linePhi = linep * deltaAngle * PI / 180.0f;
		Vec3f linePt = Vec3f(x0, y0, z0);
		Vec3f lineAxes = Vec3f(-tabSin0[linet], tabCos0[linet], 0.0f);
		lineAxes = normalize(lineAxes);
		Vec3f liner = linePhi * lineAxes;
		Mat lineR;
		Rodrigues(liner, lineR);
		Mat temp = lineR.t() * Mat(linePt);
		float angle = atan2(temp.at<float>(1), temp.at<float>(0));
		angle = (angle < 0) ? (angle + 2.0f * PI) : angle;
		int n0 = cvRound(angle * cols / (2.0f * PI));

		Vec2i line_end(n0, n0);
		for (int k = 0; k < 2; k++) {
			int gap = 0;
			int dn = (k == 0) ? 1 : -1;
			int n = n0;
			for (;; n += dn) {
				n = (n >= cols) ? (n - cols) : ((n < 0) ? (n + cols) : n);
				Vec3f pt1(tabCos1[n], tabSin1[n], 0.0f);
				Mat temp = lineR * Mat(pt1);
				Point3f pt2(temp.at<float>(0), temp.at<float>(1), temp.at<float>(2));
				float phi = acos(pt2.z);
				float theta = atan2(pt2.y, pt2.x);
				theta = (theta < 0) ? (theta + 2.0f * PI) : theta;
				int p = cvRound(phi * rows / PI);
				int t = cvRound(theta * cols / (2.0f * PI)) % cols;

				if (ptrMask[p * cols + t] > 0) {
					gap = 0;
					line_end[1 - k] = n;

					// 清除累加器
					phi = p * PI / rows;
					Vec3f axes = Vec3f(-tabSin1[t], tabCos1[t], 0.0f);
					axes = normalize(axes);
					Vec3f r = phi * axes;
					Mat R;
					Rodrigues(r, R);

					if (ptrMask[p * cols + t] == 2) {
						for (int i = 0; i < numTab; i++) {
							Vec3f pt3(tabCos0[i], tabSin0[i], 0.0f);
							Mat temp = R * Mat(pt3);
							Point3f pt4(temp.at<float>(0), temp.at<float>(1), temp.at<float>(2));
							float phi3 = acos(pt4.z);
							float theta3 = atan2(pt4.y, pt4.x);
							int p3 = cvRound(phi3 * ideltaAngle);
							int t3 = (p3 == 0) ? 0 : cvRound(theta3 * ideltaAngle);
							if (p3 >= 0 && p3 < numAngle && t3 >= 0 && t3 < numAngle)
								ptrAccum[p3 * numAngle + t3]--;
						}
					}

					ptrMask[p * cols + t] = 0;
				}
				else if (++gap > lineGap)
					break;
			}
		}

		int length = (line_end[1] >= line_end[0]) ? (line_end[1] - line_end[0]) : (line_end[1] + cols - line_end[0]);
		bool good_line = length >= lineLength;

		// 保存直线
		if (good_line) {
			float phi = linep * deltaAngle;
			float theta = linet * deltaAngle;
			float angle0 = line_end[0] * deltaPixel;
			float angle1 = line_end[1] * deltaPixel;
			Vec4f line(phi, theta, angle0, angle1);
			lines.push_back(line);
			if ((int)lines.size() >= linesMax)
				return;
		}
	}
}

}