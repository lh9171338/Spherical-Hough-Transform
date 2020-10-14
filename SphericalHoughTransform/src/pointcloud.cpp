#include "pointcloud.h"


void writePLYFile(string filename, vector<PointCloud> pointClouds)
{
	int num = pointClouds.size();

	ofstream of(filename.c_str());
	of << "ply"
		<< '\n' << "format ascii 1.0"
		<< '\n' << "element vertex " << num
		<< '\n' << "property float x"
		<< '\n' << "property float y"
		<< '\n' << "property float z"
		<< '\n' << "property uchar blue"
		<< '\n' << "property uchar green"
		<< '\n' << "property uchar red"
		<< '\n' << "end_header" << endl;

	for (int i = 0; i < num; i++)
	{
		PointCloud pc = pointClouds[i];
		of << pc.pos.x << ' ' << pc.pos.y << ' ' << pc.pos.z << ' '
			<< (short)pc.pixel[0] << ' ' << (short)pc.pixel[1] << ' ' << (short)pc.pixel[2] << '\n';
	}

	of.close();
}