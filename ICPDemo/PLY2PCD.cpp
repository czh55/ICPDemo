//main.cpp
//https://blog.csdn.net/xiangxianghehe/article/details/80009824
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::io;

int main(int argc, char** argv)
{
	pcl::PLYReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read<pcl::PointXYZ>("bunny_part2.ply", *cloud);
	pcl::io::savePCDFile("bunny_part2.pcd", *cloud);

	return 0;
}