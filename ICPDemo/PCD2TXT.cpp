//Ô­ÎÄ£ºhttps ://blog.csdn.net/j_cou/article/details/80329454?utm_source=copy 
#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  

using namespace std;

int main(int argc, char *argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data  
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file chuli.pcd\n");
		return (-1);
	}
	//for (size_t i = 0; i < cloud->points.size(); i++)
	//  std::cout << " " << cloud->points[i].x << " " << cloud->points[i].y
	//  <<" "<< cloud->points[i].z << std::endl;
	int Num = cloud->points.size();
	double *X = new double[Num] {0};
	double *Y = new double[Num] {0};
	double *Z = new double[Num] {0};
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		X[i] = cloud->points[i].x;
		Y[i] = cloud->points[i].y;
		Z[i] = cloud->points[i].z;
	}

	ofstream zos("result_pcd2txt/chuli.txt");
	for (int i = 0; i<Num; i++)
	{
		zos << X[i] << " " << Y[i] << " " << Z[i] << endl;
	}
	cout << "trans has done!!!" << endl;
	//cin.get();
	system("pause");
	return 0;
}

