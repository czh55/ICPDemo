
//https://blog.csdn.net/sunboyiris/article/details/72636809
//https://blog.csdn.net/baidu_26408419/article/details/72630664?locationNum=8&fps=1

#define PCD

#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#ifdef PLY
#include <pcl/io/ply_io.h>
#endif // PLY

#ifdef PCD
#include <pcl/io/pcd_io.h>
#endif // PCD



typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void
print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int
main(int argc,
	char* argv[])
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in_1(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_in_2(new PointCloudT);  // Original point cloud
	//PointCloudT::Ptr cloud_out_all(new PointCloudT);  // Output point cloud 
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	int iterations = 20;  // Default number of ICP iterations

	pcl::console::TicToc time;
	time.tic();

#ifdef PLY
	if (pcl::io::loadPLYFile("bunny.ply", *cloud_in_1) < 0 || pcl::io::loadPLYFile("bunny_part1.ply", *cloud_in_2) < 0)
#endif // PLY

#ifdef PCD
		if (pcl::io::loadPCDFile("rock_2.pcd", *cloud_in_1) < 0 || pcl::io::loadPCDFile("rock_1.pcd", *cloud_in_2))
#endif // PCD
		{
			PCL_ERROR("Error loading cloud %s.\n", "dragon.ply");
			system("pause");
			return (-1);
		}
	std::cout << "\nLoaded file " << "dragon.ply" << " (" << cloud_in_1->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	std::cout << "\nLoaded file " << "dragon.ply" << " (" << cloud_in_2->size() << " points) in " << time.toc() << " ms\n" << std::endl;

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud(*cloud_in_2, *cloud_tr, transformation_matrix);

	for (int j = 0; j < cloud_in_1->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = cloud_in_1->points[j].x;
		p.y = cloud_in_1->points[j].y;
		p.z = cloud_in_1->points[j].z;
		p.r = 255;//红色
		p.g = 0;
		p.b = 0;
		mergeCloud->points.push_back(p);
	}

	for (int j = 0; j < cloud_tr->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = cloud_tr->points[j].x;
		p.y = cloud_tr->points[j].y;
		p.z = cloud_tr->points[j].z;
		p.r = 20;//绿色
		p.g = 180;
		p.b = 20;
		mergeCloud->points.push_back(p);
	}
	// 设置并保存点云
	mergeCloud->height = 1;
	mergeCloud->width = mergeCloud->points.size();
	mergeCloud->is_dense = false;

#ifdef PLY
	pcl::io::savePLYFileBinary("Output.ply", *mergeCloud);
#endif // PLY

#ifdef PCD
	pcl::io::savePCDFile("ICPmerge2-in-out-icp.pcd", *mergeCloud);
#endif // PCD
	
	// 清除数据并退出
	mergeCloud->points.clear();
	std::cout << "已保存为ICPmerge2-in-out-icp.pcd" << std::endl;



	time.tic();

	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in_1, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in_1, cloud_in_color_h, "cloud_in_v1");

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1");


	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1");

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	system("pause");
	return (0);
}
