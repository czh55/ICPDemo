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

void 
savePointCloudFile(PointCloudT::Ptr cloud_in_1, PointCloudT::Ptr cloud_icp, int iterations) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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

	for (int j = 0; j < cloud_icp->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = cloud_icp->points[j].x;
		p.y = cloud_icp->points[j].y;
		p.z = cloud_icp->points[j].z;
		p.r = 20;//绿色
		p.g = 180;
		p.b = 20;
		mergeCloud->points.push_back(p);
	}
	// 设置并保存点云
	mergeCloud->height = 1;
	mergeCloud->width = mergeCloud->points.size();
	mergeCloud->is_dense = false;

	std::stringstream ss;
	ss << iterations;
	std::string fileName = "ICPmerge2-in-out-icp-" + ss.str() + ".pcd";

#ifdef PLY
	pcl::io::savePLYFileBinary("Output.ply", *mergeCloud);
#endif // PLY

#ifdef PCD
	pcl::io::savePCDFile(fileName, *mergeCloud);
#endif // PCD

	// 清除数据并退出
	mergeCloud->points.clear();
	std::cout << "已保存为"<< fileName << std::endl;

}

int
main(int argc,
	char* argv[])
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in_1(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_in_2(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_in_all(new PointCloudT);  // Original point cloud 
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

	int iterations = 10;  // Default number of ICP iterations

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
	transformation_matrix(0, 0) =  cos(theta);
	transformation_matrix(0, 1) =  -sin(theta);
	transformation_matrix(1, 0) =  sin(theta);
	transformation_matrix(1, 1) =  cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud(*cloud_in_2, *cloud_icp, transformation_matrix);
	
	*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

	// The Iterative Closest Point algorithm
	time.tic();

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in_1);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;


	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);

		//update file
		savePointCloudFile(cloud_in_1, cloud_icp, iterations);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two vertically separated viewports
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in_1, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in_1, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in_1, cloud_in_color_h, "cloud_in_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// Orginal point cloud is blue
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_all_color_h(cloud_in_all, 0, 191, 255);
	//viewer.addPointCloud(cloud_in_all, cloud_all_color_h, "cloud_all_v2", v2);

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		// The user pressed "space" :
		if (next_iteration)
		{
			// The Iterative Closest Point algorithm
			time.tic();
			icp.align(*cloud_icp);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");

				//update file
				savePointCloudFile(cloud_in_1, cloud_icp, iterations);
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				system("pause");
				return (-1);
			}
		}
		next_iteration = false;
	}
	system("pause");
	return (0);
}
