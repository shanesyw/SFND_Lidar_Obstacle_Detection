/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <iostream>
#include <pcl/console/print.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	
	while(maxIterations--)
	{
		std::unordered_set<int> testInliers;

		while(testInliers.size() < 3)
			testInliers.insert(rand()%(cloud->points.size()));

		auto itr = testInliers.begin();

		pcl::PointXYZ point1 = cloud->points[*itr]; //*(testInliers.begin())
		itr++;
		pcl::PointXYZ point2 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ point3 = cloud->points[*itr];


		pcl::PointXYZ v1;
		v1.x = point2.x - point1.x;
		v1.y = point2.y - point1.y;
		v1.z = point2.z - point1.z;

		pcl::PointXYZ v2;
		v2.x = point3.x - point1.x;
		v2.y = point3.y - point1.y;
		v2.z = point3.z - point1.z;

		float coeff_i, coeff_j, coeff_k;

		coeff_i = v1.y * v2.z - v1.z * v2.y;
		coeff_j = v1.z * v2.x - v1.x * v2.z;
		coeff_k = v1.x * v2.y - v1.y * v2.z;

		float stdcoeff_a = coeff_i;
		float stdcoeff_b = coeff_j;
		float stdcoeff_c = coeff_k;
		float stdcoeff_d = -(coeff_i*point1.x + coeff_j*point1.y + coeff_k*point1.z);


		for(int j = 0; j < cloud->points.size(); j++) 
		{
			if(testInliers.count(j)>0)
				continue;

			pcl::PointXYZ testPoint = cloud->points[j];
			float tempDist = fabs(testPoint.x * stdcoeff_a + testPoint.y * stdcoeff_b + testPoint.z * stdcoeff_c + stdcoeff_d)/
					sqrt(stdcoeff_a*stdcoeff_a+stdcoeff_b*stdcoeff_b+stdcoeff_c*stdcoeff_c);
			
			if(tempDist <= distanceTol)
				testInliers.insert(j);
		
		}

		if(testInliers.size() > inliersResult.size())
			inliersResult = testInliers;
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac routine took " << elapsedTime.count() << " milliseconds." << std::endl;

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}
*/
/*
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	
	while(maxIterations--)
	{
		std::unordered_set<int> testInliers;

		while(testInliers.size() < 3)
			testInliers.insert(rand()%(cloud->points.size()));

		auto itr = testInliers.begin();

		pcl::PointXYZI point1 = cloud->points[*itr]; //*(testInliers.begin())
		itr++;
		pcl::PointXYZI point2 = cloud->points[*itr];
		itr++;
		pcl::PointXYZI point3 = cloud->points[*itr];


		pcl::PointXYZI v1;
		v1.x = point2.x - point1.x;
		v1.y = point2.y - point1.y;
		v1.z = point2.z - point1.z;

		pcl::PointXYZI v2;
		v2.x = point3.x - point1.x;
		v2.y = point3.y - point1.y;
		v2.z = point3.z - point1.z;

		float coeff_i, coeff_j, coeff_k;

		coeff_i = v1.y * v2.z - v1.z * v2.y;
		coeff_j = v1.z * v2.x - v1.x * v2.z;
		coeff_k = v1.x * v2.y - v1.y * v2.z;

		float stdcoeff_a = coeff_i;
		float stdcoeff_b = coeff_j;
		float stdcoeff_c = coeff_k;
		float stdcoeff_d = -(coeff_i*point1.x + coeff_j*point1.y + coeff_k*point1.z);


		for(int j = 0; j < cloud->points.size(); j++) 
		{
			if(testInliers.count(j)>0)
				continue;

			pcl::PointXYZI testPoint = cloud->points[j];
			float tempDist = fabs(testPoint.x * stdcoeff_a + testPoint.y * stdcoeff_b + testPoint.z * stdcoeff_c + stdcoeff_d)/
					sqrt(stdcoeff_a*stdcoeff_a+stdcoeff_b*stdcoeff_b+stdcoeff_c*stdcoeff_c);
			
			if(tempDist <= distanceTol)
				testInliers.insert(j);
		
		}

		if(testInliers.size() > inliersResult.size())
			inliersResult = testInliers;
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac routine took " << elapsedTime.count() << " milliseconds." << std::endl;

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}
*/
int main ()
{
	std::cout << "starting ransac test" << std::endl;
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
