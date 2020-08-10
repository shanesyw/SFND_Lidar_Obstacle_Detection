// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/voxel_grid.h>
#include <unordered_set>
#include "CustomEucCluster.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>());

    std::cout << "filterRes: " << filterRes << std::endl;

    // Create the voxel filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    //Use Cropbox
    pcl::CropBox<PointT> cropFilter(true);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.setInputCloud(cloud_filtered);
    cropFilter.filter(*cloud_cropped);

    std::vector<int> roof_indices;
    pcl::CropBox<PointT> cropFilter2(true);
    cropFilter.setMin(Eigen::Vector4f(-1.5,-1.7, -1,1));
    cropFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4,1));
    cropFilter.setInputCloud(cloud_cropped);
    cropFilter.filter(roof_indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: roof_indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_cropped);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped; //cloud

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_road (new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_road);
    std::cerr << "PointCloud representing the planar component: " << cloud_road->width * cloud_road->height << " data points." << std::endl;

    extract.setNegative(true);
    extract.filter(*cloud_obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_road, cloud_obstacles);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // custom implementation for RANSAC
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{
		std::unordered_set<int> testInliers;

		while(testInliers.size() < 3)
			testInliers.insert(rand()%(cloud->points.size()));

		auto itr = testInliers.begin();

		pcl::PointXYZI point1 = cloud->points[*itr];
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

        // variable that will be repeatedly used
        float denominator = sqrt(stdcoeff_a*stdcoeff_a+stdcoeff_b*stdcoeff_b+stdcoeff_c*stdcoeff_c);

		for(int j = 0; j < cloud->points.size(); j++) 
		{
			if(testInliers.count(j)>0)
				continue;

			pcl::PointXYZI testPoint = cloud->points[j];
			float tempDist = fabs(testPoint.x * stdcoeff_a + testPoint.y * stdcoeff_b + testPoint.z * stdcoeff_c + stdcoeff_d)/denominator;
			
			if(tempDist <= distanceThreshold)
				testInliers.insert(j);
		
		}
        

		if(testInliers.size() > inliersResult.size())
			inliersResult = testInliers;
	}

    // separate road and obstacles
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


    // Finish timing of segmentation process.
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // used for custom implemtation
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    //Custom implementation
    // make KD tree
    KdTree3D* tree = new KdTree3D;
    for (int i=0; i<cloud->points.size(); i++) 
    {
        pcl::PointXYZI tempPoint = cloud->points[i];
        std::vector<float> tempPoint2 {tempPoint.x, tempPoint.y, tempPoint.z};
        tree->insert(tempPoint2,i); 

    }

    //Euclidean cluster estimation
    std::vector<std::vector<int>> list_of_indices = euclideanCluster(cloud, tree, clusterTolerance);	

    //based on list of indices fill in cluster
    for(std::vector<int> getIndices: list_of_indices)
    {
        int clusterSize = getIndices.size();
        if(clusterSize >= minSize && clusterSize <= maxSize) // only do work when cluster size meets requirement
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());

            for(int index: getIndices)
                cloud_cluster->push_back(cloud->points[index]);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }

    }
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
