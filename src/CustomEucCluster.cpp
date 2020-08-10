#include "CustomEucCluster.h"


void Proximity(std::vector<bool>& alreadyProcessed, typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int index, std::vector<int>* cluster,KdTree3D* tree, float distanceTol)
{
	alreadyProcessed[index] = true;
	cluster->push_back(index);
	//std::cout << "index saved: " << index << endl;

	pcl::PointXYZI tempPoint = cloud->points[index];
	std::vector<float> tempPoint2 {tempPoint.x, tempPoint.y, tempPoint.z};
	std::vector<int> nearby = tree->search(tempPoint2,distanceTol);
	for(int indiv_nearby : nearby)
	{
		//std::cout << "Neighbor index: " << indiv_nearby << endl;
		
		//if(!checkProcessHelper_v2(cluster,indiv_nearby)) //point has not been processed
		if(!alreadyProcessed[indiv_nearby])
		{
			//std::cout << "neighboring point found but not processed" << endl;
			Proximity(alreadyProcessed, cloud, indiv_nearby, cluster, tree, distanceTol);
		}
		
	}


}

std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree3D* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;

	std::vector<bool> alreadyProcessed(cloud->points.size(), false);

	for(int i = 0; i < cloud->points.size(); i++)
	{
		//std::cout << "x: " << points[i][0] << endl;
		//check if point has been processed
		
		//if(!checkProcessHelper(&clusters,i)) //point has not been processed
		if(!alreadyProcessed[i])
		{
			//std::cout << "Didn't find initial index and going to create sub cluster now" << endl;
			// create cluster
			std::vector<int> sub_cluster;
			Proximity(alreadyProcessed, cloud, i, &sub_cluster, tree, distanceTol);
			clusters.push_back(sub_cluster);
		}
	
	}	
 
	return clusters;

}