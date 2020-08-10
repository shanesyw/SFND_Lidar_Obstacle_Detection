#include "kdtree3d.h"

void Proximity(std::vector<bool>& alreadyProcessed, std::vector<std::vector<float>>& points,int index, std::vector<int>* cluster,KdTree3D* tree, float distanceTol);


std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree3D* tree, float distanceTol);

