/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int p1_idx, p2_idx;
	float dist;
	while(maxIterations--)
	{	
		std::unordered_set<int> inliers;
		p1_idx = rand() % cloud->points.size();
		p2_idx = p1_idx;
		while(p1_idx == p2_idx)
			p2_idx = rand() % cloud->points.size();

		float a = cloud->points[p1_idx].y - cloud->points[p2_idx].y;
		float b = cloud->points[p2_idx].x - cloud->points[p1_idx].x;
		float c = (cloud->points[p1_idx].x * cloud->points[p2_idx].y) - (cloud->points[p2_idx].x * cloud->points[p1_idx].y);
		
		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(i == p1_idx || i == p2_idx)	continue;
			dist = fabs(a*cloud->points[i].x + b*cloud->points[i].y + c) / sqrt(a*a + b*b);
			if(dist <= distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
			inliersResult.insert(p1_idx);
			inliersResult.insert(p2_idx);
		}
		
	}
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int p1_idx, p2_idx, p3_idx;
	float dist;
	while(maxIterations--)
	{	
		std::unordered_set<int> inliers;
		p1_idx = rand() % cloud->points.size();
		p2_idx = p1_idx;
		p3_idx = p1_idx;
		while(p1_idx == p2_idx)
			p2_idx = rand() % cloud->points.size();
		while((p1_idx == p3_idx) || (p2_idx == p3_idx))
		{
			p3_idx = rand() % cloud->points.size();
		}

		float a = ((cloud->points[p2_idx].y - cloud->points[p1_idx].y) * (cloud->points[p3_idx].z - cloud->points[p1_idx].z)) - ((cloud->points[p2_idx].z - cloud->points[p1_idx].z) * (cloud->points[p3_idx].y - cloud->points[p1_idx].y));
		float b = ((cloud->points[p2_idx].z - cloud->points[p1_idx].z) * (cloud->points[p3_idx].x - cloud->points[p1_idx].x)) - ((cloud->points[p2_idx].x - cloud->points[p1_idx].x) * (cloud->points[p3_idx].z - cloud->points[p1_idx].z));
		float c = ((cloud->points[p2_idx].x - cloud->points[p1_idx].x) * (cloud->points[p3_idx].y - cloud->points[p1_idx].y)) - ((cloud->points[p2_idx].y - cloud->points[p1_idx].y) * (cloud->points[p3_idx].x - cloud->points[p1_idx].x));
		float d = -(a * cloud->points[p1_idx].x + b * cloud->points[p1_idx].y + c * cloud->points[p1_idx].z);
		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(i == p1_idx || i == p2_idx || i == p3_idx)	continue;
			dist = fabs(a * cloud->points[i].x + b * cloud->points[i].y + c * cloud->points[i].z + d) / sqrt(a * a + b * b + c * c);
			if(dist <= distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
			inliersResult.insert(p1_idx);
			inliersResult.insert(p2_idx);
			inliersResult.insert(p3_idx);
		}
		
	}
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 30, 1);
	std::unordered_set<int> inliers = RansacPlane(cloud, 30, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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

