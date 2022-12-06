// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"
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
    typename pcl::PointCloud<PointT>::Ptr filterCloud(new pcl::PointCloud<PointT>());
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filterCloud);

    typename pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filterCloud);
    region.filter(*regionCloud);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(regionCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (regionCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*regionCloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCld(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCld(new pcl::PointCloud<PointT>());

    for(int idx : inliers->indices)
        planeCld->points.push_back(cloud->points[idx]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCld);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCld, planeCld);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout <<"Could not estimate a planar model for the given dataset\n";
    }
    // TODO:: Fill in this function to find inliers for the cloud.

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        clusters.push_back(cloudCluster);
    }
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
static void clusterHelper(std::vector<bool>& is_processed, std::vector<int>& cluster, int idx, const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<int> nearby = tree->search(points[idx],distanceTol);
	for(int i = 0; i < nearby.size(); i++)
	{
		if (is_processed[nearby[i]] == false)
		{
			is_processed[nearby[i]] = true;
			cluster.push_back(nearby[i]);
			clusterHelper(is_processed, cluster, nearby[i], points, tree, distanceTol);
		}
	}
}
static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> is_processed(points.size(), false);
	for(int i = 0 ; i < points.size(); i++)
	{
		if (is_processed[i] == true)
			continue;
			
		std::vector<int> cluster;
		clusterHelper(is_processed, cluster, i, points, tree, distanceTol);
        if(cluster.size() > 0)
		clusters.push_back(cluster);
        std::cout << "[euclideanCluster]points in cluster[" << i << "] : " <<cluster.size() << std::endl;
	}
    std::cout << "[euclideanCluster]points.size() : " << points.size() << std::endl;    
	return clusters;

}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree;
    for(int i = 0 ; i < cloud->points.size(); i++)
    {
        points.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, cloud->points[i].intensity});
        tree->insert(points[i], i);
    }

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);
    int clusterId = 0;
    for(std::vector<int> cluster : clusters)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: cluster)
        {
            pcl::PointXYZI p;
            p.x = points[indice][0];
            p.y = points[indice][1];
            p.z = points[indice][2];
            p.intensity = points[indice][3];
            clusterCloud->points.push_back(p);
        }
        // std::cout << "points in cluster[" << clusterId << "] : " <<cluster.size() << std::endl;
  		++clusterId;
        if(clusterCloud->points.size() > minSize)
            clusterClouds.push_back(clusterCloud);
  	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusterClouds;
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
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    // TODO:: Fill in this function to find inliers for the cloud.

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
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