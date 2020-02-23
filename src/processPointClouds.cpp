// PCL lib Functions for processing point clouds

#include <unordered_set>
#include "processPointClouds.h"


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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);

    return {obstCloud, planeCloud};
}

template<typename PointT>
std::vector<double> ProcessPointClouds<PointT>::GetPlaneCoefficients(const typename pcl::PointCloud<PointT>::Ptr cloud, const std::unordered_set<int> &indexList, double &normalMagnitude) {
	std::vector<double> coefficients(4);

	// need at least 3 poip1 nts
	if (indexList.size() < 3) {
		normalMagnitude = 0.0;
		return coefficients;
	}

    // get points
    auto indexIt = indexList.begin();
    const PointT &p1 = cloud->points[*(indexIt++)];
    const PointT &p2 = cloud->points[*(indexIt++)];
    const PointT &p3 = cloud->points[*indexIt];

	// basis vectors
	const PointT v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
	const PointT v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

	// cross product
	coefficients[0] = v1.y * v2.z - v2.y * v1.z;
	coefficients[1] = v2.x * v1.z - v1.x - v2.z;
	coefficients[2] = v1.x * v2.y - v2.x * v1.y;
	coefficients[3] = - (coefficients[0] * p1.x + coefficients[1] * p1.y + coefficients[2] * p1.z);

	// magnitude of vector normal to plane
	normalMagnitude = std::sqrt(coefficients[0]* coefficients[0] +
                                coefficients[1]* coefficients[1] +
                                coefficients[2]* coefficients[2]);

	return coefficients;
}

template<typename PointT>
std::unordered_set<int>  ProcessPointClouds<PointT>::GetGroundPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::srand(std::time(NULL));

	// all available test points
	std::deque<int> allTestPoints(cloud->points.size());
	std::iota(allTestPoints.begin(), allTestPoints.end(), 0);

	// TODO: Change to allow more test points to generate plane and use least squares solver for plane coefficients
	const int numPlanePoints = 3;

	for (int i = 0; i < maxIterations; ++i) {
		// initialize remainingPoints to all available points
		std::deque<int> remainingPoints = allTestPoints;
		// new empty inliers
		std::unordered_set<int> inliersTest;

		// get points to generate plane
		while (inliersTest.size() < numPlanePoints && !remainingPoints.empty()) {
			const int pointIndex = std::rand() % remainingPoints.size();
			// move to set of inliers
			inliersTest.insert(remainingPoints[pointIndex]);
			remainingPoints.erase(remainingPoints.begin() + pointIndex);
		}

		// get plane coefficients
		double normalMagnitude = 0.0;
		const std::vector<double> coefficients = GetPlaneCoefficients(cloud, inliersTest, normalMagnitude);
		// if points are colinear, move on
		if (normalMagnitude < FLT_EPSILON) continue;

		// test for inliers
		for (int k = 0; k < remainingPoints.size(); ++k) {
			const int pointIndex = remainingPoints[k];
			// test distance
			const double distance = std::fabs(coefficients[0] * cloud->points[pointIndex].x +
			                                  coefficients[1] * cloud->points[pointIndex].y +
											  coefficients[2] * cloud->points[pointIndex].z +
											  coefficients[3]) / normalMagnitude;
			if (distance < distanceTol) {
				// add to set of inliers
				inliersTest.insert(pointIndex);
			}
		}
		// test if best fit
		if (inliersTest.size() > inliersResult.size()) {
			inliersResult = inliersTest;
		}

	}

	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // get ground plane
    std::unordered_set<int> inliers = GetGroundPlane(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Custom plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    if (inliers.empty())
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // extract cloud inliers and outliers from unordered list
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

    return {cloudOutliers, cloudInliers};

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePcl(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "PCL plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

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

// instantiate expected types for ProcessPointClouds
// https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class ProcessPointClouds<pcl::PointXYZ>;
