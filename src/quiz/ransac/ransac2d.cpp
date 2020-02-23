/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>

#include "render/render.h"
#include "processPointClouds.h"

#ifndef SFND_SENSOR_DATA_ROOT
#define SFND_SENSOR_DATA_ROOT "../../../sensors/data"
#endif

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
	return pointProcessor.loadPcd(std::string(SFND_SENSOR_DATA_ROOT) + "/pcd/simpleHighway.pcd");
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

static std::vector<double> GetPlaneCoefficients(const std::vector<pcl::PointXYZ> &points, double &normalMagnitude) {
	std::vector<double> result(4);

	// need at least 3 points
	if (points.size() < 3) {
		normalMagnitude = 0.0;
		return result;
	}

	// basis vectors
	const Vect3 v1(points[1].x - points[0].x, points[1].y - points[0].y, points[1].z - points[0].z);
	const Vect3 v2(points[2].x - points[0].x, points[2].y - points[0].y, points[2].z - points[0].z);

	// cross product
	result[0] = v1.y * v2.z - v2.y * v1.z;
	result[1] = v2.x * v1.z - v1.x - v2.z;
	result[2] = v1.x * v2.y - v2.x * v1.y;
	result[3] = - (result[0] * points[0].x + result[1] * points[0].y + result[2] * points[0].z);

	// magnitude of vector normal to plane
	normalMagnitude = std::sqrt(result[0]* result[0] + result[1]* result[1] + result[2]* result[2]);

	return result;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

		// TODO: instead of making new vector of points, pass cloud and index list to GetPlaneCoefficients
		// gather points to create plane
		std::vector<pcl::PointXYZ> planePoints;
		for (const auto pointIndex : inliersTest) {
			planePoints.push_back(cloud->points[pointIndex]);
		}

		// get plane coefficients
		double normalMagnitude = 0.0;
		const std::vector<double> coefficients = GetPlaneCoefficients(planePoints, normalMagnitude);
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::srand(std::time(NULL));

	// initialize list of available lines (end point indices of cloud)
	std::deque<std::pair<int, int>> remainingLines;
	for (int j = 0; j < cloud->points.size(); ++j) {
		for (int k = j+1; k < cloud->points.size(); ++k) {
			remainingLines.push_back(std::pair<int, int>(j, k));
		}
	}

	for (int i = 0; i < maxIterations && !remainingLines.empty(); ++i) {
		// get random line from cloud
		const int lineIndex = std::rand() % remainingLines.size();
		std::pair<int, int> linePoints = remainingLines[lineIndex];
		// remove line from list
		remainingLines.erase(remainingLines.begin() + lineIndex);

		// create line
		const double coeff_a = cloud->points[linePoints.first].y - cloud->points[linePoints.second].y;
		const double coeff_b = cloud->points[linePoints.second].x - cloud->points[linePoints.first].x;
		const double coeff_c = cloud->points[linePoints.first].x * cloud->points[linePoints.second].y - cloud->points[linePoints.second].x * cloud->points[linePoints.first].y;

		// test for inliers
		std::unordered_set<int> inliersTest;
		for (int k = 0; k < cloud->points.size(); ++k) {
			// test distance
			const double distance = std::fabs(coeff_a * cloud->points[k].x + coeff_b * cloud->points[k].y + coeff_c) / sqrt(coeff_a * coeff_a + coeff_b * coeff_b);
			if (distance < distanceTol) {
				inliersTest.insert(k);
			}
		}
		// test if best fit
		if (inliersTest.size() > inliersResult.size()) {
			inliersResult = inliersTest;
		}

	}

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.25);

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
