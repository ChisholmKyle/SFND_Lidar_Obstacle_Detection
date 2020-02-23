/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>

#include "render/render.h"
#include "processPointClouds.h"

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
