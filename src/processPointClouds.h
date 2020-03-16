// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    /**
     * @brief   Extract plane from point cloud with list of point indices
     *
     * Only the first 3 point indices will be used to form the plane
     *
     * @param     points          Point cloud
     * @param     indexList       List of point indices to include in plane fit
     * @param     normalMagnitude Magnitude of plane normal vector
     * @return    std::vector<double> Plane coefficients
     */
    std::vector<double> GetPlaneCoefficients(const typename pcl::PointCloud<PointT>::Ptr cloud, const std::unordered_set<int> &indexList, double &normalMagnitude);

    /**
     * @brief   Single plane segmentation RANSAC implementation
     *
     * Three points chosen at random are used to generate a plane and all remaining point distances to plane are tested.
     *
     * @param     cloud         Point cloud
     * @param     maxIterations Number of planes to test
     * @param     distanceTol   Distance to include test points as inliers
     * @return    std::unordered_set<int> Best set of inlier point indices
     */
    std::unordered_set<int> GetGroundPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlanePcl(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    // std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringPcl(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    void Proximity(const float distanceTol, const typename pcl::PointCloud<PointT>::Ptr cloud, const int targetId, KdTree<PointT> &tree, std::unordered_set<int> &processedPoints, std::vector<int> &cluster);

    std::vector<std::vector<int>> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT> &tree, float distanceTol, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};
#endif /* PROCESSPOINTCLOUDS_H_ */
