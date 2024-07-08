#pragma once

#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ViamOrientationVector {
public:
    Eigen::Vector3f vec;  // Vector representing the spherical axis of rotation
    float th;  // Theta. The rotation angle in radians

    ViamOrientationVector(float ox = 0, float oy = 0, float oz = 1, float theta = 0);

    Eigen::Quaternionf toQuaternion() const;
};

struct RawPCD {
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    std::vector<int> viewpoint;
    int width;
    int height;
    int points;
    std::string dataType;
    std::vector<unsigned char> rawData;
};

// Helper function to convert a single pcl::PointXYZ to binary
std::string pointToString(const pcl::PointXYZ& point);
// Converts PCL point cloud data to raw PCD payload in binary format
std::vector<unsigned char> pclCloudToPCDBytes(const pcl::PointCloud<pcl::PointXYZ>& pc);
// Helper that converts raw Viam PCD data into structured data
RawPCD parseRawPCD(const std::vector<unsigned char>& input);
// Converts RawPCD struct to pcl::PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(const RawPCD& rawPCD);
// Aligns source cloud to target cloud using PCL's ICP methods
pcl::PointCloud<pcl::PointXYZ>::Ptr alignPointCloudsUsingICP(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    int proximityThreshold);
// Concatenates a vector of PCL clouds into one cloud
pcl::PointCloud<pcl::PointXYZ> combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);
