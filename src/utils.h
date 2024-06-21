#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

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
// Override stream operator for RawPCD better debug printing
std::ostream& operator<<(std::ostream& os, const RawPCD& pcd);
