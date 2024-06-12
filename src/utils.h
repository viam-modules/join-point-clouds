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

std::string pointToString(const pcl::PointXYZ& point);
std::vector<unsigned char> pclCloudToPCDBytes(const pcl::PointCloud<pcl::PointXYZ>& pc);
RawPCD parseRawPCD(const std::vector<unsigned char>& input);
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(const RawPCD& rawPCD);
std::ostream& operator<<(std::ostream& os, const RawPCD& pcd);
