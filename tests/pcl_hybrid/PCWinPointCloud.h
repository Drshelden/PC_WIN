#pragma once

#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

class PCWin_PointCloud {
public:
    PCWin_PointCloud();

    // Public instance variables (as requested)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    std::vector<int> planeLabels;

    // Import points from a file; returns the same integer status codes as load_pointcloud_file
    int importPoints(const std::string& filename);
};
