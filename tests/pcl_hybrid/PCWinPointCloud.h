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
    // Import points from an in-memory contiguous float32 buffer (x,y,z,x,y,z,...)
    // 'n_points' is the number of 3D points in the buffer.
    int importPointsFromBuffer(const float* xyz, size_t n_points);
};
