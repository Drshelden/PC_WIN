#ifndef PCL_HYBRID_UTILS_H
#define PCL_HYBRID_UTILS_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// Return codes for load_pointcloud_file
enum LoadStatus {
    LOAD_OK = 0,
    LOAD_NOT_FOUND = 1,
    LOAD_UNSUPPORTED_FORMAT = 2,
    LOAD_PARSE_ERROR = 3
};

// Loads a point cloud (PCD/XYZ/PTS/E57) into 'cloud' and optionally computes
// normals into 'normals' (pass nullptr to skip normals). Returns a LoadStatus.
int load_pointcloud_file(const std::string &filename,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr normals);

#endif // PCL_HYBRID_UTILS_H
