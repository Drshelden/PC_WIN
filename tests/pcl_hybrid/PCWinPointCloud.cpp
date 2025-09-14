#include "PCWinPointCloud.h"
#include "utils.h" // for load_pointcloud_file and status codes
#include <cmath>

PCWin_PointCloud::PCWin_PointCloud()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>), normals(new pcl::PointCloud<pcl::Normal>) {}

int PCWin_PointCloud::importPoints(const std::string& filename) {
    int status = load_pointcloud_file(filename.c_str(), cloud, normals);
    if (status != LOAD_OK)
        return status;

    // Classify dominant plane (smallest absolute component rule)
    planeLabels.resize(cloud->size());
    for (std::size_t i = 0; i < normals->size(); ++i) {
        auto& n = normals->at(i);
        float ax = std::abs(n.normal_x);
        float ay = std::abs(n.normal_y);
        float az = std::abs(n.normal_z);

        if (az <= ax && az <= ay)
            planeLabels[i] = 0; // XY
        else if (ax <= ay && ax <= az)
            planeLabels[i] = 1; // YZ
        else
            planeLabels[i] = 2; // ZX
    }

    return LOAD_OK;
}
