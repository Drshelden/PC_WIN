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

int PCWin_PointCloud::importPointsFromBuffer(const float* xyz, size_t n_points) {
    if (!xyz || n_points == 0) return LOAD_PARSE_ERROR;
    cloud->clear();
    cloud->reserve(n_points);

    for (size_t i = 0; i < n_points; ++i) {
        pcl::PointXYZ p;
        p.x = xyz[i*3 + 0];
        p.y = xyz[i*3 + 1];
        p.z = xyz[i*3 + 2];
        cloud->push_back(p);
    }


    // Estimate normals using PCL's NormalEstimation with a KDTree
    try {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setKSearch(16); // reasonable default neighborhood

        pcl::PointCloud<pcl::Normal>::Ptr computed_normals(new pcl::PointCloud<pcl::Normal>());
        ne.compute(*computed_normals);

        // If computed_normals size doesn't match cloud, fallback to zeros
        if (computed_normals->size() == cloud->size()) {
            normals = computed_normals;
        } else {
            normals->clear();
            normals->resize(cloud->size());
            for (size_t i = 0; i < cloud->size(); ++i) {
                normals->at(i).normal_x = 0.0f;
                normals->at(i).normal_y = 0.0f;
                normals->at(i).normal_z = 1.0f;
            }
        }
    } catch (...) {
        // On any failure, fill with upward normals to keep downstream code robust
        normals->clear();
        normals->resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            normals->at(i).normal_x = 0.0f;
            normals->at(i).normal_y = 0.0f;
            normals->at(i).normal_z = 1.0f;
        }
    }

    // Populate planeLabels using the same rule as file import (smallest absolute component)
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
