#pragma once

#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PCWin_PointCloud; // forward

class ShapeFinder {
public:
    using PointT = pcl::PointXYZ;

    // Each cluster is a point cloud (plane clusters first, then euclidean residual clusters)
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    // For plane-origin clusters, store plane label (0..2). For non-plane clusters, value is -1
    std::vector<int> clusterPlaneLabels;
    
    // High-level shape objects (CylinderShape/PlaneShape)
    std::vector<std::shared_ptr<class Shape>> shapeCollection;

    // Find shapes/clusters from the provided point cloud; returns 0 (OK) or negative on error
    int findShapes(const PCWin_PointCloud& pc);
};
