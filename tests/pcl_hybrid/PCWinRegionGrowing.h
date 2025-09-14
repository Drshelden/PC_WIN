#pragma once

#include <vector>
#include <pcl/segmentation/region_growing.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// PlaneAwareRegionGrowing: region growing that only groups points sharing
// the same dominant plane label and whose normals are angularly consistent.
class PlaneAwareRegionGrowing : public pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> {
public:
    PlaneAwareRegionGrowing(const std::vector<int>& labels,
                            pcl::PointCloud<pcl::Normal>::ConstPtr norms);

protected:
    bool validatePoint(pcl::index_t initial_seed, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const override;

private:
    const std::vector<int>& normal_labels_;
    pcl::PointCloud<pcl::Normal>::ConstPtr normals_;
};

// CylinderAwareRegionGrowing: region growing that only groups points sharing
// the same dominant plane label and whose normals are angularly consistent.
class CylinderAwareRegionGrowing : public pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> {
public:
    CylinderAwareRegionGrowing(const std::vector<int>& labels,
                            pcl::PointCloud<pcl::Normal>::ConstPtr norms);

protected:
    bool validatePoint(pcl::index_t initial_seed, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const override;

private:
    const std::vector<int>& cylinder_labels_;
    pcl::PointCloud<pcl::Normal>::ConstPtr normals_;
};
