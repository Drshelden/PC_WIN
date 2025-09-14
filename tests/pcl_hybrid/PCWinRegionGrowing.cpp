#include "PCWinRegionGrowing.h"
#include <Eigen/Dense>
#include <cmath>

#define angle_tolerance_deg 2.0f

PlaneAwareRegionGrowing::PlaneAwareRegionGrowing(const std::vector<int>& labels,
                                                 pcl::PointCloud<pcl::Normal>::ConstPtr norms)
    : normal_labels_(labels), normals_(norms) {}

bool PlaneAwareRegionGrowing::validatePoint(pcl::index_t /*initial_seed*/, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const {
    // Default (match base behaviour): assume this can be a seed
    is_a_seed = true;

    // Require same dominant normal label
    if (normal_labels_[static_cast<std::size_t>(point)] != normal_labels_[static_cast<std::size_t>(nghbr)])
        return false;
    
    // check for any orinality of points
    if (normal_labels_[static_cast<std::size_t>(point)] == -1)
        return false; // no normal label, so skip normal check

    // Check angular consistency of normals between the current point and the neighbour
    Eigen::Vector3f n1(normals_->at(static_cast<std::size_t>(point)).normal_x,
                       normals_->at(static_cast<std::size_t>(point)).normal_y,
                       normals_->at(static_cast<std::size_t>(point)).normal_z);
    Eigen::Vector3f n2(normals_->at(static_cast<std::size_t>(nghbr)).normal_x,
                       normals_->at(static_cast<std::size_t>(nghbr)).normal_y,
                       normals_->at(static_cast<std::size_t>(nghbr)).normal_z);

    // Compare normals in a direction-agnostic way: use absolute dot so n and -n are treated equal
    float raw_dot = n1.dot(n2);
    float abs_dot = std::max(-1.0f, std::min(1.0f, std::fabs(raw_dot)));
    float cos_thresh = std::cos(angle_tolerance_deg / 180.0f * static_cast<float>(M_PI));
    if (abs_dot < cos_thresh)
        return false;

    return true;
}


CylinderAwareRegionGrowing::CylinderAwareRegionGrowing(const std::vector<int>& labels,
                                                 pcl::PointCloud<pcl::Normal>::ConstPtr norms)
    : cylinder_labels_(labels), normals_(norms) {}

bool CylinderAwareRegionGrowing::validatePoint(pcl::index_t /*initial_seed*/, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const {
    // Default (match base behaviour): assume this can be a seed
    is_a_seed = true;

    // Require same dominant plane label
    if (cylinder_labels_[static_cast<std::size_t>(point)] != cylinder_labels_[static_cast<std::size_t>(nghbr)])
        return false;
    
    // check for any orinality of points
    if (cylinder_labels_[static_cast<std::size_t>(point)] == -1)
        return false; // no plane label, so skip normal check 

    // // Check angular consistency of normals between the current point and the neighbour
    // Eigen::Vector3f n1(normals_->at(static_cast<std::size_t>(point)).normal_x,
    //                    normals_->at(static_cast<std::size_t>(point)).normal_y,
    //                    normals_->at(static_cast<std::size_t>(point)).normal_z);
    // Eigen::Vector3f n2(normals_->at(static_cast<std::size_t>(nghbr)).normal_x,
    //                    normals_->at(static_cast<std::size_t>(nghbr)).normal_y,
    //                    normals_->at(static_cast<std::size_t>(nghbr)).normal_z);

    // // Compare normals in a direction-agnostic way: use absolute dot so n and -n are treated equal
    // float raw_dot = n1.dot(n2);
    // float abs_dot = std::max(-1.0f, std::min(1.0f, std::fabs(raw_dot)));
    // float cos_thresh = std::cos(angle_tolerance_deg / 180.0f * static_cast<float>(M_PI));
    // if (abs_dot < cos_thresh)
    //     return false;

    return true;
}
