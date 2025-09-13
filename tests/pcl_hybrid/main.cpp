#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

// -------------------------------------------------------------------
// Custom region growing class with plane-aware predicate
// -------------------------------------------------------------------
class PlaneAwareRegionGrowing : public pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> {
public:
    PlaneAwareRegionGrowing(const std::vector<int>& labels,
                            pcl::PointCloud<pcl::Normal>::ConstPtr norms)
        : plane_labels_(labels), normals_(norms) {}

protected:
    // RegionGrowing in PCL uses validatePoint(initial_seed, point, nghbr, is_a_seed)
    // Implement that signature and use it to enforce plane-label equality and normal consistency.
    bool validatePoint(pcl::index_t /*initial_seed*/, pcl::index_t point, pcl::index_t nghbr, bool& is_a_seed) const override {
        // Default (match base behaviour): assume this can be a seed
        is_a_seed = true;

        // Require same dominant plane label
        if (plane_labels_[static_cast<std::size_t>(point)] != plane_labels_[static_cast<std::size_t>(nghbr)])
            return false;

        // Check angular consistency of normals between the current point and the neighbour
        Eigen::Vector3f n1(normals_->at(static_cast<std::size_t>(point)).normal_x,
                           normals_->at(static_cast<std::size_t>(point)).normal_y,
                           normals_->at(static_cast<std::size_t>(point)).normal_z);
        Eigen::Vector3f n2(normals_->at(static_cast<std::size_t>(nghbr)).normal_x,
                           normals_->at(static_cast<std::size_t>(nghbr)).normal_y,
                           normals_->at(static_cast<std::size_t>(nghbr)).normal_z);

        float dot = std::max(-1.0f, std::min(1.0f, n1.dot(n2)));
        float angle = std::acos(dot);

        // Use a more generous angular tolerance (20 degrees) to allow slightly curved cylinder surfaces
        if (!(angle < 20.0f / 180.0f * static_cast<float>(M_PI)))
            return false;

        return true;
    }

private:
    const std::vector<int>& plane_labels_;
    pcl::PointCloud<pcl::Normal>::ConstPtr normals_;
};

// -------------------------------------------------------------------
// Main
// -------------------------------------------------------------------
int main(int argc, char** argv)
{
    using PointT = pcl::PointXYZ;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // ----------------------
    // Load point cloud
    // ----------------------
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " input.pcd" << std::endl;
        return -1;
    }
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points\n";

    // ----------------------
    // Estimate normals
    // ----------------------
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    // Use a larger neighborhood for more stable normals on cylindrical surfaces
    ne.setKSearch(50);
    ne.compute(*normals);

    // ----------------------
    // Classify dominant plane (smallest absolute component rule)
    // ----------------------
    std::vector<int> plane_labels(cloud->size());
    for (std::size_t i = 0; i < normals->size(); ++i) {
        auto& n = normals->at(i);
        float ax = std::abs(n.normal_x);
        float ay = std::abs(n.normal_y);
        float az = std::abs(n.normal_z);

        if (az <= ax && az <= ay)
            plane_labels[i] = 0; // XY
        else if (ax <= ay && ax <= az)
            plane_labels[i] = 1; // YZ
        else
            plane_labels[i] = 2; // ZX
    }

    // ----------------------
    // Stage 1: Plane-aware clustering
    // ----------------------
    PlaneAwareRegionGrowing reg(plane_labels, normals);
    reg.setMinClusterSize(25);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    // Increase number of neighbours used for region growing to be more generous
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);

    std::vector<pcl::PointIndices> plane_clusters;
    reg.extract(plane_clusters);

    std::cout << "Stage 1: found " << plane_clusters.size() << " plane-aware clusters\n";

    // Mark used points
    std::unordered_set<int> used_points;
    std::vector<int> cluster_plane_labels;
    for (auto& cl : plane_clusters) {
        if (cl.indices.empty()) continue;
        int label = plane_labels[cl.indices.front()];
        cluster_plane_labels.push_back(label);
        for (int idx : cl.indices)
            used_points.insert(idx);
    }

    // ----------------------
    // Stage 2: Euclidean clustering on residual points
    // ----------------------
    pcl::PointCloud<PointT>::Ptr residual_cloud(new pcl::PointCloud<PointT>);
    std::vector<int> residual_indices_map;

    for (std::size_t i = 0; i < cloud->size(); ++i) {
        if (used_points.find(static_cast<int>(i)) == used_points.end()) {
            residual_cloud->push_back(cloud->at(i));
            residual_indices_map.push_back(static_cast<int>(i));
        }
    }
    std::cout << "Residual cloud size: " << residual_cloud->size() << " points\n";

    std::vector<pcl::PointIndices> euclidean_clusters;
    if (!residual_cloud->empty()) {
        pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>());
        tree2->setInputCloud(residual_cloud);

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.02); // adjust distance threshold
        ec.setMinClusterSize(25);
        ec.setMaxClusterSize(1000000);
        ec.setSearchMethod(tree2);
        ec.setInputCloud(residual_cloud);
        ec.extract(euclidean_clusters);
    }
    std::cout << "Stage 2: found " << euclidean_clusters.size() << " Euclidean clusters\n";

    // ----------------------
    // Visualization
    // ----------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 255);
    // bright shades for plane-based colors (128..255)
    std::uniform_int_distribution<int> bright(128, 255);

    int cluster_id = 0;

    // Plane-based clusters (fixed colors by plane)
    for (size_t i = 0; i < plane_clusters.size(); ++i) {
        pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
        for (int idx : plane_clusters[i].indices)
            cluster_cloud->push_back(cloud->at(idx));

        int label = cluster_plane_labels[i];
    int r=0, g=0, b=0;
    if (label == 0) { r = bright(rng); g = 0;   b = 0;   } // XY = red shade
    if (label == 1) { r = 0;   g = bright(rng); b = 0;   } // YZ = green shade
    if (label == 2) { r = 0;   g = 0;   b = bright(rng); } // ZX = blue shade

        pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cluster_cloud, r, g, b);
        viewer->addPointCloud<PointT>(cluster_cloud, color_handler,
            "plane_cluster_" + std::to_string(cluster_id++));
    }

    // Residual clusters (random colors)
    for (auto& cl : euclidean_clusters) {
        pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
        for (int ridx : cl.indices) {
            int original_idx = residual_indices_map[ridx];
            cluster_cloud->push_back(cloud->at(original_idx));
        }
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(
            cluster_cloud, dist(rng), dist(rng), dist(rng));
        viewer->addPointCloud<PointT>(cluster_cloud, color_handler,
            "euclidean_cluster_" + std::to_string(cluster_id++));
    }

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
