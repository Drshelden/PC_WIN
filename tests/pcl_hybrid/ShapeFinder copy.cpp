#include "ShapeFinder.h"
#include "PCWinPointCloud.h"
#include "PlaneAwareRegionGrowing.h"
#include "shapes.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <unordered_set>

int ShapeFinder::findShapes(const PCWin_PointCloud& pc) {
    clusters.clear();
    clusterPlaneLabels.clear();
    shapeCollection.clear();

    using PointT = pcl::PointXYZ;

    // Stage 0: Coarse segmentation (create parent shapes)
    // Use a larger tolerance to split the cloud into coarse clusters
    {
        pcl::search::KdTree<PointT>::Ptr ctree(new pcl::search::KdTree<PointT>());
        ctree->setInputCloud(pc.cloud);
        pcl::EuclideanClusterExtraction<PointT> cec;
        cec.setClusterTolerance(0.1); // coarse tolerance (meters)
        cec.setMinClusterSize(50);
        cec.setMaxClusterSize(10000000);
        cec.setSearchMethod(ctree);
        cec.setInputCloud(pc.cloud);
        std::vector<pcl::PointIndices> coarse_clusters;
        cec.extract(coarse_clusters);

        // For each coarse cluster create a parent shape (as a CylinderShape placeholder)
        for (auto &cl : coarse_clusters) {
            if (cl.indices.empty()) continue;
            pcl::PointCloud<PointT>::Ptr parent_cloud(new pcl::PointCloud<PointT>());
            for (int idx : cl.indices) parent_cloud->push_back(pc.cloud->at(idx));
            // create parent shape and register it
            std::shared_ptr<Shape> parent = std::make_shared<CylinderShape>(parent_cloud);
            shapeCollection.push_back(parent);
            // also keep in clusters list for backwards compatibility
            clusters.push_back(parent_cloud);
            clusterPlaneLabels.push_back(-1);
        }
    }

    // Stage 1: Plane-aware clustering (run per-parent on coarse clusters)
    // We'll iterate over the parent shapes we created earlier and run the detailed
    // plane-aware + euclidean clustering pipeline on each parent cluster to produce children.
    for (auto &parent : shapeCollection) {
        // parent points
        pcl::PointCloud<PointT>::Ptr parent_cloud = parent->getPoints();
        if (!parent_cloud || parent_cloud->empty()) continue;

        // Extract corresponding normals from the global pc.normals for points that match by coordinate.
        // Simpler: compute normals locally on the parent_cloud
        pcl::PointCloud<pcl::Normal>::Ptr parent_normals(new pcl::PointCloud<pcl::Normal>());
        {
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            pcl::search::KdTree<PointT>::Ptr ntree(new pcl::search::KdTree<PointT>());
            ne.setSearchMethod(ntree);
            ne.setKSearch(50);
            ne.setInputCloud(parent_cloud);
            ne.compute(*parent_normals);
        }

        // Compute plane labels locally
        std::vector<int> parent_plane_labels(parent_cloud->size());
        for (std::size_t i = 0; i < parent_normals->size(); ++i) {
            auto& n = parent_normals->at(i);
            float ax = std::abs(n.normal_x);
            float ay = std::abs(n.normal_y);
            float az = std::abs(n.normal_z);
            if (az <= ax && az <= ay)
                parent_plane_labels[i] = 0;
            else if (ax <= ay && ax <= az)
                parent_plane_labels[i] = 1;
            else
                parent_plane_labels[i] = 2;
        }

        // Run plane-aware region growing on the parent cloud
        PlaneAwareRegionGrowing preg(parent_plane_labels, parent_normals);
        preg.setMinClusterSize(25);
        preg.setMaxClusterSize(1000000);
        pcl::search::KdTree<PointT>::Ptr ptree(new pcl::search::KdTree<PointT>());
        preg.setSearchMethod(ptree);
        preg.setNumberOfNeighbours(50);
        preg.setInputCloud(parent_cloud);
        preg.setInputNormals(parent_normals);

        std::vector<pcl::PointIndices> p_plane_clusters;
        preg.extract(p_plane_clusters);

        // For each local plane cluster, create a child PlaneShape and attach to parent
        std::unordered_set<int> used_local_indices;
        for (auto &cl : p_plane_clusters) {
            if (cl.indices.empty()) continue;
            pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>());
            for (int idx : cl.indices) {
                child_cloud->push_back(parent_cloud->at(idx));
                used_local_indices.insert(idx);
            }
            int label = parent_plane_labels[cl.indices.front()];
            std::shared_ptr<Shape> child = std::make_shared<PlaneShape>(child_cloud, label);
            parent->addChild(child);
            clusters.push_back(child_cloud);
            clusterPlaneLabels.push_back(label);
        }

        // Residual clustering on leftover local points
        pcl::PointCloud<PointT>::Ptr residual_local(new pcl::PointCloud<PointT>());
        std::vector<int> residual_local_map;
        for (std::size_t i = 0; i < parent_cloud->size(); ++i) {
            if (used_local_indices.find(static_cast<int>(i)) == used_local_indices.end()) {
                residual_local->push_back(parent_cloud->at(i));
                residual_local_map.push_back(static_cast<int>(i));
            }
        }

        if (!residual_local->empty()) {
            pcl::search::KdTree<PointT>::Ptr rtree(new pcl::search::KdTree<PointT>());
            rtree->setInputCloud(residual_local);
            pcl::EuclideanClusterExtraction<PointT> rec;
            rec.setClusterTolerance(0.02);
            rec.setMinClusterSize(25);
            rec.setMaxClusterSize(1000000);
            rec.setSearchMethod(rtree);
            rec.setInputCloud(residual_local);
            std::vector<pcl::PointIndices> local_euc_clusters;
            rec.extract(local_euc_clusters);

            for (auto &cl : local_euc_clusters) {
                if (cl.indices.empty()) continue;
                pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>());
                for (int ridx : cl.indices) {
                    int original_idx = residual_local_map[ridx];
                    child_cloud->push_back(parent_cloud->at(original_idx));
                }
                std::shared_ptr<Shape> child = std::make_shared<CylinderShape>(child_cloud);
                parent->addChild(child);
                clusters.push_back(child_cloud);
                clusterPlaneLabels.push_back(-1);
            }
        }
    }

    return 0;
}
