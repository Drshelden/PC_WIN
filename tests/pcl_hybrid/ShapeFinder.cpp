#include "ShapeFinder.h"
#include "PCWinPointCloud.h"
#include "PCWinRegionGrowing.h"
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
    // Temporarily disabled — wrap the coarse segmentation in a preprocessor guard
#if 0
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

        // For each coarse cluster create a parent shape (as a OtherShape placeholder)
        for (auto &cl : coarse_clusters) {
            if (cl.indices.empty()) continue;
            pcl::PointCloud<PointT>::Ptr parent_cloud(new pcl::PointCloud<PointT>());
            for (int idx : cl.indices) parent_cloud->push_back(pc.cloud->at(idx));
            // create parent shape and register it
            std::shared_ptr<Shape> parent = std::make_shared<OtherShape>(parent_cloud);
            shapeCollection.push_back(parent);
            // also keep in clusters list for backwards compatibility
            clusters.push_back(parent_cloud);
            clusterPlaneLabels.push_back(-1);
        }
    }
#endif

    // If Stage 0 was disabled, ensure we have at least one parent shape that
    // contains the whole cloud so the rest of the pipeline runs on something.
    if (shapeCollection.empty()) {
        // use GenericShape as a flexible root that can collect residual points
        std::shared_ptr<GenericShape> root = std::make_shared<GenericShape>(pc.cloud);
        shapeCollection.push_back(root);
        // keep backwards-compatible clusters/labels lists updated
        clusters.push_back(pc.cloud);
        clusterPlaneLabels.push_back(-1);
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
        CylinderAwareRegionGrowing preg(parent_plane_labels, parent_normals);
        preg.setMinClusterSize(25);
        preg.setMaxClusterSize(1000000);
        pcl::search::KdTree<PointT>::Ptr ptree(new pcl::search::KdTree<PointT>());
        preg.setSearchMethod(ptree);
        preg.setNumberOfNeighbours(50);
        preg.setInputCloud(parent_cloud);
        preg.setInputNormals(parent_normals);

        std::vector<pcl::PointIndices> p_plane_clusters;
        preg.extract(p_plane_clusters);

        // For each local plane cluster, create a child CylinderShape and attach to parent
        std::unordered_set<int> used_local_indices;
        for (auto &cl : p_plane_clusters) {
            if (cl.indices.empty()) continue;
            pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>());
            for (int idx : cl.indices) {
                child_cloud->push_back(parent_cloud->at(idx));
                used_local_indices.insert(idx);
            }
            int label = parent_plane_labels[cl.indices.front()];
            std::shared_ptr<Shape> child = std::make_shared<CylinderShape>(child_cloud, label);
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
            // Instead of creating separate child clusters for residuals, append
            // them to the root GenericShape so they are preserved and visible.
            // Find the root (the first shape in shapeCollection is our root when
            // Stage 0 is disabled or no coarse clusters created).
            if (!shapeCollection.empty()) {
                // we expect the root to be a GenericShape; attempt dynamic_cast
                auto root_generic = std::dynamic_pointer_cast<GenericShape>(shapeCollection.front());
                if (root_generic) {
                    // construct a temporary cloud of residuals in original parent coordinates
                    pcl::PointCloud<PointT>::Ptr mapped_residual(new pcl::PointCloud<PointT>());
                    for (int i = 0; i < static_cast<int>(residual_local->size()); ++i) {
                        mapped_residual->push_back(residual_local->at(i));
                    }
                    root_generic->appendPoints(mapped_residual);
                } else {
                    // fallback: if root isn't GenericShape, push residuals as clusters
                    for (std::size_t i = 0; i < residual_local->size(); ++i) {
                        pcl::PointCloud<PointT>::Ptr single(new pcl::PointCloud<PointT>());
                        single->push_back(residual_local->at(i));
                        parent->addChild(std::make_shared<OtherShape>(single));
                        clusters.push_back(single);
                        clusterPlaneLabels.push_back(-1);
                    }
                }
            }
        }
    }

    return 0;
}
