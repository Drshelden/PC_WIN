#include "ShapeFinder.h"
#include "PCWinPointCloud.h"
#include "PCWinRegionGrowing.h"
#include "shapes.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <unordered_set>
#include <unordered_map>

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

        // For each coarse cluster create a parent shape (as a PlaneShape placeholder)
        for (auto &cl : coarse_clusters) {
            if (cl.indices.empty()) continue;
            pcl::PointCloud<PointT>::Ptr parent_cloud(new pcl::PointCloud<PointT>());
            for (int idx : cl.indices) parent_cloud->push_back(pc.cloud->at(idx));
            // create parent shape and register it
            std::shared_ptr<Shape> parent = std::make_shared<PlaneShape>(parent_cloud);
            shapeCollection.push_back(parent);
            // also keep in clusters list for backwards compatibility
            clusters.push_back(parent_cloud);
            // clusterPlaneLabels.push_back(-1);
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
        // clusterPlaneLabels.push_back(-1);
    }

    // Stage 1: Cylinder-aware clustering (run per-parent on coarse clusters)
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

        // Find the sin of a small angle
        float small_angle_deg = 3.0f;
        float small_angle_sin = std::sin(small_angle_deg * static_cast<float>(M_PI) / 180.0f);

        // Compute normal-dominant labels locally (norm: 0=X,1=Y,2=Z, -1 none)
        // These indicate which axis the normal is most aligned with.
        std::vector<int> parent_plane_labels(parent_cloud->size(), -1);
        for (std::size_t i = 0; i < parent_normals->size(); ++i) {
            auto& n = parent_normals->at(i);
            float ax = std::abs(n.normal_x);
            float ay = std::abs(n.normal_y);
            float az = std::abs(n.normal_z);
            // pick largest absolute component; if near-equal use -1
            if (az > ax && az > ay) parent_plane_labels[i] = 2;
            else if (ax > ay && ax > az) parent_plane_labels[i] = 0;
            else if (ay > ax && ay > az) parent_plane_labels[i] = 1;
            else parent_plane_labels[i] = -1;
        }

        // Compute plane-compatibility labels (0..2 or -1)
        std::vector<int> parent_cylinder_labels(parent_cloud->size(), -1);
        for (std::size_t i = 0; i < parent_normals->size(); ++i) {
            auto& n = parent_normals->at(i);
            float ax = std::abs(n.normal_x);
            float ay = std::abs(n.normal_y);
            float az = std::abs(n.normal_z);
            if (az <= ax && az <= ay) {
                if (az >= small_angle_sin) parent_cylinder_labels[i] = -1; else parent_cylinder_labels[i] = 2;
            } else if (ax <= ay && ax <= az) {
                if (ax >= small_angle_sin) parent_cylinder_labels[i] = -1; else parent_cylinder_labels[i] = 0;
            } else {
                if (ay >= small_angle_sin) parent_cylinder_labels[i] = -1; else parent_cylinder_labels[i] = 1;
            }
        }

        // Run plane-aware region growing on the parent cloud
        CylinderAwareRegionGrowing preg(parent_cylinder_labels, parent_normals);
        preg.setMinClusterSize(25);
        preg.setMaxClusterSize(1000000);
        pcl::search::KdTree<PointT>::Ptr ptree(new pcl::search::KdTree<PointT>());
        preg.setSearchMethod(ptree);
        preg.setNumberOfNeighbours(50);
        preg.setInputCloud(parent_cloud);
        preg.setInputNormals(parent_normals);

        std::vector<pcl::PointIndices> p_plane_clusters;
        preg.extract(p_plane_clusters);

        // For each local cylinder cluster, decide whether a plane-aware cluster fits more points
        std::unordered_set<int> used_local_indices;

        // Extract plane-aware clusters so we can compare overlaps
        PlaneAwareRegionGrowing preg_plane(parent_plane_labels, parent_normals);
        preg_plane.setMinClusterSize(25);
        preg_plane.setMaxClusterSize(1000000);
        pcl::search::KdTree<PointT>::Ptr ptree_plane(new pcl::search::KdTree<PointT>());
        preg_plane.setSearchMethod(ptree_plane);
        preg_plane.setNumberOfNeighbours(50);
        preg_plane.setInputCloud(parent_cloud);
        preg_plane.setInputNormals(parent_normals);
        std::vector<pcl::PointIndices> plane_clusters;
        preg_plane.extract(plane_clusters);
        std::vector<char> plane_used(plane_clusters.size(), 0);

        for (auto &cl : p_plane_clusters) {
            if (cl.indices.empty()) continue;

            // build quick lookup for cylinder cluster indices
            std::unordered_set<int> cylset;
            for (int idx : cl.indices) cylset.insert(idx);
            int cylinder_size = static_cast<int>(cl.indices.size());

            // find best matching plane cluster by intersection size
            int best_plane_idx = -1;
            int best_overlap = 0;
            for (std::size_t pid = 0; pid < plane_clusters.size(); ++pid) {
                if (plane_used[pid]) continue;
                int overlap = 0;
                for (int pidx : plane_clusters[pid].indices) if (cylset.find(pidx) != cylset.end()) ++overlap;
                if (overlap > best_overlap) { best_overlap = overlap; best_plane_idx = static_cast<int>(pid); }
            }
            
            int plane_size = (best_plane_idx != -1) ? static_cast<int>(plane_clusters[best_plane_idx].indices.size()) : 0;
            if (best_plane_idx != -1 && plane_size >= cylinder_size) {
                // plane-aware cluster covers more points -> create PlaneShape from that plane cluster
                pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>());
                for (int pidx : plane_clusters[best_plane_idx].indices) {
                    child_cloud->push_back(parent_cloud->at(pidx));
                    used_local_indices.insert(pidx);
                }
                // determine dominant plane_label and plane_label for this plane cluster
                std::unordered_map<int,int> freq;
                for (int pidx : plane_clusters[best_plane_idx].indices) {
                    int lab = parent_plane_labels[pidx];
                    if (lab != -1) freq[lab]++;
                }
                int dominant_plane = -1; int best_count = 0;
                for (auto &kv : freq) if (kv.second > best_count) { best_count = kv.second; dominant_plane = kv.first; }
                // dominant normal-axis label
                std::unordered_map<int,int> nfreq;
                for (int pidx : plane_clusters[best_plane_idx].indices) {
                    int nl = parent_plane_labels[pidx];
                    if (nl != -1) nfreq[nl]++;
                }
                int dominant_norm = -1; best_count = 0;
                for (auto &kv : nfreq) if (kv.second > best_count) { best_count = kv.second; dominant_norm = kv.first; }
                std::shared_ptr<Shape> child = std::make_shared<PlaneShape>(child_cloud, dominant_norm);
                parent->addChild(child);
                clusters.push_back(child_cloud);
                clusterPlaneLabels.push_back(dominant_plane);
                plane_used[best_plane_idx] = 1;
            } else {
                // keep cylinder cluster as-is -> create a CylinderShape
                pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>());
                for (int idx : cl.indices) {
                    child_cloud->push_back(parent_cloud->at(idx));
                    used_local_indices.insert(idx);
                }
                // determine dominant cylinder label within this cylinder cluster
                std::unordered_map<int,int> cfreq;
                for (int idx : cl.indices) {
                    int lab = parent_cylinder_labels[idx];
                    if (lab != -1) cfreq[lab]++;
                }
                int dominant_cylinder_label = -1; int cbest = 0;
                for (auto &kv : cfreq) if (kv.second > cbest) { cbest = kv.second; dominant_cylinder_label = kv.first; }
                std::shared_ptr<Shape> child = std::make_shared<CylinderShape>(child_cloud, dominant_cylinder_label);
                parent->addChild(child);
                clusters.push_back(child_cloud);
                clusterPlaneLabels.push_back(dominant_cylinder_label);
            }
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
                        parent->addChild(std::make_shared<PlaneShape>(single));
                        clusters.push_back(single);
                        // clusterPlaneLabels.push_back(-1);
                    }
                }
            }
        }
    }

    return 0;
}
