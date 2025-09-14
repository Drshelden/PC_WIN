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
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <functional>

#include "utils.h"
#include "PCWinPointCloud.h"
#include "ShapeFinder.h"
#include "shapes.h"
#include "PCWinRegionGrowing.h"

#define VISUALIZE
#ifdef VISUALIZE
void visualize(const ShapeFinder &sf);    
#endif

// class CylinderAwareRegionGrowing moved to PCWinRegionGrowing.h / .cpp

// -------------------------------------------------------------------
// Main
// -------------------------------------------------------------------



int main(int argc, char** argv)
{
    //using PointT = pcl::PointXYZ;
    // Create a PCWin_PointCloud instance and import points
    PCWin_PointCloud pc;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " input.(pcd|xyz|e57)" << std::endl;
        return -1;
    }

    int load_status = pc.importPoints(argv[1]);
    switch (load_status) {
        case LOAD_OK:
            std::cout << "Loaded " << pc.cloud->size() << " points from " << argv[1] << "\n";
            break;
        case LOAD_NOT_FOUND:
            std::cerr << "File not found: " << argv[1] << std::endl;
            return -1;
        case LOAD_UNSUPPORTED_FORMAT:
            std::cerr << "Unsupported file format: " << argv[1] << std::endl;
            return -2;
        case LOAD_PARSE_ERROR:
        default:
            std::cerr << "Failed to parse or load file: " << argv[1] << std::endl;
            return -3;
    }

    // ----------------------
    // Find shapes/clusters using ShapeFinder
    // ----------------------
    ShapeFinder sf;
    sf.findShapes(pc);
    std::cout << "Found " << sf.clusters.size() << " clusters in total\n";

    // ----------------------
    // Visualization
    // ----------------------
    #ifdef VISUALIZE
        visualize(sf);
    #endif

    return 0;
}


#ifdef VISUALIZE
void visualize(const ShapeFinder &sf) {
    using PointT = pcl::PointXYZ;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(255, 255);
    // bright shades for plane-based colors (255..255)
    std::uniform_int_distribution<int> bright(255, 255);
    

    int cluster_id = 0;

    // recursive renderer: draws the given shape and then its children
    std::function<void(const std::shared_ptr<Shape>&, int)> renderShape;
    renderShape = [&](const std::shared_ptr<Shape>& shape, int depth) {
        if (!shape) return;
        pcl::PointCloud<PointT>::Ptr cluster_cloud = shape->getPoints();
        if (cluster_cloud && !cluster_cloud->empty()) {
            int r = dist(rng), g = dist(rng), b = dist(rng);
            // Color rules:
            // - generic (residual/root) clusters: neutral grey
            // - plane clusters: colored based on dominant normal axis
            if (shape->getType() == "generic") {
                r = g = b = 128;
            } 
            
            if (shape->getType() == "plane") {
                // attempt to read plane_label via PlaneShape
                int plane_label = -1;
                if (auto ps = std::dynamic_pointer_cast<PlaneShape>(shape)) plane_label = ps->getPlaneLabel();
                if (plane_label == 0) { r = bright(rng); g = bright(rng); b = 0; } // X-dominant
                else if (plane_label == 1) { r = 0; g = bright(rng); b = bright(rng); } // Y-dominant
                else if (plane_label == 2) { r = bright(rng); g = 0; b = bright(rng); } // Z-dominant
            }
            
            if (shape->getType() == "cylinder") {
                int cylinder_label = -1;
                if (auto cs = std::dynamic_pointer_cast<CylinderShape>(shape)) cylinder_label = cs->getCylinderLabel();
                if (cylinder_label == 0) { r = bright(rng); g = 0; b = 0; }
                if (cylinder_label == 1) { r = 0; g = bright(rng); b = 0; }
                if (cylinder_label == 2) { r = 0; g = 0; b = bright(rng); }
            }

            // // slightly dim colors for deeper levels (optional visual cue)
            // float dim = 1.0f - std::min(0.5f, depth * 0.12f);
            // r = static_cast<int>(r * dim);
            // g = static_cast<int>(g * dim);
            // b = static_cast<int>(b * dim);

            pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cluster_cloud, r, g, b);
            int this_cluster_id = cluster_id++;
            std::string id = "shape_cluster_" + std::to_string(this_cluster_id);
            viewer->addPointCloud<PointT>(cluster_cloud, color_handler, id);

            // If this is a plane, draw connected line segments between sequential
            // critical points (convex hull) and close the loop. Use same color.
            if (shape->getType() == "plane") {
                if (auto ps = std::dynamic_pointer_cast<PlaneShape>(shape)) {
                    auto cp = ps->getCriticalPoints();
                    if (cp && cp->size() >= 2) {
                        for (size_t k = 0; k < cp->size(); ++k) {
                            const PointT &p1 = (*cp)[k];
                            const PointT &p2 = (*cp)[(k + 1) % cp->size()];
                            std::string lid = "plane_bound_" + std::to_string(this_cluster_id) + "_" + std::to_string(k);
                            viewer->addLine<PointT>(p1, p2, r, g, b, lid);
                        }
                    }
                }
            }
            // If cylinder, draw a single spine segment between its two critical points
            if (shape->getType() == "cylinder") {
                if (auto cs = std::dynamic_pointer_cast<CylinderShape>(shape)) {
                    auto cp = cs->getCriticalPoints();
                    if (cp && cp->size() >= 2) {
                        const PointT &p1 = (*cp)[0];
                        const PointT &p2 = (*cp)[1];
                        std::string lid = "cyl_spine_" + std::to_string(this_cluster_id);
                        viewer->addLine<PointT>(p1, p2, r, g, b, lid);
                    }
                }
            }
        }

        // recurse into children
        for (auto &child : shape->getChildren()) {
            renderShape(child, depth + 1);
        }
    };

    for (size_t i = 0; i < sf.shapeCollection.size(); ++i) {
        renderShape(sf.shapeCollection[i], 0);
    }

    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
#endif
