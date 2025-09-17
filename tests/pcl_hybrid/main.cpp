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
#if VISUALIZE
#include <pcl/visualization/pcl_visualizer.h>
#endif
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
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#if VISUALIZE
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
        std::cerr << "Usage: " << argv[0] << " input.(pcd|xyz|e57) [export.json]" << std::endl;
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

    // If an export filename is supplied as the second argument, write the root JSON
    if (argc > 2) {
        std::string export_file = argv[2];
        if (sf.rootShape) {
            try {
                json root = json::parse(sf.rootShape->toJSON());
                std::ofstream ofs(export_file);
                if (!ofs) {
                    std::cerr << "Failed to open export file: " << export_file << std::endl;
                } else {
                    ofs << root.dump(2);
                    ofs.close();
                    std::cout << "Wrote root JSON to " << export_file << std::endl;
                }
            } catch (const std::exception &e) {
                // fallback: write raw string
                std::ofstream ofs(export_file);
                if (ofs) ofs << sf.rootShape->toJSON();
                std::cerr << "Warning: JSON parse/dump failed: " << e.what() << ", wrote raw string." << std::endl;
            }
        } else {
            std::cerr << "No root shape available to export." << std::endl;
        }
    }

    // ----------------------
    // Visualization
    // ----------------------
    #if VISUALIZE
        visualize(sf);
    #endif

    return 0;
}


#if VISUALIZE
void visualize(const ShapeFinder &sf) {
    using PointT = pcl::PointXYZ;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(255, 255);
    // bright shades for plane-based colors (255..255)
    std::uniform_int_distribution<int> bright(255, 255);
    

    int cluster_id = 0;

    // Build JSON from the root shape and render from that structure
    if (!sf.rootShape) return;
    json root = json::parse(sf.rootShape->toJSON());

    std::function<void(const json&, int)> renderFromJson;
    renderFromJson = [&](const json &node, int depth) {
        if (node.is_null()) return;
        // draw points
        if (node.contains("points") && node["points"].is_array() && !node["points"].empty()) {
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            for (const auto &pt : node["points"]) cloud->push_back(PointT(pt[0].get<float>(), pt[1].get<float>(), pt[2].get<float>()));
            int r = dist(rng), g = dist(rng), b = dist(rng);
            std::string type = node.value("type", std::string("generic"));
            if (type == "generic") r = g = b = 128;
            else if (type == "plane") {
                int plane_label = node.value("plane_label", -1);
                if (plane_label == 0) { r = bright(rng); g = bright(rng); b = 0; }
                else if (plane_label == 1) { r = 0; g = bright(rng); b = bright(rng); }
                else if (plane_label == 2) { r = bright(rng); g = 0; b = bright(rng); }
            } else if (type == "cylinder") {
                int cyl_label = node.value("cylinder_label", -1);
                if (cyl_label == 0) { r = bright(rng); g = 0; b = 0; }
                if (cyl_label == 1) { r = 0; g = bright(rng); b = 0; }
                if (cyl_label == 2) { r = 0; g = 0; b = bright(rng); }
            }
            pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, r, g, b);
            int this_cluster_id = cluster_id++;
            std::string id = "shape_cluster_" + std::to_string(this_cluster_id);
            viewer->addPointCloud<PointT>(cloud, color_handler, id);

            // draw critical points segments depending on type
            if (node.contains("critical_points") && node["critical_points"].is_array()) {
                auto &cp = node["critical_points"];
                if (node.value("type", std::string("")) == "plane" && cp.size() >= 2) {
                    for (size_t k = 0; k < cp.size(); ++k) {
                        PointT p1(cp[k][0].get<float>(), cp[k][1].get<float>(), cp[k][2].get<float>());
                        PointT p2(cp[(k+1) % cp.size()][0].get<float>(), cp[(k+1) % cp.size()][1].get<float>(), cp[(k+1) % cp.size()][2].get<float>());
                        std::string lid = "hull_line_" + std::to_string(this_cluster_id) + "_" + std::to_string(k);
                        viewer->addLine<PointT>(p1, p2, r, g, b, lid);
                    }
                } else if (node.value("type", std::string("")) == "cylinder" && cp.size() >= 2) {
                    PointT p1(cp[0][0].get<float>(), cp[0][1].get<float>(), cp[0][2].get<float>());
                    PointT p2(cp[1][0].get<float>(), cp[1][1].get<float>(), cp[1][2].get<float>());
                    std::string lid = "cyl_spine_" + std::to_string(this_cluster_id);
                    viewer->addLine<PointT>(p1, p2, r, g, b, lid);
                }
            }
        }

        // recurse into children
        if (node.contains("children") && node["children"].is_array()) {
            for (const auto &c : node["children"]) renderFromJson(c, depth+1);
        }
    };

    renderFromJson(root, 0);

    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
#endif
