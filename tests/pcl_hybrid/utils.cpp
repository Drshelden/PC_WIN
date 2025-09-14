#include "utils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <cstdio>



int load_pointcloud_file(const std::string &filename,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    if (!cloud)
        return LOAD_PARSE_ERROR;

    // determine extension (lowercase)
    std::string ext;
    auto pos = filename.rfind('.');
    if (pos != std::string::npos)
        ext = filename.substr(pos);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == ".pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
            return LOAD_PARSE_ERROR;
    }

    if (ext == ".xyz") {
        std::ifstream ifs(filename);
        if (!ifs.is_open())
            return LOAD_NOT_FOUND;

        std::string line;
        std::size_t line_no = 0;
        while (std::getline(ifs, line)) {
            ++line_no;
            // skip comments/empty lines
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream iss(line);
            pcl::PointXYZ p;
            if (!(iss >> p.x >> p.y >> p.z)) {
                // skip lines that don't have 3 floats (tolerant)
                continue;
            }
            cloud->push_back(p);
        }
        if (cloud->empty())
            return LOAD_PARSE_ERROR;
    }

    // Support .pts files (common scanner output). Variants:
    // - First non-empty line may be an integer point count
    // - Subsequent lines contain: x y z [r g b] [intensity] ...
    // We'll parse the first three numeric columns and ignore the rest.
    if (ext == ".pts") {
        std::ifstream ifs(filename);
        if (!ifs.is_open())
            return LOAD_NOT_FOUND;

        std::string line;
        std::size_t line_no = 0;
        bool first_line_read = false;
        while (std::getline(ifs, line)) {
            ++line_no;
            // trim leading spaces
            std::size_t i = 0;
            while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
            if (i == line.size()) continue; // empty
            if (line[i] == '#') continue; // comment

            std::istringstream iss(line);
            // If first data line, check if it's a pure integer count header
            if (!first_line_read) {
                first_line_read = true;
                long possible_count = 0;
                if (iss >> possible_count) {
                    // if there is nothing else on the line, treat as count header
                    std::string rest;
                    if (!(iss >> rest)) {
                        continue; // skip header line
                    }
                }
                // Reset stream to parse as coordinates
                iss.clear();
                iss.str(line);
            }

            pcl::PointXYZ p;
            if (!(iss >> p.x >> p.y >> p.z)) {
                // tolerate lines with insufficient numeric columns
                continue;
            }
            cloud->push_back(p);
        }

        if (cloud->empty())
            return LOAD_PARSE_ERROR;
    }

    if (ext == ".e57") {
        // Prefer using the PDAL command-line tool to convert E57 -> PCD if available.
        int has_pdal = std::system("which pdal > /dev/null 2>&1") == 0;
        if (!has_pdal)
            return LOAD_UNSUPPORTED_FORMAT;

        std::string tmp_pcd = filename + ".pdal_tmp.pcd";
        std::string cmd = "pdal translate \"" + filename + "\" \"" + tmp_pcd + "\" -f pcd 2>/dev/null";
        int rc = std::system(cmd.c_str());
        if (rc != 0)
            return LOAD_PARSE_ERROR;

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(tmp_pcd, *cloud) == -1) {
            std::remove(tmp_pcd.c_str());
            return LOAD_PARSE_ERROR;
        }

        std::remove(tmp_pcd.c_str());
    }

    // If we got here, the point cloud was loaded into 'cloud'. Now compute normals
    // if a valid normals pointer was provided.
    if (!normals)
        return LOAD_OK; // caller doesn't want normals

    try {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(50);
        ne.compute(*normals);
    } catch (...) {
        return LOAD_PARSE_ERROR;
    }

    return LOAD_OK;
}
