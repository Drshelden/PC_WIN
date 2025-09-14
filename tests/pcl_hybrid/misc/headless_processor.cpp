#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <set>
#include <nlohmann/json.hpp>

#ifdef _WIN32
    #include <windows.h>
    #include <io.h>
    #define M_PI 3.14159265358979323846
#else
    #include <unistd.h>
#endif

// Disable visualization for headless mode
//#define PCL_VIEWER     // Disabled for headless mode
//#define USE_PCL        // Comment this line to disable PCL shape detection
#define USE_CGAL       // Comment this line to disable CGAL shape detection
#define FIT_PLANES     // Comment this line to disable plane detection
#define FIT_CYLINDERS  // Comment this line to disable cylinder detection

// PCL includes
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>

// CGAL includes
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Euclidean_distance.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_2.h>

// Add piping sequence header
#include "RansacPipingSequence.h"

// Type definitions
// Global settings json
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using json = nlohmann::json;

// CGAL types
using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Polyhedron_3 = CGAL::Polyhedron_3<Kernel>;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

// CGAL Point with normal type
using Point_with_normal = CGAL::Point_with_normal_3<Kernel>;
using Pwn_vector = std::vector<Point_with_normal>;

// CGAL Efficient RANSAC types
using RANSAC_Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using RANSAC_Point_with_normal = std::pair<RANSAC_Kernel::Point_3, RANSAC_Kernel::Vector_3>;
using RANSAC_Pwn_vector = std::vector<RANSAC_Point_with_normal>;
using Point_map = CGAL::First_of_pair_property_map<RANSAC_Point_with_normal>;
using Normal_map = CGAL::Second_of_pair_property_map<RANSAC_Point_with_normal>;

using RANSAC_Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<RANSAC_Kernel, RANSAC_Pwn_vector, Point_map, Normal_map>;
using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<RANSAC_Traits>;
using Plane = CGAL::Shape_detection::Plane<RANSAC_Traits>;
using Cylinder = CGAL::Shape_detection::Cylinder<RANSAC_Traits>;
using Cone = CGAL::Shape_detection::Cone<RANSAC_Traits>;
using Torus = CGAL::Shape_detection::Torus<RANSAC_Traits>;
// Alias for our template PipingSequence instantiated with project traits
using PipingSeq = ::PipingSequence<RANSAC_Traits>;

// CGAL clustering types
using Search_traits = CGAL::Search_traits_3<RANSAC_Kernel>;
using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<Search_traits>;
using Tree = CGAL::Kd_tree<Search_traits>;
using Distance = CGAL::Euclidean_distance<Search_traits>;

json g_settings;


// Type for CGAL cluster indices (similar to PCL's PointIndices)
struct CGALClusterIndices {
    std::vector<std::size_t> indices;
};

// Abstract Shape class
class Shape {
public:
    virtual ~Shape() = default;
    virtual std::string getType() const = 0;
    virtual json toJSON() const = 0;
    virtual size_t getPointCount() const = 0;
    virtual const std::vector<std::size_t>& getAssignedIndices() const = 0;
    virtual void setCriticalPoints() = 0;
    
protected:
    size_t cluster_id_;
    size_t global_id_;
    std::vector<std::size_t> assigned_indices_;
    std::vector<std::array<double, 3>> points_;
    std::vector<std::array<double, 3>> critical_points_;
    
public:
    Shape(size_t cluster_id, size_t global_id, const std::vector<std::size_t>& indices, 
          const RANSAC_Pwn_vector& remaining_points) 
        : cluster_id_(cluster_id), global_id_(global_id), assigned_indices_(indices) {
        // Extract point coordinates
        for (std::size_t idx : assigned_indices_) {
            if (idx < remaining_points.size()) {
                const auto& point = remaining_points[idx].first;
                points_.push_back({point.x(), point.y(), point.z()});
            }
        }
        // Critical points will be calculated by derived classes when needed
    }
};

// Plane Shape implementation
class PlaneShape : public Shape {
private:
    struct PlaneCoefficients {
        double nx, ny, nz, d;  // Normal vector (nx, ny, nz) and distance d
    } coefficients_;
    
public:
    PlaneShape(size_t cluster_id, size_t global_id, const std::vector<std::size_t>& indices,
               const RANSAC_Pwn_vector& remaining_points, const std::array<double, 4>& coefficients)
        : Shape(cluster_id, global_id, indices, remaining_points) {
        coefficients_.nx = coefficients[0];
        coefficients_.ny = coefficients[1];
        coefficients_.nz = coefficients[2];
        coefficients_.d = coefficients[3];
    }
    
    std::string getType() const override { return "plane"; }
    
    size_t getPointCount() const override { return assigned_indices_.size(); }
    
    const std::vector<std::size_t>& getAssignedIndices() const override { return assigned_indices_; }
    
    void setCriticalPoints() override {

        if (points_.size() < 3) return;
        
        // Project points onto the plane to get 2D coordinates
        RANSAC_Kernel::Vector_3 normal(coefficients_.nx, coefficients_.ny, coefficients_.nz);
        normal = normal / std::sqrt(normal.squared_length());
        
        // Find two orthogonal vectors in the plane
        RANSAC_Kernel::Vector_3 u, v;
        if (std::abs(normal.x()) < 0.9) {
            u = CGAL::cross_product(normal, RANSAC_Kernel::Vector_3(1, 0, 0));
        } else {
            u = CGAL::cross_product(normal, RANSAC_Kernel::Vector_3(0, 1, 0));
        }
        u = u / std::sqrt(u.squared_length());
        v = CGAL::cross_product(normal, u);
        v = v / std::sqrt(v.squared_length());
        
        // Project points to 2D plane coordinates
        std::vector<CGAL::Point_2<RANSAC_Kernel>> points_2d;
        for (const auto& pt : points_) {
            RANSAC_Kernel::Point_3 point(pt[0], pt[1], pt[2]);
            // Project to plane coordinate system
            double x_coord = (point - CGAL::ORIGIN) * u;
            double y_coord = (point - CGAL::ORIGIN) * v;
            points_2d.push_back(CGAL::Point_2<RANSAC_Kernel>(x_coord, y_coord));
        }
        
        // Compute 2D convex hull
        std::vector<CGAL::Point_2<RANSAC_Kernel>> hull_2d;
        CGAL::convex_hull_2(points_2d.begin(), points_2d.end(), std::back_inserter(hull_2d));
        
        // Convert back to 3D coordinates
        RANSAC_Kernel::Point_3 plane_origin = CGAL::ORIGIN + normal * (-coefficients_.d);
        for (const auto& hull_pt : hull_2d) {
            RANSAC_Kernel::Point_3 pt_3d = plane_origin + u * hull_pt.x() + v * hull_pt.y();
            critical_points_.push_back({pt_3d.x(), pt_3d.y(), pt_3d.z()});
        }
    }
    
    json toJSON() const override {
        json shape_json;
        shape_json["cluster_id"] = cluster_id_;
        shape_json["point_count"] = getPointCount();
        shape_json["type"] = getType();
        shape_json["global_id"] = global_id_;
        shape_json["coefficients"] = {
            {"nx", coefficients_.nx},
            {"ny", coefficients_.ny},
            {"nz", coefficients_.nz},
            {"d", coefficients_.d}
        };
        
        json points_array = json::array();
        for (const auto& point : points_) {
            points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["points"] = points_array;
        
        json critical_points_array = json::array();
        for (const auto& point : critical_points_) {
            critical_points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["critical_points"] = critical_points_array;
        
        return shape_json;
    }
};

// Cylinder Shape implementation
class CylinderShape : public Shape {
private:
    struct CylinderCoefficients {
        double px, py, pz;      // Point on axis
        double dx, dy, dz;      // Direction vector
        double radius;          // Cylinder radius
    } coefficients_;
    
public:
    CylinderShape(size_t cluster_id, size_t global_id, const std::vector<std::size_t>& indices,
                  const RANSAC_Pwn_vector& remaining_points, const std::array<double, 7>& coefficients)
        : Shape(cluster_id, global_id, indices, remaining_points) {
        coefficients_.px = coefficients[0];
        coefficients_.py = coefficients[1];
        coefficients_.pz = coefficients[2];
        coefficients_.dx = coefficients[3];
        coefficients_.dy = coefficients[4];
        coefficients_.dz = coefficients[5];
        coefficients_.radius = coefficients[6];
    }
    
    std::string getType() const override { return "cylinder"; }
    
    size_t getPointCount() const override { return assigned_indices_.size(); }
    
    const std::vector<std::size_t>& getAssignedIndices() const override { return assigned_indices_; }
    
    void setCriticalPoints() override {
        critical_points_.clear();
        if (points_.empty()) return;
        
        // Cylinder axis
        RANSAC_Kernel::Point_3 axis_point(coefficients_.px, coefficients_.py, coefficients_.pz);
        RANSAC_Kernel::Vector_3 axis_dir(coefficients_.dx, coefficients_.dy, coefficients_.dz);
        axis_dir = axis_dir / std::sqrt(axis_dir.squared_length()); // Normalize
        
        // Project all points onto the cylinder axis
        double min_projection = std::numeric_limits<double>::max();
        double max_projection = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : points_) {
            RANSAC_Kernel::Point_3 point(pt[0], pt[1], pt[2]);
            RANSAC_Kernel::Vector_3 to_point = point - axis_point;
            double projection = to_point * axis_dir;
            
            min_projection = std::min(min_projection, projection);
            max_projection = std::max(max_projection, projection);
        }
        
        // Create the two critical points P1 and P2 at the projection limits
        RANSAC_Kernel::Point_3 p1 = axis_point + axis_dir * min_projection;
        RANSAC_Kernel::Point_3 p2 = axis_point + axis_dir * max_projection;
        
        critical_points_.push_back({p1.x(), p1.y(), p1.z()});
        critical_points_.push_back({p2.x(), p2.y(), p2.z()});
    }
    
    json toJSON() const override {
        json shape_json;
        shape_json["cluster_id"] = cluster_id_;
        shape_json["point_count"] = getPointCount();
        shape_json["type"] = getType();
        shape_json["global_id"] = global_id_;
        shape_json["coefficients"] = {
            {"px", coefficients_.px},
            {"py", coefficients_.py},
            {"pz", coefficients_.pz},
            {"dx", coefficients_.dx},
            {"dy", coefficients_.dy},
            {"dz", coefficients_.dz},
            {"radius", coefficients_.radius}
        };
        
        json points_array = json::array();
        for (const auto& point : points_) {
            points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["points"] = points_array;
        
        json critical_points_array = json::array();
        for (const auto& point : critical_points_) {
            critical_points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["critical_points"] = critical_points_array;
        
        return shape_json;
    }
};

// Cone Shape implementation
class ConeShape : public Shape {
private:
    struct ConeCoefficients {
        double apex_x, apex_y, apex_z;  // Cone apex point
        double dx, dy, dz;              // Axis direction vector
        double angle;                   // Half-angle of the cone
    } coefficients_;
    
public:
    ConeShape(size_t cluster_id, size_t global_id, const std::vector<std::size_t>& indices,
              const RANSAC_Pwn_vector& remaining_points, const std::array<double, 7>& coefficients)
        : Shape(cluster_id, global_id, indices, remaining_points) {
        coefficients_.apex_x = coefficients[0];
        coefficients_.apex_y = coefficients[1];
        coefficients_.apex_z = coefficients[2];
        coefficients_.dx = coefficients[3];
        coefficients_.dy = coefficients[4];
        coefficients_.dz = coefficients[5];
        coefficients_.angle = coefficients[6];
    }
    
    std::string getType() const override { return "cone"; }
    
    size_t getPointCount() const override { return assigned_indices_.size(); }
    
    const std::vector<std::size_t>& getAssignedIndices() const override { return assigned_indices_; }
    
    void setCriticalPoints() override {
        critical_points_.clear();
        if (points_.empty()) return;
        
        // Cone axis
        RANSAC_Kernel::Point_3 apex(coefficients_.apex_x, coefficients_.apex_y, coefficients_.apex_z);
        RANSAC_Kernel::Vector_3 axis_dir(coefficients_.dx, coefficients_.dy, coefficients_.dz);
        axis_dir = axis_dir / std::sqrt(axis_dir.squared_length()); // Normalize
        
        // Project all points onto the cone axis
        double min_projection = std::numeric_limits<double>::max();
        double max_projection = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : points_) {
            RANSAC_Kernel::Point_3 point(pt[0], pt[1], pt[2]);
            RANSAC_Kernel::Vector_3 to_point = point - apex;
            double projection = to_point * axis_dir;
            
            min_projection = std::min(min_projection, projection);
            max_projection = std::max(max_projection, projection);
        }
        
        // Create the two critical points P1 and P2 at the projection limits
        RANSAC_Kernel::Point_3 p1 = apex + axis_dir * min_projection;
        RANSAC_Kernel::Point_3 p2 = apex + axis_dir * max_projection;
        
        critical_points_.push_back({p1.x(), p1.y(), p1.z()});
        critical_points_.push_back({p2.x(), p2.y(), p2.z()});
    }
    
    json toJSON() const override {
        json shape_json;
        shape_json["cluster_id"] = cluster_id_;
        shape_json["point_count"] = getPointCount();
        shape_json["type"] = getType();
        shape_json["global_id"] = global_id_;
        shape_json["coefficients"] = {
            {"apex_x", coefficients_.apex_x},
            {"apex_y", coefficients_.apex_y},
            {"apex_z", coefficients_.apex_z},
            {"dx", coefficients_.dx},
            {"dy", coefficients_.dy},
            {"dz", coefficients_.dz},
            {"angle", coefficients_.angle}
        };
        
        json points_array = json::array();
        for (const auto& point : points_) {
            points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["points"] = points_array;
        
        json critical_points_array = json::array();
        for (const auto& point : critical_points_) {
            critical_points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["critical_points"] = critical_points_array;
        
        return shape_json;
    }
};

// Torus Shape implementation
class TorusShape : public Shape {
private:
    struct TorusCoefficients {
        double center_x, center_y, center_z;  // Torus center point
        double nx, ny, nz;                    // Normal vector (axis direction)
        double major_radius, minor_radius;    // Major and minor radii
    } coefficients_;
    
public:
    TorusShape(size_t cluster_id, size_t global_id, const std::vector<std::size_t>& indices,
               const RANSAC_Pwn_vector& remaining_points, const std::array<double, 8>& coefficients)
        : Shape(cluster_id, global_id, indices, remaining_points) {
        coefficients_.center_x = coefficients[0];
        coefficients_.center_y = coefficients[1];
        coefficients_.center_z = coefficients[2];
        coefficients_.nx = coefficients[3];
        coefficients_.ny = coefficients[4];
        coefficients_.nz = coefficients[5];
        coefficients_.major_radius = coefficients[6];
        coefficients_.minor_radius = coefficients[7];
    }
    
    std::string getType() const override { return "torus"; }
    
    size_t getPointCount() const override { return assigned_indices_.size(); }
    
    const std::vector<std::size_t>& getAssignedIndices() const override { return assigned_indices_; }
    
    void setCriticalPoints() override {
    // Torus critical point processing: project points onto spine, find amin, amax, P1
    critical_points_.clear();
    if (points_.empty()) {
        // Fallback: just add center if no points available
        critical_points_.push_back({coefficients_.center_x, coefficients_.center_y, coefficients_.center_z});
        return;
    }
    // Use Eigen for all math
    Eigen::Vector3d center(coefficients_.center_x, coefficients_.center_y, coefficients_.center_z);
    Eigen::Vector3d normal(coefficients_.nx, coefficients_.ny, coefficients_.nz);
    normal.normalize();
    double major_radius = coefficients_.major_radius;

    Eigen::Vector3d VCP1;
    Eigen::Vector3d CP0;
    Eigen::Vector3d CP1;
    Eigen::Vector3d CP2;
    Eigen::Vector3d pc_pt;
    Eigen::Vector3d v;
    bool CP1_set = false;
    double amax = -std::numeric_limits<double>::infinity();
    double amin = std::numeric_limits<double>::infinity();
    const double PI = 3.14159265358979323846;

    for (size_t idx = 0; idx < points_.size(); ++idx) {
        const auto& arr = points_[idx];
        Eigen::Vector3d pa_pt(arr[0], arr[1], arr[2]);
        Eigen::Vector3d v_c_a_norm = (pa_pt - center);
        v_c_a_norm.normalize();
        Eigen::Vector3d v_c_a_perp = (v_c_a_norm.cross(normal));
        v_c_a_perp.normalize();
        v = normal.cross(v_c_a_perp);
        v.normalize();
        pc_pt = center + v * major_radius;
        // Eigen::Vector3d pb_pt = pa_pt; // Replace with actual closest point logic
        // Eigen::Vector3d v = pb_pt - center;
        // if (v.norm() > 0.0) {
        //     v.normalize();
        //     Eigen::Vector3d pc_pt = center + v * major_radius;
        if (!CP1_set) {

            VCP1 = v;
            CP1 = pc_pt;
            CP0 = Eigen::Vector3d::Zero();
            CP2 = Eigen::Vector3d::Zero();
            CP1_set = true;
        } 
        else {
            Eigen::Vector3d vnext = v;
            double dot = VCP1.dot(vnext);
            Eigen::Vector3d cross = VCP1.cross(vnext);
            double va = std::acos(dot);
            double anext = va;
            if (cross.dot(normal) < 0) {
                anext = -anext;
            }
            if (anext > 0 && anext <= PI && anext > amax) {
                amax = anext;
                CP2 = pc_pt;
            }
            if (anext < 0 && anext >= -PI && anext < amin) {
                amin = anext;
                CP0 = pc_pt;
            }
        }
    
    }

    // Convert Eigen::Vector3d to std::array<double, 3> before pushing
    auto to_array = [](const Eigen::Vector3d& v) {
        return std::array<double, 3>{v.x(), v.y(), v.z()};
    };
    critical_points_.push_back(to_array(CP0));
    critical_points_.push_back(to_array(CP1));
    critical_points_.push_back(to_array(CP2));

    // // Debug print: output torus critical points
    // std::cout << "Torus critical points:" << std::endl;
    // std::cout << "  CP0: [" << CP0.x() << ", " << CP0.y() << ", " << CP0.z() << "]" << std::endl;
    // std::cout << "  CP1: [" << CP1.x() << ", " << CP1.y() << ", " << CP1.z() << "]" << std::endl;
    // std::cout << "  CP2: [" << CP2.x() << ", " << CP2.y() << ", " << CP2.z() << "]" << std::endl;

    }
    
    json toJSON() const override {
        json shape_json;
        shape_json["cluster_id"] = cluster_id_;
        shape_json["point_count"] = getPointCount();
        shape_json["type"] = getType();
        shape_json["global_id"] = global_id_;
        shape_json["coefficients"] = {
            {"center_x", coefficients_.center_x},
            {"center_y", coefficients_.center_y},
            {"center_z", coefficients_.center_z},
            {"nx", coefficients_.nx},
            {"ny", coefficients_.ny},
            {"nz", coefficients_.nz},
            {"major_radius", coefficients_.major_radius},
            {"minor_radius", coefficients_.minor_radius}
        };
        
        json points_array = json::array();
        for (const auto& point : points_) {
            points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["points"] = points_array;
        
        json critical_points_array = json::array();
        for (const auto& point : critical_points_) {
            critical_points_array.push_back({point[0], point[1], point[2]});
        }
        shape_json["critical_points"] = critical_points_array;
        
        return shape_json;
    }
};

// PipingSequence Shape wrapper used by our Shape/Factory system
class PipingSequenceShape : public Shape {
public:
    PipingSequenceShape(size_t cluster_id, size_t global_id, const std::vector<std::size_t>& indices,
                       const RANSAC_Pwn_vector& remaining_points, double radius, int segments)
        : Shape(cluster_id, global_id, indices, remaining_points), radius_(radius), segments_(segments) {}

    std::string getType() const override { return "piping_sequence"; }
    size_t getPointCount() const override { return assigned_indices_.size(); }
    const std::vector<std::size_t>& getAssignedIndices() const override { return assigned_indices_; }

    void setCriticalPoints() override {
        critical_points_.clear();
        if (points_.empty()) return;
        critical_points_.push_back(points_.front());
        critical_points_.push_back(points_[points_.size()/2]);
        critical_points_.push_back(points_.back());
    }

    json toJSON() const override {
        json shape_json;
        shape_json["cluster_id"] = cluster_id_;
        shape_json["point_count"] = getPointCount();
        shape_json["type"] = getType();
        shape_json["global_id"] = global_id_;
        shape_json["radius"] = radius_;
        shape_json["segments"] = segments_;

        json points_array = json::array();
        for (const auto& point : points_) points_array.push_back({point[0], point[1], point[2]});
        shape_json["points"] = points_array;

        json critical_points_array = json::array();
        for (const auto& point : critical_points_) critical_points_array.push_back({point[0], point[1], point[2]});
        shape_json["critical_points"] = critical_points_array;

        // Children (compound geometry): cylinders and toruses
        json children = json::array();
        for (const auto &c : children_) {
            json child_json;
            child_json["type"] = c.type;
            child_json["coefficients"] = c.coefficients;
            child_json["critical_points"] = json::array();
            for (const auto &cp : c.critical_points) child_json["critical_points"].push_back({cp[0], cp[1], cp[2]});
            child_json["points"] = json::array();
            for (const auto &pt : c.points) child_json["points"].push_back({pt[0], pt[1], pt[2]});
            children.push_back(child_json);
        }
        shape_json["children"] = children;

        return shape_json;
    }

    // Public child geometry type and helper to add children from factories
    struct ChildGeometry {
        std::string type; // "cylinder" or "torus"
        std::array<double, 8> coefficients; // use 7 for cylinder, 8 for torus
        std::vector<std::array<double,3>> critical_points;
        std::vector<std::array<double,3>> points;
        ChildGeometry() { coefficients = {0,0,0,0,0,0,0,0}; }
    };

    void addChild(const ChildGeometry &c) { children_.push_back(c); }

private:
    double radius_ = 0.0;
    int segments_ = 1;
    std::vector<ChildGeometry> children_;
};

// Abstract ShapeFactory class
class ShapeFactory {
public:
    virtual ~ShapeFactory() = default;
    virtual std::unique_ptr<Shape> detect(RANSAC_Pwn_vector& remaining_points,
                                         size_t cluster_id, size_t& global_id) = 0;
    virtual std::string getShapeType() const = 0;
};


// Factory for PipingSequence using CGAL's RANSAC integration
class PipingSequenceShapeFactory : public ShapeFactory {
private:
    Efficient_ransac::Parameters parameters_;
public:
    PipingSequenceShapeFactory() {
        json s = (g_settings.contains("piping_sequence") && g_settings["piping_sequence"].is_object()) ? g_settings["piping_sequence"] : json::object();
        parameters_.min_points = s.value("min_points", 100);
        parameters_.epsilon = s.value("epsilon", 1.0);
        parameters_.cluster_epsilon = s.value("cluster_epsilon", 2.0);
        parameters_.normal_threshold = s.value("normal_threshold", 0.6);
        parameters_.probability = s.value("probability", 0.01);
    }

    std::string getShapeType() const override { return "piping_sequence"; }

    std::unique_ptr<Shape> detect(RANSAC_Pwn_vector& remaining_points, size_t cluster_id, size_t& global_id) override {
        Efficient_ransac ransac;
        ransac.set_input(remaining_points);
        ransac.add_shape_factory<PipingSeq>();

        Efficient_ransac::Parameters ransac_parameters = parameters_;
        ransac.detect(ransac_parameters);

        size_t best_size = 0;
        auto best_shape = ransac.shapes().end();
        for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
            size_t shape_size = (*it)->indices_of_assigned_points().size();
            if (shape_size > best_size) { best_size = shape_size; best_shape = it; }
        }

        if (best_shape != ransac.shapes().end() && best_size >= parameters_.min_points) {
            auto raw = (*best_shape).get();
            PipingSeq* ps = dynamic_cast<PipingSeq*>(raw);
            double radius = 0.0;
            int segments = 1;
            if (ps) { radius = static_cast<double>(ps->radius()); segments = ps->segments(); }

            auto assigned_indices = (*best_shape)->indices_of_assigned_points();
            std::vector<std::size_t> indices_vec(assigned_indices.begin(), assigned_indices.end());

            // Build shape and populate children (simple PCA along major axis)
            auto shape_ptr = std::make_unique<PipingSequenceShape>(cluster_id, global_id++, indices_vec, remaining_points, radius, segments);

            // Gather 3D points for assigned indices
            std::vector<Eigen::Vector3d> pts;
            pts.reserve(indices_vec.size());
            for (auto idx : indices_vec) {
                if (idx < remaining_points.size()) {
                    const auto &p = remaining_points[idx].first;
                    pts.emplace_back(p.x(), p.y(), p.z());
                }
            }

            if (!pts.empty()) {
                // Compute centroid
                Eigen::Vector3d centroid(0,0,0);
                for (auto &v : pts) centroid += v;
                centroid /= double(pts.size());

                // Compute covariance and principal axis
                Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                for (auto &v : pts) {
                    Eigen::Vector3d d = v - centroid;
                    cov += d * d.transpose();
                }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                Eigen::Vector3d axis = es.eigenvectors().col(2).normalized(); // principal axis

                // Project points onto axis and sort by projection
                std::vector<std::pair<double,Eigen::Vector3d>> proj;
                proj.reserve(pts.size());
                for (auto &v : pts) proj.emplace_back((v-centroid).dot(axis), v);
                std::sort(proj.begin(), proj.end(), [](auto &a, auto &b){ return a.first < b.first; });

                // Split into 'segments' contiguous bins
                int segs = std::max(1, segments);
                size_t per_seg = std::max((size_t)1, proj.size() / segs);
                std::vector<Eigen::Vector3d> seg_centroids;
                std::vector<double> seg_radii;
                std::vector<std::vector<Eigen::Vector3d>> seg_points(segs);

                for (int s = 0; s < segs; ++s) {
                    size_t start = s * per_seg;
                    size_t end = (s == segs-1) ? proj.size() : std::min(proj.size(), start + per_seg);
                    if (start >= end) continue;
                    Eigen::Vector3d sc(0,0,0);
                    for (size_t k = start; k < end; ++k) { sc += proj[k].second; seg_points[s].push_back(proj[k].second); }
                    sc /= double(end - start);
                    seg_centroids.push_back(sc);

                    // estimate radius as mean radial distance to axis through sc
                    double mean_r = 0.0;
                    for (size_t k = start; k < end; ++k) {
                        Eigen::Vector3d d = proj[k].second - sc;
                        Eigen::Vector3d radial = d - axis * (d.dot(axis));
                        mean_r += radial.norm();
                    }
                    mean_r /= double(end - start);
                    seg_radii.push_back(mean_r);
                }

                // Create cylinder children for each segment; if adjacent segment centroids show curvature, create torus child instead
                for (size_t s = 0; s < seg_centroids.size(); ++s) {
                    PipingSequenceShape::ChildGeometry child;
                    child.type = "cylinder";
                    // cylinder coefficients: px,py,pz, dx,dy,dz, radius
                    child.coefficients = { seg_centroids[s].x(), seg_centroids[s].y(), seg_centroids[s].z(), axis.x(), axis.y(), axis.z(), seg_radii[s], 0 };
                    child.critical_points.push_back({ seg_centroids[s].x(), seg_centroids[s].y(), seg_centroids[s].z() });
                    for (auto &p : seg_points[s]) child.points.push_back({ p.x(), p.y(), p.z() });
                    // If bend detected with neighbor, instead produce a torus child spanning the bend
                    if (s+1 < seg_centroids.size()) {
                        Eigen::Vector3d v1 = (seg_centroids[s] - centroid).normalized();
                        Eigen::Vector3d v2 = (seg_centroids[s+1] - centroid).normalized();
                        double dot = v1.dot(v2);
                        dot = std::min(1.0, std::max(-1.0, dot));
                        double ang = std::acos(dot);
                        if (std::abs(ang) > 0.3) { // threshold for curvature => create torus
                            PipingSequenceShape::ChildGeometry torus;
                            torus.type = "torus";
                            // torus coefficients: center_x, center_y, center_z, nx,ny,nz, major, minor
                            Eigen::Vector3d mid = (seg_centroids[s] + seg_centroids[s+1]) * 0.5;
                            double major = (mid - centroid).norm();
                            double minor = 0.5 * (seg_radii[s] + seg_radii[s+1]);
                            torus.coefficients = { mid.x(), mid.y(), mid.z(), axis.x(), axis.y(), axis.z(), major, minor };
                            torus.critical_points.push_back({ seg_centroids[s].x(), seg_centroids[s].y(), seg_centroids[s].z() });
                            torus.critical_points.push_back({ mid.x(), mid.y(), mid.z() });
                            torus.critical_points.push_back({ seg_centroids[s+1].x(), seg_centroids[s+1].y(), seg_centroids[s+1].z() });
                            // merge points from both segments
                            for (auto &p : seg_points[s]) torus.points.push_back({p.x(), p.y(), p.z()});
                            for (auto &p : seg_points[s+1]) torus.points.push_back({p.x(), p.y(), p.z()});
                            shape_ptr->addChild(torus);
                            continue; // skip adding the cylinder for this segment pair
                        }
                    }
                    shape_ptr->addChild(child);
                }
            }

            return shape_ptr;
        }

        return nullptr;
    }
};

// Plane Factory implementation
class PlaneShapeFactory : public ShapeFactory {
private:
    Efficient_ransac::Parameters parameters_;
public:
    PlaneShapeFactory() {
        json s = (g_settings.contains("plane") && g_settings["plane"].is_object()) ? g_settings["plane"] : json::object();
        parameters_.min_points = s.value("min_points", 10);
        parameters_.epsilon = s.value("epsilon", 0.5);
        parameters_.cluster_epsilon = s.value("cluster_epsilon", 1.0);
        parameters_.normal_threshold = s.value("normal_threshold", 0.9);
        parameters_.probability = s.value("probability", 0.05);
    }
    
    std::string getShapeType() const override { return "plane"; }
    
    std::unique_ptr<Shape> detect(RANSAC_Pwn_vector& remaining_points, 
                                 size_t cluster_id, size_t& global_id) override {
        Efficient_ransac ransac;
        ransac.set_input(remaining_points);
        ransac.add_shape_factory<Plane>();
        
        // Create and set parameters object
        Efficient_ransac::Parameters ransac_parameters;
        ransac_parameters.min_points = parameters_.min_points;
        ransac_parameters.epsilon = parameters_.epsilon;
        ransac_parameters.cluster_epsilon = parameters_.cluster_epsilon;
        ransac_parameters.normal_threshold = parameters_.normal_threshold;
        ransac_parameters.probability = parameters_.probability;
        
        ransac.detect(ransac_parameters);
        
        // Find best shape
        size_t best_size = 0;
        auto best_shape = ransac.shapes().end();
        for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
            size_t shape_size = (*it)->indices_of_assigned_points().size();
            if (shape_size > best_size) {
                best_size = shape_size;
                best_shape = it;
            }
        }
        
        if (best_shape != ransac.shapes().end() && best_size >= 50) {
            auto plane = std::dynamic_pointer_cast<Plane>(*best_shape);
            auto plane_normal = plane->plane_normal();
            std::array<double, 4> coefficients = {
                plane_normal.x(), plane_normal.y(), plane_normal.z(), -plane->d()
            };
            
            auto assigned_indices = (*best_shape)->indices_of_assigned_points();
            std::vector<std::size_t> indices_vec(assigned_indices.begin(), assigned_indices.end());
            
            auto shape = std::make_unique<PlaneShape>(cluster_id, global_id++, indices_vec, remaining_points, coefficients);
            return shape;
        }
        
        return nullptr;
    }
};

// Cylinder Factory implementation
class CylinderShapeFactory : public ShapeFactory {
private:
    Efficient_ransac::Parameters parameters_;
public:
    CylinderShapeFactory() {
        json s = (g_settings.contains("cylinder") && g_settings["cylinder"].is_object()) ? g_settings["cylinder"] : json::object();
        parameters_.min_points = s.value("min_points", 8);
        parameters_.epsilon = s.value("epsilon", 2.0);
        parameters_.cluster_epsilon = s.value("cluster_epsilon", 2.5);
        parameters_.normal_threshold = s.value("normal_threshold", 0.6);
        parameters_.probability = s.value("probability", 0.01);
    }
    
    std::string getShapeType() const override { return "cylinder"; }
    
    std::unique_ptr<Shape> detect(RANSAC_Pwn_vector& remaining_points, 
                                 size_t cluster_id, size_t& global_id) override {
        Efficient_ransac ransac;
        ransac.set_input(remaining_points);
        ransac.add_shape_factory<Cylinder>();
        
        // Create and set parameters object
        Efficient_ransac::Parameters ransac_parameters;
        ransac_parameters.min_points = parameters_.min_points;
        ransac_parameters.epsilon = parameters_.epsilon;
        ransac_parameters.cluster_epsilon = parameters_.cluster_epsilon;
        ransac_parameters.normal_threshold = parameters_.normal_threshold;
        ransac_parameters.probability = parameters_.probability;
        
        ransac.detect(ransac_parameters);
        
        // Find best shape
        size_t best_size = 0;
        auto best_shape = ransac.shapes().end();
        for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
            size_t shape_size = (*it)->indices_of_assigned_points().size();
            if (shape_size > best_size) {
                best_size = shape_size;
                best_shape = it;
            }
        }
        
        if (best_shape != ransac.shapes().end() && best_size >= 30) {
            auto cylinder = std::dynamic_pointer_cast<Cylinder>(*best_shape);
            auto axis = cylinder->axis();
            auto axis_point = axis.point(0);
            auto axis_direction = axis.direction();
            
            std::array<double, 7> coefficients = {
                axis_point.x(), axis_point.y(), axis_point.z(),
                axis_direction.dx(), axis_direction.dy(), axis_direction.dz(),
                cylinder->radius()
            };
            
            auto assigned_indices = (*best_shape)->indices_of_assigned_points();
            std::vector<std::size_t> indices_vec(assigned_indices.begin(), assigned_indices.end());
            
            auto shape = std::make_unique<CylinderShape>(cluster_id, global_id++, indices_vec, remaining_points, coefficients);
            return shape;
        }
        
        return nullptr;
    }
};

// Cone Factory implementation
class ConeShapeFactory : public ShapeFactory {
private:
    Efficient_ransac::Parameters parameters_;
public:
    ConeShapeFactory() {
        json s = (g_settings.contains("cone") && g_settings["cone"].is_object()) ? g_settings["cone"] : json::object();
        parameters_.min_points = s.value("min_points", 8);
        parameters_.epsilon = s.value("epsilon", 2.0);
        parameters_.cluster_epsilon = s.value("cluster_epsilon", 2.5);
        parameters_.normal_threshold = s.value("normal_threshold", 0.6);
        parameters_.probability = s.value("probability", 0.01);
    }
    
    std::string getShapeType() const override { return "cone"; }
    
    std::unique_ptr<Shape> detect(RANSAC_Pwn_vector& remaining_points, 
                                 size_t cluster_id, size_t& global_id) override {
        Efficient_ransac ransac;
        ransac.set_input(remaining_points);
        ransac.add_shape_factory<Cone>();
        
        // Create and set parameters object
        Efficient_ransac::Parameters ransac_parameters;
        ransac_parameters.min_points = parameters_.min_points;
        ransac_parameters.epsilon = parameters_.epsilon;
        ransac_parameters.cluster_epsilon = parameters_.cluster_epsilon;
        ransac_parameters.normal_threshold = parameters_.normal_threshold;
        ransac_parameters.probability = parameters_.probability;
        
        ransac.detect(ransac_parameters);
        
        // Find best shape
        size_t best_size = 0;
        auto best_shape = ransac.shapes().end();
        for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
            size_t shape_size = (*it)->indices_of_assigned_points().size();
            if (shape_size > best_size) {
                best_size = shape_size;
                best_shape = it;
            }
        }
        
        if (best_shape != ransac.shapes().end() && best_size >= 30) {
            auto cone = std::dynamic_pointer_cast<Cone>(*best_shape);
            auto axis = cone->axis();
            auto apex = cone->apex();
            auto axis_direction = axis.direction();
            
            std::array<double, 7> coefficients = {
                apex.x(), apex.y(), apex.z(),
                axis_direction.dx(), axis_direction.dy(), axis_direction.dz(),
                cone->angle()
            };
            
            auto assigned_indices = (*best_shape)->indices_of_assigned_points();
            std::vector<std::size_t> indices_vec(assigned_indices.begin(), assigned_indices.end());
            
            auto shape = std::make_unique<ConeShape>(cluster_id, global_id++, indices_vec, remaining_points, coefficients);
            return shape;
        }
        
        return nullptr;
    }
};

// Torus Factory implementation
class TorusShapeFactory : public ShapeFactory {
private:
    Efficient_ransac::Parameters parameters_;
public:
    TorusShapeFactory() {
        json s = (g_settings.contains("torus") && g_settings["torus"].is_object()) ? g_settings["torus"] : json::object();
        parameters_.min_points = s.value("min_points", 8);
        parameters_.epsilon = s.value("epsilon", 3.0);
        parameters_.cluster_epsilon = s.value("cluster_epsilon", 3.5);
        parameters_.normal_threshold = s.value("normal_threshold", 0.5);
        parameters_.probability = s.value("probability", 0.01);
    }
    
    std::string getShapeType() const override { return "torus"; }
    
    std::unique_ptr<Shape> detect(RANSAC_Pwn_vector& remaining_points, 
                                 size_t cluster_id, size_t& global_id) override {
        Efficient_ransac ransac;
        ransac.set_input(remaining_points);
        ransac.add_shape_factory<Torus>();
        
        // Create and set parameters object
        Efficient_ransac::Parameters ransac_parameters;
        ransac_parameters.min_points = parameters_.min_points;
        ransac_parameters.epsilon = parameters_.epsilon;
        ransac_parameters.cluster_epsilon = parameters_.cluster_epsilon;
        ransac_parameters.normal_threshold = parameters_.normal_threshold;
        ransac_parameters.probability = parameters_.probability;
        
        ransac.detect(ransac_parameters);
        
        // Find best shape
        size_t best_size = 0;
        auto best_shape = ransac.shapes().end();
        for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
            size_t shape_size = (*it)->indices_of_assigned_points().size();
            if (shape_size > best_size) {
                best_size = shape_size;
                best_shape = it;
            }
        }
        
        if (best_shape != ransac.shapes().end() && best_size >= 30) {
            auto torus = std::dynamic_pointer_cast<Torus>(*best_shape);
            auto center = torus->center();
            auto normal = torus->axis();
            
            std::array<double, 8> coefficients = {
                center.x(), center.y(), center.z(),
                normal.x(), normal.y(), normal.z(),
                torus->major_radius(), torus->minor_radius()
            };
            
            auto assigned_indices = (*best_shape)->indices_of_assigned_points();
            std::vector<std::size_t> indices_vec(assigned_indices.begin(), assigned_indices.end());
            
            auto shape = std::make_unique<TorusShape>(cluster_id, global_id++, indices_vec, remaining_points, coefficients);
            return shape;
        }
        
        return nullptr;
    }
};

class HeadlessPointCloudProcessor {
private:
    PointCloudT::Ptr pcl_points_;
    NormalCloudT::Ptr pcl_normals_;
    RANSAC_Pwn_vector cgal_points_;
    
public:
    HeadlessPointCloudProcessor() : 
        pcl_points_(new PointCloudT),
        pcl_normals_(new NormalCloudT)
    {
    }
    
    // Load points from XYZ data (string format)
    bool loadPointsFromString(const std::string& xyz_data) {
        pcl_points_->clear();
        
        std::istringstream stream(xyz_data);
        std::string line;
        
        while (std::getline(stream, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            PointT point;
            if (iss >> point.x >> point.y >> point.z) {
                pcl_points_->points.push_back(point);
            }
        }
        
        pcl_points_->width = pcl_points_->points.size();
        pcl_points_->height = 1;
        pcl_points_->is_dense = true;
        
        if (pcl_points_->points.empty()) {
            return false;
        }
        
        // Estimate normals for the loaded point cloud
        estimateNormals();
        
        // Convert to CGAL format
        convertToCGAL();
        
        return true;
    }
    
    // Load points from JSON array format
    bool loadPointsFromJson(const json& points_array) {
        pcl_points_->clear();
        
        try {
            for (const auto& point : points_array) {
                PointT p;
                p.x = point["x"].get<float>();
                p.y = point["y"].get<float>();
                p.z = point["z"].get<float>();
                pcl_points_->points.push_back(p);
            }
            
            pcl_points_->width = pcl_points_->points.size();
            pcl_points_->height = 1;
            pcl_points_->is_dense = true;
            
            if (pcl_points_->points.empty()) {
                return false;
            }
            
            // Estimate normals for the loaded point cloud
            estimateNormals();
            
            // Convert to CGAL format
            convertToCGAL();
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing JSON points: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Process point cloud and return JSON results
    json processPointCloud(const std::string& solver_type = "BEST") {
        json result;
        if (pcl_points_->empty()) {
            result["error"] = "No points loaded";
            return result;
        }
        // Read clustering params from global settings
        json general = (g_settings.contains("general") && g_settings["general"].is_object()) ? g_settings["general"] : json::object();
        double cluster_tolerance = general.value("cluster_tolerance", 3.0);
        int min_cluster_size = general.value("min_cluster_size", 50);
        int max_cluster_size = general.value("max_cluster_size", 1000000);
        // Perform clustering
        auto pcl_clusters = performClusteringPCL(cluster_tolerance, min_cluster_size, max_cluster_size);
        auto converted_cgal_clusters = convertPCLtoCGALClusters(pcl_clusters);
        // Run shape detection
        json pcl_results, cgal_results;
        #ifdef USE_PCL
        pcl_results = detectShapesWithPCL(pcl_clusters);
        #endif
        #ifdef USE_CGAL
        cgal_results = detectShapesWithCGAL(converted_cgal_clusters, solver_type);
        #endif
        // Combine results
        result["pcl_results"] = pcl_results;
        result["cgal_results"] = cgal_results;
        result["total_points"] = pcl_points_->size();
        result["processing_timestamp"] = std::time(nullptr);
        return result;
    }
    
    size_t getCloudSize() const { return pcl_points_->size(); }
    
private:
    
    void estimateNormals(int k_neighbors = 20) {
        pcl::NormalEstimation<PointT, NormalT> ne;
        auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
        
        ne.setInputCloud(pcl_points_);
        ne.setSearchMethod(tree);
        ne.setKSearch(k_neighbors);
        ne.compute(*pcl_normals_);
    }
    
    void convertToCGAL() {
        cgal_points_.clear();
        
        if (pcl_points_->size() != pcl_normals_->size()) {
            std::cerr << "Warning: Point cloud and normals size mismatch!" << std::endl;
            return;
        }
        
        for (size_t i = 0; i < pcl_points_->size(); ++i) {
            const auto& pcl_point = pcl_points_->points[i];
            const auto& pcl_normal = pcl_normals_->points[i];
            
            RANSAC_Kernel::Point_3 cgal_point(pcl_point.x, pcl_point.y, pcl_point.z);
            RANSAC_Kernel::Vector_3 cgal_normal(pcl_normal.normal_x, pcl_normal.normal_y, pcl_normal.normal_z);
            
            cgal_points_.emplace_back(cgal_point, cgal_normal);
        }
    }
    
    std::vector<pcl::PointIndices> performClusteringPCL(
        double cluster_tolerance = 3.0,
        int min_cluster_size = 50,
        int max_cluster_size = 1000000) {
        
        auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
        tree->setInputCloud(pcl_points_);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_points_);
        ec.extract(cluster_indices);
        
        return cluster_indices;
    }
    
    // Utility function to convert PCL clusters to CGAL clusters
    std::vector<CGALClusterIndices> convertPCLtoCGALClusters(const std::vector<pcl::PointIndices>& pcl_clusters) {
        std::vector<CGALClusterIndices> cgal_clusters;
        cgal_clusters.reserve(pcl_clusters.size());
        
        for (const auto& pcl_cluster : pcl_clusters) {
            CGALClusterIndices cgal_cluster;
            cgal_cluster.indices.reserve(pcl_cluster.indices.size());
            
            // Convert int indices to size_t indices
            for (int idx : pcl_cluster.indices) {
                cgal_cluster.indices.push_back(static_cast<std::size_t>(idx));
            }
            
            cgal_clusters.push_back(std::move(cgal_cluster));
        }
        
        return cgal_clusters;
    }
     
    json detectShapesWithCGAL(const std::vector<CGALClusterIndices>& cluster_indices, const std::string& solver_type) {
        json result;
        result["method"] = "CGAL";
        result["shapes"] = json::array();
        result["clusters"] = json::array();

        // Select factories based on solver_type
        std::vector<std::unique_ptr<ShapeFactory>> factories;
        if (solver_type == "BEST" || solver_type == "") {
            factories.push_back(std::make_unique<PlaneShapeFactory>());
            factories.push_back(std::make_unique<CylinderShapeFactory>());
            factories.push_back(std::make_unique<ConeShapeFactory>());
            factories.push_back(std::make_unique<TorusShapeFactory>());
            factories.push_back(std::make_unique<PipingSequenceShapeFactory>());
        } else if (solver_type == "PLANE") {
            factories.push_back(std::make_unique<PlaneShapeFactory>());
        } else if (solver_type == "CYLINDER") {
            factories.push_back(std::make_unique<CylinderShapeFactory>());
        } else if (solver_type == "CONE") {
            factories.push_back(std::make_unique<ConeShapeFactory>());
        } else if (solver_type == "TORUS") {
            factories.push_back(std::make_unique<TorusShapeFactory>());
        } else if (solver_type == "PIPING_SEQUENCE") {
            factories.push_back(std::make_unique<PipingSequenceShapeFactory>());
        }
        
        // Process each cluster for shape detection
        size_t global_plane_count = 0;
        size_t global_cylinder_count = 0;
        size_t global_cone_count = 0;
        size_t global_torus_count = 0;
        size_t global_piping_count = 0;

        for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
            const auto& indices = cluster_indices[cluster_idx];

            json cluster_result;
            cluster_result["cluster_id"] = cluster_idx;
            cluster_result["total_points"] = indices.indices.size();
            cluster_result["shapes_found"] = json::array();

            if (indices.indices.size() < 10) {
                cluster_result["shapes_found_count"] = 0;
                cluster_result["remaining_points"] = indices.indices.size();
                result["clusters"].push_back(cluster_result);
                continue;
            }

            // Extract cluster points from pre-converted CGAL data
            RANSAC_Pwn_vector remaining_points;
            for (std::size_t idx : indices.indices) {
                if (idx < cgal_points_.size()) {
                    remaining_points.push_back(cgal_points_[idx]);
                }
            }

            // Shape fitting loop
            std::vector<std::unique_ptr<Shape>> cluster_shapes;
            const int max_shapes_per_cluster = 10;
            const int min_points_threshold = 50;

            while (cluster_shapes.size() < max_shapes_per_cluster && remaining_points.size() > min_points_threshold) {
                std::vector<std::unique_ptr<Shape>> candidate_shapes;

                if (solver_type == "BEST" || solver_type == "") {
                    // Try all factories and pick best shape
                    for (auto& factory : factories) {
                        size_t *gid_ptr = nullptr;
                        if (factory->getShapeType() == "plane") gid_ptr = &global_plane_count;
                        else if (factory->getShapeType() == "cylinder") gid_ptr = &global_cylinder_count;
                        else if (factory->getShapeType() == "cone") gid_ptr = &global_cone_count;
                        else if (factory->getShapeType() == "torus") gid_ptr = &global_torus_count;
                        else if (factory->getShapeType() == "piping_sequence") gid_ptr = &global_piping_count;

                        size_t gid_ref = gid_ptr ? *gid_ptr : 0;
                        auto shape = factory->detect(remaining_points, cluster_idx, gid_ref);
                        if (gid_ptr) *gid_ptr = gid_ref;

                        if (shape) {
                            candidate_shapes.push_back(std::move(shape));
                        }
                    }
                    // Find the best shape among candidates
                    std::unique_ptr<Shape> best_shape = nullptr;
                    size_t best_size = 0;
                    for (auto& shape : candidate_shapes) {
                        size_t shape_size = shape->getPointCount();
                        size_t min_threshold = 50;
                        if (shape->getType() == "cylinder" || shape->getType() == "cone" || shape->getType() == "torus") {
                            min_threshold = 30;
                        }
                        if (shape_size >= min_threshold && shape_size > best_size) {
                            best_size = shape_size;
                            best_shape = std::move(shape);
                        }
                    }
                    if (best_shape) {
                        // Update global counters
                        if (best_shape->getType() == "plane") {
                            global_plane_count++;
                        } else if (best_shape->getType() == "cylinder") {
                            global_cylinder_count++;
                        } else if (best_shape->getType() == "cone") {
                            global_cone_count++;
                        } else if (best_shape->getType() == "torus") {
                            global_torus_count++;
                        } else if (best_shape->getType() == "piping_sequence") {
                            global_piping_count++;
                        }
                        best_shape->setCriticalPoints();
                        // Remove assigned points from remaining points
                        const auto& indices_to_remove = best_shape->getAssignedIndices();
                        std::set<std::size_t> remove_set(indices_to_remove.begin(), indices_to_remove.end());
                        RANSAC_Pwn_vector new_remaining_points;
                        for (std::size_t i = 0; i < remaining_points.size(); ++i) {
                            if (remove_set.find(i) == remove_set.end()) {
                                new_remaining_points.push_back(remaining_points[i]);
                            }
                        }
                        remaining_points = std::move(new_remaining_points);
                        // Add shape to cluster results
                        json shape_json = best_shape->toJSON();
                        cluster_result["shapes_found"].push_back(shape_json);
                        result["shapes"].push_back(shape_json);
                        cluster_shapes.push_back(std::move(best_shape));
                    } else {
                        break;
                    }
                } else {
                    // Only use the selected factory
                    auto& factory = factories[0];
                    size_t *gid_ptr = nullptr;
                    if (factory->getShapeType() == "plane") gid_ptr = &global_plane_count;
                    else if (factory->getShapeType() == "cylinder") gid_ptr = &global_cylinder_count;
                    else if (factory->getShapeType() == "cone") gid_ptr = &global_cone_count;
                    else if (factory->getShapeType() == "torus") gid_ptr = &global_torus_count;
                    else if (factory->getShapeType() == "piping_sequence") gid_ptr = &global_piping_count;

                    size_t gid_ref = gid_ptr ? *gid_ptr : 0;
                    auto shape = factory->detect(remaining_points, cluster_idx, gid_ref);
                    if (gid_ptr) *gid_ptr = gid_ref;

                    if (shape) {
                        // Update global counters
                        if (shape->getType() == "plane") {
                            global_plane_count++;
                        } else if (shape->getType() == "cylinder") {
                            global_cylinder_count++;
                        } else if (shape->getType() == "cone") {
                            global_cone_count++;
                        } else if (shape->getType() == "torus") {
                            global_torus_count++;
                        } else if (shape->getType() == "piping_sequence") {
                            global_piping_count++;
                        }
                        shape->setCriticalPoints();
                        // Remove assigned points from remaining points
                        const auto& indices_to_remove = shape->getAssignedIndices();
                        std::set<std::size_t> remove_set(indices_to_remove.begin(), indices_to_remove.end());
                        RANSAC_Pwn_vector new_remaining_points;
                        for (std::size_t i = 0; i < remaining_points.size(); ++i) {
                            if (remove_set.find(i) == remove_set.end()) {
                                new_remaining_points.push_back(remaining_points[i]);
                            }
                        }
                        remaining_points = std::move(new_remaining_points);
                        // Add shape to cluster results
                        json shape_json = shape->toJSON();
                        cluster_result["shapes_found"].push_back(shape_json);
                        result["shapes"].push_back(shape_json);
                        cluster_shapes.push_back(std::move(shape));
                    } else {
                        break;
                    }
                }
            }

            cluster_result["shapes_found_count"] = cluster_shapes.size();
            cluster_result["remaining_points"] = remaining_points.size();
            result["clusters"].push_back(cluster_result);
        }

        result["summary"] = {
            {"total_planes", global_plane_count},
            {"total_cylinders", global_cylinder_count},
            {"total_cones", global_cone_count},
            {"total_torus", global_torus_count},
            {"total_piping_sequences", global_piping_count},
            {"total_shapes", global_plane_count + global_cylinder_count + global_cone_count + global_torus_count + global_piping_count},
            {"total_clusters_processed", cluster_indices.size()}
        };

        return result;
    }
};

// Main function for headless processing
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file> [output_file] [cluster_tolerance] [min_cluster_size] [max_cluster_size]" << std::endl;
        std::cerr << "Input formats supported: XYZ text file or JSON file with points array" << std::endl;
        std::cerr << "Default values: cluster_tolerance=3.0, min_cluster_size=50, max_cluster_size=1000000" << std::endl;
        return -1;
    }
    
    std::string input_file = argv[1];
    std::string output_file = (argc > 2) ? argv[2] : "";
    std::string solver_type = (argc > 3) ? argv[3] : "BEST";

    // Read settings from data/settings.json or fallback to data/base_settings
    // Load global settings
    g_settings = json();
    std::ifstream settings_file("/mnt/c/Temp/pcl_processing/settings.json");
    if (settings_file.is_open()) {
        try {
            settings_file >> g_settings;
        } catch (...) {
            std::cerr << "Warning: Failed to parse settings.json, will try base_settings." << std::endl;
        }
        settings_file.close();
    }
    if (g_settings.empty()) {
        std::ifstream base_file("data/base_settings");
        if (base_file.is_open()) {
            try {
                base_file >> g_settings;
            } catch (...) {
                std::cerr << "Error: Failed to parse base_settings." << std::endl;
            }
            base_file.close();
        }
    }
    if (g_settings.empty()) {
        std::cerr << "Error: No valid settings found in settings.json or base_settings." << std::endl;
        return -1;
    }

    std::cout << "Processing parameters from settings:" << std::endl;
    std::cout << "  Solver type: " << solver_type << std::endl;

    HeadlessPointCloudProcessor processor;

    // Load input data
    std::ifstream file(input_file);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open input file: " << input_file << std::endl;
        return -1;
    }

    // Read entire file content
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    bool loaded = false;

    // Try to parse as JSON first
    try {
        json input_json = json::parse(content);
        if (input_json.contains("points") && input_json["points"].is_array()) {
            loaded = processor.loadPointsFromJson(input_json["points"]);
        }
    } catch (const std::exception&) {
        // Not JSON, try as XYZ format
        loaded = processor.loadPointsFromString(content);
    }

    if (!loaded) {
        std::cerr << "Error: Failed to load point cloud data from: " << input_file << std::endl;
        return -1;
    }

    std::cout << "Loaded " << processor.getCloudSize() << " points from " << input_file << std::endl;

    // Process the point cloud with parameters from global settings
    json result = processor.processPointCloud(solver_type);
    result["solver_type"] = solver_type;

    // Output results
    if (!output_file.empty()) {
        std::ofstream outfile(output_file);
        if (outfile.is_open()) {
            outfile << result.dump(4) << std::endl;
            outfile.close();
            std::cout << "Results written to: " << output_file << std::endl;
        } else {
            std::cerr << "Error: Cannot write to output file: " << output_file << std::endl;
            std::cout << result.dump(4) << std::endl;
        }
    } else {
        std::cout << result.dump(4) << std::endl;
    }

    return 0;
}
