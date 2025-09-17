#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>

#include "PCWinPointCloud.h"
#include "ShapeFinder.h"
#include "shapes.h"

namespace py = pybind11;

// Helper to produce root JSON using the existing toJSON() on shapes
static std::string shape_root_to_json(const ShapeFinder &sf) {
    if (!sf.rootShape) return std::string("{}");
    return sf.rootShape->toJSON();
}

// Convert a pcl::PointCloud<PointT>::Ptr to a contiguous NumPy array (N x 3, float32)
static py::array_t<float> pointcloud_to_numpy(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    if (!cloud) return py::array_t<float>();
    size_t n = cloud->size();
    if (n == 0) return py::array_t<float>({0,3});
    // create array with shape (n,3) using a shape vector to avoid MSVC initializer-list issues
    std::vector<py::ssize_t> shape;
    shape.push_back(static_cast<py::ssize_t>(n));
    shape.push_back(3);
    py::array_t<float> arr(shape);
    float *buf = arr.mutable_data();
    for (size_t i = 0; i < n; ++i) {
        const auto &p = cloud->points[i];
        buf[i*3 + 0] = p.x;
        buf[i*3 + 1] = p.y;
        buf[i*3 + 2] = p.z;
    }
    return arr;
}

PYBIND11_MODULE(pcl_hybrid_py, m) {
    m.doc() = "Python bindings for pcl_hybrid core functionality";

    py::class_<PCWin_PointCloud>(m, "PCWin_PointCloud")
        .def(py::init<>())
        .def("importPoints", &PCWin_PointCloud::importPoints, "Import points from file")
        .def("get_points_array", [](const PCWin_PointCloud &pc){ return pointcloud_to_numpy(pc.cloud); }, "Return point cloud as Nx3 numpy array")
        ;

    py::class_<ShapeFinder>(m, "ShapeFinder")
        .def(py::init<>())
        .def("findShapes", &ShapeFinder::findShapes, "Run shape finding on a PCWin_PointCloud")
        .def("get_root_json", &shape_root_to_json, "Return the root shape JSON string")
        .def("clusters_size", [](const ShapeFinder &s){ return (size_t)s.clusters.size(); })
        .def("get_cluster_size", [](const ShapeFinder &s, size_t idx){
            if (idx >= s.clusters.size()) throw std::out_of_range("cluster index out of range");
            return (size_t)s.clusters[idx]->size();
        })
        .def("get_cluster_array", [](const ShapeFinder &s, size_t idx){
            if (idx >= s.clusters.size()) throw std::out_of_range("cluster index out of range");
            return pointcloud_to_numpy(s.clusters[idx]);
        }, "Return a single cluster as an Nx3 numpy array")
        .def("get_all_clusters", [](const ShapeFinder &s){
            py::list out;
            for (const auto &c : s.clusters) out.append(pointcloud_to_numpy(c));
            return out;
        }, "Return a list of Nx3 numpy arrays, one per cluster")
        ;
}
