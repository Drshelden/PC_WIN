#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>

#include <pcl/kdtree/kdtree_flann.h>

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

// Convert a vector of PointT (critical points) into an Nx3 numpy array
static py::array_t<float> points_vector_to_numpy(const std::vector<pcl::PointXYZ> &vec) {
    std::vector<py::ssize_t> shape;
    shape.push_back(static_cast<py::ssize_t>(vec.size()));
    shape.push_back(3);
    py::array_t<float> arr(shape);
    float *buf = arr.mutable_data();
    for (size_t i = 0; i < vec.size(); ++i) {
        buf[i*3 + 0] = vec[i].x;
        buf[i*3 + 1] = vec[i].y;
        buf[i*3 + 2] = vec[i].z;
    }
    return arr;
}

// Return cluster as Nx6 numpy array (x,y,z,nx,ny,nz) by nearest-neighbor lookup into the original PC's cloud+normals
static py::array_t<float> cluster_with_normals(const ShapeFinder &sf, const PCWin_PointCloud &pc, size_t idx) {
    if (idx >= sf.clusters.size()) throw std::out_of_range("cluster index out of range");
    auto cluster = sf.clusters[idx];
    size_t n = cluster->size();
    std::vector<py::ssize_t> shape = { (py::ssize_t)n, 6 };
    py::array_t<float> arr(shape);
    float *buf = arr.mutable_data();

    // if original cloud or normals missing, fill normals with 0
    bool have_orig = (pc.cloud && !pc.cloud->empty());
    bool have_normals = (pc.normals && !pc.normals->empty() && have_orig && pc.normals->size() == pc.cloud->size());

    pcl::KdTreeFLANN<pcl::PointXYZ> kdt;
    if (have_orig) kdt.setInputCloud(pc.cloud);

    for (size_t i = 0; i < n; ++i) {
        const auto &p = cluster->points[i];
        float nx = 0.0f, ny = 0.0f, nz = 0.0f;
        if (have_orig) {
            std::vector<int> nn(1);
            std::vector<float> dist(1);
            if (kdt.nearestKSearch(p, 1, nn, dist) > 0) {
                int ii = nn[0];
                if (have_normals) {
                    const auto &nrm = pc.normals->points[ii];
                    nx = nrm.normal_x; ny = nrm.normal_y; nz = nrm.normal_z;
                }
            }
        }
        buf[i*6 + 0] = p.x;
        buf[i*6 + 1] = p.y;
        buf[i*6 + 2] = p.z;
        buf[i*6 + 3] = nx;
        buf[i*6 + 4] = ny;
        buf[i*6 + 5] = nz;
    }
    return arr;
}

// Return a NumPy structured array with fields (x,y,z,nx,ny,nz) using a view on the Nx6 float array
static py::object cluster_structured(const ShapeFinder &sf, const PCWin_PointCloud &pc, size_t idx) {
    py::array arr6 = cluster_with_normals(sf, pc, idx);
    // ensure contiguous
    py::module np = py::module::import("numpy");
    arr6 = np.attr("ascontiguousarray")(arr6);
    py::buffer_info binfo = arr6.request();
    if (binfo.ndim != 2 || binfo.shape[1] != 6) {
        throw std::runtime_error("expected Nx6 float array from cluster_with_normals");
    }
    py::ssize_t n = binfo.shape[0];

    // build dtype: list of (name, type)
    py::list fields;
    fields.append(py::make_tuple(py::str("x"), py::str("f4")));
    fields.append(py::make_tuple(py::str("y"), py::str("f4")));
    fields.append(py::make_tuple(py::str("z"), py::str("f4")));
    fields.append(py::make_tuple(py::str("nx"), py::str("f4")));
    fields.append(py::make_tuple(py::str("ny"), py::str("f4")));
    fields.append(py::make_tuple(py::str("nz"), py::str("f4")));

    py::object dtype = np.attr("dtype")(fields);

    // view the (n,6) float32 array as a (n,) structured array of 6 float32 fields
    py::object viewed = arr6.attr("view")(dtype);
    viewed = viewed.attr("reshape")(py::make_tuple(n));
    return viewed;
}

PYBIND11_MODULE(pcl_hybrid_py, m) {
    m.doc() = "Python bindings for pcl_hybrid core functionality";

    py::class_<PCWin_PointCloud>(m, "PCWin_PointCloud")
        .def(py::init<>())
        .def("importPoints", &PCWin_PointCloud::importPoints, "Import points from file")
        .def("importPointsFromBuffer", [](PCWin_PointCloud &pc, py::buffer b){
            // Accepts a buffer/ndarray of float32, shape (N,3) or flat length 3*N
            py::buffer_info info = b.request();
            if (info.format != py::format_descriptor<float>::format() && info.format != "f")
                throw std::runtime_error("importPointsFromBuffer requires float32 buffer");
            ssize_t len = 1;
            for (auto d : info.shape) len *= d;
            if (len % 3 != 0) throw std::runtime_error("buffer length must be multiple of 3 (x,y,z)");
            size_t n_points = (size_t)(len / 3);
            // ensure contiguous
            py::array_t<float> arr = py::array_t<float>(info.shape, info.strides, (float*)info.ptr).attr("copy")();
            float *data = arr.mutable_data();
            return pc.importPointsFromBuffer(data, n_points);
        }, "Import points from an in-memory float32 buffer (x,y,z repeating)")
        .def("get_points_array", [](const PCWin_PointCloud &pc){ return pointcloud_to_numpy(pc.cloud); }, "Return point cloud as Nx3 numpy array")
        .def("get_points_and_normals", [](const PCWin_PointCloud &pc){
            // if normals are present and match cloud size, produce Nx6 array, otherwise Nx3
            if (pc.cloud && pc.normals && pc.cloud->size() == pc.normals->size()) {
                size_t n = pc.cloud->size();
                std::vector<py::ssize_t> shape = { (py::ssize_t)n, 6 };
                py::array_t<float> arr(shape);
                float *buf = arr.mutable_data();
                for (size_t i = 0; i < n; ++i) {
                    const auto &p = pc.cloud->points[i];
                    const auto &nr = pc.normals->points[i];
                    buf[i*6 + 0] = p.x; buf[i*6 + 1] = p.y; buf[i*6 + 2] = p.z;
                    buf[i*6 + 3] = nr.normal_x; buf[i*6 + 4] = nr.normal_y; buf[i*6 + 5] = nr.normal_z;
                }
                return py::object(arr);
            }
            return py::object(pointcloud_to_numpy(pc.cloud));
        }, "Return point cloud as Nx3 or Nx6 numpy array (with normals if available)")
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
        .def("get_cluster_with_normals", [](const ShapeFinder &s, const PCWin_PointCloud &pc, size_t idx){ return cluster_with_normals(s, pc, idx); }, "Return Nx6 array (x,y,z,nx,ny,nz) for a cluster by matching nearest points in the original cloud")
        .def("get_cluster_structured", [](const ShapeFinder &s, const PCWin_PointCloud &pc, size_t idx){ return cluster_structured(s, pc, idx); }, "Return a structured numpy array with fields (x,y,z,nx,ny,nz) for a cluster")
        .def("get_root_shape", [](const ShapeFinder &s) -> std::shared_ptr<Shape> {
            if (!s.rootShape) return nullptr;
            return std::static_pointer_cast<Shape>(s.rootShape);
        }, "Return the root Shape (or None)")
        ;

    // Expose Shape hierarchy so Python can walk the tree
    py::class_<Shape, std::shared_ptr<Shape>>(m, "Shape")
        .def("get_type", &Shape::getType)
        .def("to_json", &Shape::toJSON)
        .def("get_points_array", [](const Shape &sh){ return pointcloud_to_numpy(sh.getPoints()); })
        .def("get_children", [](const Shape &sh){ py::list out; for (auto &c : sh.getChildren()) out.append(c); return out; })
        ;

    py::class_<PlaneShape, Shape, std::shared_ptr<PlaneShape>>(m, "PlaneShape")
        .def("get_plane_label", &PlaneShape::getPlaneLabel)
        .def("get_coefficients", [](const PlaneShape &p){ return *(p.getCoefficients()); })
        .def("get_critical_points_array", [](const PlaneShape &p){ return points_vector_to_numpy(*(p.getCriticalPoints())); })
        ;

    py::class_<CylinderShape, Shape, std::shared_ptr<CylinderShape>>(m, "CylinderShape")
        .def("get_cylinder_label", &CylinderShape::getCylinderLabel)
        .def("get_coefficients", [](const CylinderShape &c){ return *(c.getCoefficients()); })
        .def("get_critical_points_array", [](const CylinderShape &c){ return points_vector_to_numpy(*(c.getCriticalPoints())); })
        ;

    py::class_<GenericShape, Shape, std::shared_ptr<GenericShape>>(m, "GenericShape")
        .def(py::init<>())
        ;
}
