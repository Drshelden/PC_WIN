#include "shapes.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <random>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>

using json = nlohmann::json;

std::string CylinderShape::toJSON() const {
    json j;
    j["type"] = getType();
    j["cylinder_label"] = getCylinderLabel();
    j["points"] = json::array();
    for (const auto& pt : points_->points) {
        j["points"].push_back({pt.x, pt.y, pt.z});
    }
    // coefficients
    j["coefficients"] = json::array();
    if (coefficients_) {
        for (double v : *coefficients_) j["coefficients"].push_back(v);
    }
    // critical points
    j["critical_points"] = json::array();
    if (critical_points_) {
        for (const auto &p : *critical_points_) j["critical_points"].push_back({p.x, p.y, p.z});
    }
    return j.dump();
}

std::string PlaneShape::toJSON() const {
    json j;
    j["type"] = getType();
    j["points"] = json::array();
    j["plane_label"] = getPlaneLabel();
    for (const auto& pt : points_->points) {
        j["points"].push_back({pt.x, pt.y, pt.z});
    }
    j["coefficients"] = json::array();
    if (coefficients_) {
        for (double v : *coefficients_) j["coefficients"].push_back(v);
    }
    j["critical_points"] = json::array();
    if (critical_points_) {
        for (const auto &p : *critical_points_) j["critical_points"].push_back({p.x, p.y, p.z});
    }
    return j.dump();
}

std::string GenericShape::toJSON() const {
    json j;
    j["type"] = getType();
    j["points"] = json::array();
    if (points_) {
        for (const auto& pt : points_->points) {
            j["points"].push_back({pt.x, pt.y, pt.z});
        }
    }
    return j.dump();
}

// Utility: 2D convex hull (Monotone chain), points are pair<double,double>
static std::vector<std::pair<double,double>> convexHull2D(std::vector<std::pair<double,double>> pts) {
    if (pts.size() <= 1) return pts;
    std::sort(pts.begin(), pts.end());
    std::vector<std::pair<double,double>> lo, hi;
    for (const auto &p : pts) {
        while (lo.size() >= 2) {
            auto a = lo[lo.size()-2], b = lo[lo.size()-1];
            double cross = (b.first - a.first)*(p.second - a.second) - (b.second - a.second)*(p.first - a.first);
            if (cross <= 0) lo.pop_back(); else break;
        }
        lo.push_back(p);
    }
    for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
        const auto &p = *it;
        while (hi.size() >= 2) {
            auto a = hi[hi.size()-2], b = hi[hi.size()-1];
            double cross = (b.first - a.first)*(p.second - a.second) - (b.second - a.second)*(p.first - a.first);
            if (cross <= 0) hi.pop_back(); else break;
        }
        hi.push_back(p);
    }
    lo.pop_back(); hi.pop_back();
    lo.insert(lo.end(), hi.begin(), hi.end());
    return lo;
}

// PlaneShape: set coefficients (nx,ny,nz,d) and critical points (convex hull projected onto plane)
void PlaneShape::setCoefficients() {
    if (!points_ || points_->empty()) return;
    // determine normal from plane_label: 0->X,1->Y,2->Z; fallback compute using PCA
    double nx=0, ny=0, nz=0;
    if (plane_label_ == 0) { nx = 1.0; }
    else if (plane_label_ == 1) { ny = 1.0; }
    else if (plane_label_ == 2) { nz = 1.0; }
    else {
        // PCA for normal
        Eigen::MatrixXd M(points_->size(),3);
        for (size_t i=0;i<points_->size();++i){ M(i,0)=points_->points[i].x; M(i,1)=points_->points[i].y; M(i,2)=points_->points[i].z; }
    Eigen::RowVector3d mean = M.colwise().mean();
    for (Eigen::Index i = 0; i < M.rows(); ++i) M.row(i) -= mean;
        Eigen::Matrix3d cov = (M.adjoint()*M)/(M.rows()-1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Vector3d normal = es.eigenvectors().col(0); // smallest eigenvalue
        nx = normal.x(); ny = normal.y(); nz = normal.z();
    }
    // normalize
    double norm = std::sqrt(nx*nx+ny*ny+nz*nz);
    if (norm < 1e-12) { nx=0; ny=0; nz=1; norm=1.0; }
    nx/=norm; ny/=norm; nz/=norm;
    // D is average coordinate along dominant axis
    double D=0.0;
    if (plane_label_==0) {
        for (const auto &p : points_->points) { D += p.x; }
        D = - (D / static_cast<double>(points_->size()));
    } else if (plane_label_==1) {
        for (const auto &p : points_->points) { D += p.y; }
        D = - (D / static_cast<double>(points_->size()));
    } else if (plane_label_==2) {
        for (const auto &p : points_->points) { D += p.z; }
        D = - (D / static_cast<double>(points_->size()));
    } else {
        // project mean onto normal: d = -n dot mean => plane equation n.x * X + ... + d = 0; store -d
        Eigen::Vector3d mean(0,0,0);
        for (const auto &p : points_->points) { mean += Eigen::Vector3d(p.x,p.y,p.z); }
        mean /= points_->size();
        D = -(nx*mean.x() + ny*mean.y() + nz*mean.z());
    }
    if (coefficients_) {
        (*coefficients_)[0] = nx; (*coefficients_)[1] = ny; (*coefficients_)[2] = nz; (*coefficients_)[3] = D;
    }
}

void PlaneShape::setCriticalPoints() {
    if (!points_ || points_->empty()) return;
    // Use plane coefficients to project points onto plane and compute 2D convex hull
    double nx = (*coefficients_)[0]; double ny = (*coefficients_)[1]; double nz = (*coefficients_)[2]; double D = (*coefficients_)[3];
    // Choose arbitrary in-plane basis u,v
    Eigen::Vector3d n(nx,ny,nz);
    if (n.norm() < 1e-9) n = Eigen::Vector3d(0,0,1);
    n.normalize();
    Eigen::Vector3d u;
    if (std::abs(n.x()) < 0.9) u = Eigen::Vector3d(1,0,0); else u = Eigen::Vector3d(0,1,0);
    u -= u.dot(n)*n; if (u.norm() < 1e-9) u = Eigen::Vector3d(0,1,0); u.normalize();
    Eigen::Vector3d v = n.cross(u); v.normalize();
    // Project points
    std::vector<std::pair<double,double>> pts2d; pts2d.reserve(points_->size());
    for (const auto &p : points_->points) {
        Eigen::Vector3d P(p.x,p.y,p.z);
        // project onto plane: P_proj = P - (n.dot(P)+D)*n
        double dist = n.dot(P) + D;
        Eigen::Vector3d Pp = P - dist * n;
        double x = Pp.dot(u); double y = Pp.dot(v);
        pts2d.emplace_back(x,y);
    }
    auto hull2d = convexHull2D(pts2d);
    if (!critical_points_) critical_points_ = std::make_shared<std::vector<PointT>>();
    critical_points_->clear();
    // map back 2D hull to 3D
    for (auto &hp : hull2d) {
        Eigen::Vector3d P3 = u*hp.first + v*hp.second - D*n; // approximate point on plane
        PointT pt; pt.x = static_cast<float>(P3.x()); pt.y = static_cast<float>(P3.y()); pt.z = static_cast<float>(P3.z());
        critical_points_->push_back(pt);
    }
}

// CylinderShape: simple PCA axis and radius estimate; critical points are axis endpoints
void CylinderShape::setCoefficients() {
    if (!points_ || points_->empty()) return;
    // Build local arrays of point positions and (optional) normals if available
    std::vector<Eigen::Vector3d> P;
    P.reserve(points_->size());
    for (const auto &pt : points_->points) P.emplace_back(pt.x, pt.y, pt.z);

    // For sampling we need normals. We will estimate normals locally via PCA on k-neighbors
    // Build a temporary PCL cloud and compute normals if needed
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pt : points_->points) tmp->push_back(pt);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(tmp);
    ne.setSearchMethod(tree);
    ne.setKSearch(std::min<size_t>(30, tmp->size()));
    pcl::PointCloud<pcl::Normal> normals;
    ne.compute(normals);

    // Determine the weak axis (cylinder plane) from cylinder_label_: 0->X weak axis is X, etc.
    int weak_axis = cylinder_label_ >= 0 ? cylinder_label_ : 2;

    // sampling parameters
    std::mt19937 rng(12345);
    size_t sampleN = std::min<size_t>(30, P.size());

    std::vector<Eigen::Vector3d> pint_accum;
    pint_accum.reserve(sampleN);

    std::uniform_int_distribution<size_t> dist(0, P.size()-1);
    for (size_t s=0; s<sampleN; ++s) {
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);
        if (i1 == i2) continue;
        Eigen::Vector3d p1 = P[i1]; Eigen::Vector3d p2 = P[i2];
        Eigen::Vector3d n1(0,0,1), n2(0,0,1);
        if (i1 < normals.size()) n1 = Eigen::Vector3d(normals[i1].normal_x, normals[i1].normal_y, normals[i1].normal_z);
        if (i2 < normals.size()) n2 = Eigen::Vector3d(normals[i2].normal_x, normals[i2].normal_y, normals[i2].normal_z);

        // zero the weak axis coordinate to project into the governing plane
        if (weak_axis == 0) { p1.x() = 0; p2.x() = 0; n1.x() = 0; n2.x() = 0; }
        else if (weak_axis == 1) { p1.y() = 0; p2.y() = 0; n1.y() = 0; n2.y() = 0; }
        else { p1.z() = 0; p2.z() = 0; n1.z() = 0; n2.z() = 0; }

        // In-plane intersection of lines p1 + t*n1 and p2 + u*n2.
        // Solve for intersection in 2D by choosing two coordinates that remain (indices a,b)
        int a = (weak_axis + 1) % 3; int b = (weak_axis + 2) % 3;
        double x1 = p1[a], y1 = p1[b], dx1 = n1[a], dy1 = n1[b];
        double x2 = p2[a], y2 = p2[b], dx2 = n2[a], dy2 = n2[b];
        double denom = dx1*dy2 - dy1*dx2;
        if (std::abs(denom) < 1e-9) continue; // parallel
        double t = ( (x2 - x1)*dy2 - (y2 - y1)*dx2 ) / denom;
        Eigen::Vector3d pint;
        // reconstruct full 3D pint by inserting the remaining coordinate as 0 in weak axis slot
        for (int k=0;k<3;++k) pint[k] = 0.0;
        pint[a] = x1 + t*dx1; pint[b] = y1 + t*dy1;
        pint_accum.push_back(pint);
    }

    if (pint_accum.empty()) {
        // fallback to centroid
        Eigen::Vector3d centroid(0,0,0); for (auto &p : P) centroid += p; centroid /= P.size();
        (*coefficients_)[0] = centroid.x(); (*coefficients_)[1] = centroid.y(); (*coefficients_)[2] = centroid.z();
        // axis is the weak axis unit vector
        Eigen::Vector3d axis(0,0,0); axis[weak_axis] = 1.0;
        (*coefficients_)[3] = axis.x(); (*coefficients_)[4] = axis.y(); (*coefficients_)[5] = axis.z();
        // radius average
        double rsum = 0; for (auto &p : P) { Eigen::Vector3d v = p - centroid; Eigen::Vector3d proj = axis*(v.dot(axis)); rsum += (v - proj).norm(); }
        (*coefficients_)[6] = rsum / P.size();
        return;
    }

    // average pint to get porig
    Eigen::Vector3d porig(0,0,0); for (auto &pp : pint_accum) porig += pp; porig /= pint_accum.size();
    // axis is weak-axis unit vector
    Eigen::Vector3d axis(0,0,0); axis[weak_axis] = 1.0;

    // project points onto axis, find min/max projections and average perpendicular distance
    double amin = std::numeric_limits<double>::infinity(), amax = -std::numeric_limits<double>::infinity();
    double rsum = 0.0;
    for (const auto &p : P) {
        Eigen::Vector3d v = p - porig;
        double proj = v.dot(axis);
        amin = std::min(amin, proj); amax = std::max(amax, proj);
        Eigen::Vector3d perp = v - proj*axis; rsum += perp.norm();
    }
    double radius = rsum / P.size();

    Eigen::Vector3d p1 = porig + axis * amin;
    (*coefficients_)[0] = p1.x(); (*coefficients_)[1] = p1.y(); (*coefficients_)[2] = p1.z();
    (*coefficients_)[3] = axis.x(); (*coefficients_)[4] = axis.y(); (*coefficients_)[5] = axis.z(); (*coefficients_)[6] = radius;
}

void CylinderShape::setCriticalPoints() {
    if (!points_ || points_->empty()) return;
    if (!coefficients_) return;
    Eigen::Vector3d p1((*coefficients_)[0], (*coefficients_)[1], (*coefficients_)[2]);
    Eigen::Vector3d axis((*coefficients_)[3], (*coefficients_)[4], (*coefficients_)[5]); axis.normalize();
    // project each point onto axis to find min/max
    double amin = std::numeric_limits<double>::infinity(), amax = -std::numeric_limits<double>::infinity();
    Eigen::Vector3d porig = p1; // p1 was chosen as porig origin in setCoefficients
    for (const auto &pt : points_->points) {
        Eigen::Vector3d v(pt.x - porig.x(), pt.y - porig.y(), pt.z - porig.z());
        double proj = v.dot(axis);
        amin = std::min(amin, proj); amax = std::max(amax, proj);
    }
    Eigen::Vector3d cp1 = porig + axis * amin; Eigen::Vector3d cp2 = porig + axis * amax;
    if (!critical_points_) critical_points_ = std::make_shared<std::vector<PointT>>();
    critical_points_->clear();
    PointT pp1; pp1.x = static_cast<float>(cp1.x()); pp1.y = static_cast<float>(cp1.y()); pp1.z = static_cast<float>(cp1.z());
    PointT pp2; pp2.x = static_cast<float>(cp2.x()); pp2.y = static_cast<float>(cp2.y()); pp2.z = static_cast<float>(cp2.z());
    critical_points_->push_back(pp1); critical_points_->push_back(pp2);
}
