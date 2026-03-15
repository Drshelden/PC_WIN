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
    // children
    if (!children_.empty()) {
        j["children"] = json::array();
        for (const auto &c : children_) j["children"].push_back(json::parse(c->toJSON()));
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
    if (!children_.empty()) {
        j["children"] = json::array();
        for (const auto &c : children_) j["children"].push_back(json::parse(c->toJSON()));
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
    if (!children_.empty()) {
        j["children"] = json::array();
        for (const auto &c : children_) j["children"].push_back(json::parse(c->toJSON()));
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
    // Determine the weak axis from labels: 0->X, 1->Y, 2->Z.
    int weak_axis = cylinder_label_ >= 0 ? cylinder_label_ : 2;
    int a = (weak_axis + 1) % 3;
    int b = (weak_axis + 2) % 3;

    const size_t n = points_->size();
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd rhs(n);

    double axis_min = std::numeric_limits<double>::infinity();

    // Fit circle in the plane orthogonal to axis:
    // u^2 + v^2 + alpha*u + beta*v + gamma = 0
    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d p(points_->points[i].x, points_->points[i].y, points_->points[i].z);
        double u = p[a];
        double v = p[b];
        A(static_cast<Eigen::Index>(i), 0) = u;
        A(static_cast<Eigen::Index>(i), 1) = v;
        A(static_cast<Eigen::Index>(i), 2) = 1.0;
        rhs(static_cast<Eigen::Index>(i)) = -(u * u + v * v);

        axis_min = std::min(axis_min, p[weak_axis]);
    }

    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(rhs);
    double alpha = sol[0];
    double beta = sol[1];
    double gamma = sol[2];

    double center_u = -0.5 * alpha;
    double center_v = -0.5 * beta;
    double radius_sq = center_u * center_u + center_v * center_v - gamma;
    if (radius_sq < 0.0) radius_sq = 0.0;
    double radius = std::sqrt(radius_sq);

    Eigen::Vector3d p1(0.0, 0.0, 0.0);
    p1[weak_axis] = axis_min;
    p1[a] = center_u;
    p1[b] = center_v;

    Eigen::Vector3d axis(0.0, 0.0, 0.0);
    axis[weak_axis] = 1.0;

    (*coefficients_)[0] = p1.x();
    (*coefficients_)[1] = p1.y();
    (*coefficients_)[2] = p1.z();
    (*coefficients_)[3] = axis.x();
    (*coefficients_)[4] = axis.y();
    (*coefficients_)[5] = axis.z();
    (*coefficients_)[6] = radius;
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
