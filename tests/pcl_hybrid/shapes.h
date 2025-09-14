#pragma once

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;

// Abstract Shape
class Shape {
public:
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
    virtual ~Shape() = default;
    virtual std::string getType() const = 0;
    // toJSON uses nlohmann::json if available; we forward-declare to avoid heavy dependency here
    virtual std::string toJSON() const = 0;
    virtual PointCloudPtr getPoints() const = 0;
    virtual int getPlaneLabel() const { return -1; }
    virtual int getCylinderLabel() const { return -1; }
    // Child shapes (hierarchical decomposition)
    void addChild(std::shared_ptr<Shape> child) { children_.push_back(child); }
    const std::vector<std::shared_ptr<Shape>>& getChildren() const { return children_; }

protected:
    std::vector<std::shared_ptr<Shape>> children_;

};

// PlaneShape stores cluster points; for this simplified version we don't fit parameters
// PlaneShape stores cluster points and pointers to coefficients and critical points
class PlaneShape : public Shape {
public:
    using CoeffPtr = std::shared_ptr<std::vector<double>>; // nx,ny,nz,d (4)
    using CritPtr = std::shared_ptr<std::vector<PointT>>;

    PlaneShape(const PointCloudPtr& pts, int plane_label = -1)
        : points_(pts), plane_label_(plane_label) {
        coefficients_ = std::make_shared<std::vector<double>>(4, 0.0);
        critical_points_ = std::make_shared<std::vector<PointT>>();
        setCoefficients();
        setCriticalPoints();
    }

    std::string getType() const override { return "plane"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }
    int getPlaneLabel() const { return plane_label_; }

    void setCoefficients();
    void setCriticalPoints();

    CoeffPtr getCoefficients() const { return coefficients_; }
    CritPtr getCriticalPoints() const { return critical_points_; }

private:
    PointCloudPtr points_;
    int plane_label_ = -1;
    CoeffPtr coefficients_;
    CritPtr critical_points_;
};

// Extended CylinderShape with coefficients and critical points
class CylinderShape : public Shape {
public:
    using CoeffPtr = std::shared_ptr<std::vector<double>>; // px,py,pz, dx,dy,dz, radius (7)
    using CritPtr = std::shared_ptr<std::vector<PointT>>;

    CylinderShape(const PointCloudPtr& pts, int cylinder_label = -1)
        : points_(pts), cylinder_label_(cylinder_label) {
        coefficients_ = std::make_shared<std::vector<double>>(7, 0.0);
        critical_points_ = std::make_shared<std::vector<PointT>>();
        setCoefficients();
        setCriticalPoints();
    }

    std::string getType() const override { return "cylinder"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }
    int getCylinderLabel() const override { return cylinder_label_; }

    // compute and populate coefficient array
    void setCoefficients();
    // compute and populate critical points (e.g., axis endpoints)
    void setCriticalPoints();

    CoeffPtr getCoefficients() const { return coefficients_; }
    CritPtr getCriticalPoints() const { return critical_points_; }

private:
    PointCloudPtr points_;
    int cylinder_label_ = -1;
    CoeffPtr coefficients_;
    CritPtr critical_points_;
};

// GenericShape is a flexible container for root/residual points. It allows
// appending points after construction and is suitable as a root shape.
class GenericShape : public Shape {
public:
    GenericShape() : points_(new pcl::PointCloud<PointT>()) {}
    GenericShape(const PointCloudPtr& pts) : points_(pts ? pts : PointCloudPtr(new pcl::PointCloud<PointT>())) {}

    std::string getType() const override { return "generic"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }

    void appendPoints(const PointCloudPtr& other) {
        if (!other || other->empty()) return;
        if (!points_) points_.reset(new pcl::PointCloud<PointT>());
        *points_ += *other;
    }

    void addPoint(const PointT& p) {
        if (!points_) points_.reset(new pcl::PointCloud<PointT>());
        points_->push_back(p);
    }

private:
    PointCloudPtr points_;
};
