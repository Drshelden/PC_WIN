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

// CylinderShape stores the cluster's points and the dominant plane label
class CylinderShape : public Shape {
public:
    CylinderShape(const PointCloudPtr& pts, int cylinder_label = -1)
        : points_(pts), cylinder_label_(cylinder_label) {}

    std::string getType() const override { return "cylinder"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }
    int getCylinderLabel() const override { return cylinder_label_; }

private:
    PointCloudPtr points_;
    int cylinder_label_ = -1;
};

// PlaneShape stores cluster points; for this simplified version we don't fit parameters
class PlaneShape : public Shape {
public:
    PlaneShape(const PointCloudPtr& pts, int plane_label = -1)
        : points_(pts), plane_label_(plane_label) {}

    std::string getType() const override { return "plane"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }
    int getPlaneLabel() const { return plane_label_; }

private:
    PointCloudPtr points_;
    int plane_label_ = -1;
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
