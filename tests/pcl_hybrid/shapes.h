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
    // Child shapes (hierarchical decomposition)
    void addChild(std::shared_ptr<Shape> child) { children_.push_back(child); }
    const std::vector<std::shared_ptr<Shape>>& getChildren() const { return children_; }

protected:
    std::vector<std::shared_ptr<Shape>> children_;
};

// PlaneShape stores the cluster's points and the dominant plane label
class PlaneShape : public Shape {
public:
    PlaneShape(const PointCloudPtr& pts, int plane_label)
        : points_(pts), plane_label_(plane_label) {}

    std::string getType() const override { return "plane"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }
    int getPlaneLabel() const override { return plane_label_; }

private:
    PointCloudPtr points_;
    int plane_label_ = -1;
    std::vector<std::shared_ptr<Shape>> children_;
};

// CylinderShape stores cluster points; for this simplified version we don't fit parameters
class CylinderShape : public Shape {
public:
    CylinderShape(const PointCloudPtr& pts)
        : points_(pts) {}

    std::string getType() const override { return "cylinder"; }
    std::string toJSON() const override;
    PointCloudPtr getPoints() const override { return points_; }

private:
    PointCloudPtr points_;
};
