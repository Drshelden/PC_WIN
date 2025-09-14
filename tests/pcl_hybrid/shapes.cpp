#include "shapes.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::string PlaneShape::toJSON() const {
    json j;
    j["type"] = getType();
    j["plane_label"] = plane_label_;
    j["points"] = json::array();
    for (const auto& pt : points_->points) {
        j["points"].push_back({pt.x, pt.y, pt.z});
    }
    return j.dump();
}

std::string CylinderShape::toJSON() const {
    json j;
    j["type"] = getType();
    j["points"] = json::array();
    for (const auto& pt : points_->points) {
        j["points"].push_back({pt.x, pt.y, pt.z});
    }
    return j.dump();
}
