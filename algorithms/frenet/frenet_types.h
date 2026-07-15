#pragma once

#include <string>

namespace topology_map::algorithms {

struct FrenetPoint {
    double s = 0.0;
    double l = 0.0;
    double x = 0.0;
    double y = 0.0;
};

struct FrenetPose {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double tangent_x = 1.0;
    double tangent_y = 0.0;
    double normal_x = 0.0;
    double normal_y = 1.0;
    double heading_rad = 0.0;
    double curvature_m_inv = 0.0;
};

enum class FrenetBoundaryType {
    kUnknown,
    kLaneLine,
    kRoadEdge,
    kCurb,
};

FrenetBoundaryType frenetBoundaryTypeFromString(const std::string& value);
std::string frenetBoundaryTypeToString(FrenetBoundaryType type);
bool isLaneBoundaryType(FrenetBoundaryType type);
bool isStrongRoadBoundaryType(FrenetBoundaryType type);

}  // namespace topology_map::algorithms
