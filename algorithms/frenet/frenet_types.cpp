#include "frenet_types.h"

namespace topology_map::algorithms {

FrenetBoundaryType frenetBoundaryTypeFromString(const std::string& value) {
    if (value == "lane_line" || value == "lane_boundary") return FrenetBoundaryType::kLaneLine;
    if (value == "road_edge" || value == "edge") return FrenetBoundaryType::kRoadEdge;
    if (value == "curb") return FrenetBoundaryType::kCurb;
    return FrenetBoundaryType::kUnknown;
}

std::string frenetBoundaryTypeToString(FrenetBoundaryType type) {
    switch (type) {
        case FrenetBoundaryType::kLaneLine:
            return "lane_line";
        case FrenetBoundaryType::kRoadEdge:
            return "road_edge";
        case FrenetBoundaryType::kCurb:
            return "curb";
        case FrenetBoundaryType::kUnknown:
        default:
            return "unknown";
    }
}

bool isLaneBoundaryType(FrenetBoundaryType type) {
    return type == FrenetBoundaryType::kLaneLine;
}

bool isStrongRoadBoundaryType(FrenetBoundaryType type) {
    return type == FrenetBoundaryType::kRoadEdge || type == FrenetBoundaryType::kCurb;
}

}  // namespace topology_map::algorithms
