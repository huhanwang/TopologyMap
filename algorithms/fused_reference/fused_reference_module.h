#pragma once

#include <string>
#include <vector>

#include "navigation_route/navigation_route_tracker.h"
#include "visual_reference/visual_reference_module.h"

namespace topology_map::algorithms {

struct FusedReferencePoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double heading_rad = 0.0;
    double curvature_m_inv = 0.0;
    std::string source;
};

struct FusedReferenceResult {
    bool ok = false;
    std::string error;
    std::string method;
    std::vector<FusedReferencePoint> points;
    double confidence = 0.0;
    double lateral_offset_m = 0.0;
    double heading_error_rad = 0.0;
    double overlap_length_m = 0.0;
    double visual_end_x_m = 0.0;
    double fused_start_x_m = 0.0;
    double fused_end_x_m = 0.0;
    bool used_navigation = false;
};

class FusedReferenceModule {
public:
    struct Config {
        double sample_step_m = 2.0;
        double min_visual_length_m = 15.0;
        double min_navigation_overlap_m = 20.0;
        double max_lateral_offset_m = 12.0;
        double max_heading_error_rad = 0.35;
        double max_extension_m = 150.0;
        double max_heading_delta_from_visual_rad = 0.9;
        double max_heading_step_rad = 0.03;
        double navigation_trend_ramp_m = 55.0;
    };

    FusedReferenceModule();
    explicit FusedReferenceModule(Config config);

    FusedReferenceResult process(
        const VisualReferenceResult& visual,
        const NavigationReferenceResult& navigation) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
