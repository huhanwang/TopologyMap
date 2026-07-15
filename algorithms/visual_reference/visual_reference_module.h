#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "interface/pilot/fused_static.pb.h"

namespace topology_map::algorithms {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct VisualReferencePoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
};

struct VisualReferenceResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::string selected_source;
    std::string method;
    std::string left_line_id;
    std::string right_line_id;
    double confidence = 0.0;
    std::pair<double, double> s_range_m{0.0, 0.0};
    std::vector<double> center_coeffs;
    std::vector<VisualReferencePoint> points;
    int input_line_count = 0;
    int selected_source_line_count = 0;
    std::string left_lane_position;
    std::string right_lane_position;
    std::string left_source_type;
    std::string right_source_type;
    double left_line_x_span_m = 0.0;
    double right_line_x_span_m = 0.0;
    double left_line_length_m = 0.0;
    double right_line_length_m = 0.0;
};

class VisualReferenceModule {
public:
    struct Config {
        std::vector<std::string> preferred_sources{"smooth_bev_lanes", "vehicle_bev_lanes"};
        double slice_spacing_m = 2.0;
        double min_line_x_span_m = 20.0;
        double min_reference_length_m = 20.0;
        double max_reference_extension_m = 5.0;
        double max_length_m = 120.0;
        double max_abs_d_m = 35.0;
    };

    VisualReferenceResult process(
        std::int64_t frame_id,
        const idrive::workflow::proto::FusedStaticMsg& fused_static) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
