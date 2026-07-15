#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "boundary_sampling/boundary_sampling_module.h"
#include "frenet_ribbon_builder.h"

namespace topology_map::algorithms {

struct FrenetJunctionCandidate {
    std::string id;
    std::string label;
    double s = 0.0;
    double l = 0.0;
    double x = 0.0;
    double y = 0.0;
    std::string right_line_id;
    std::string left_line_id;
    std::string right_track_id;
    std::string left_track_id;
    double distance_m = 0.0;
    double prev_distance_m = 0.0;
    double next_distance_m = 0.0;
    bool has_prev_distance = false;
    bool has_next_distance = false;
    std::string event_hint = "near_pair";
    std::string ribbon_id;
    std::string ribbon_label;
    double ribbon_start_s = 0.0;
    double ribbon_end_s = 0.0;
    double ribbon_width_start_m = 0.0;
    double ribbon_width_end_m = 0.0;
    double ribbon_width_slope_m_per_m = 0.0;
    int ribbon_sample_count = 0;
};

struct FrenetJunctionResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<FrenetJunctionCandidate> candidates;
};

class FrenetJunctionAnalyzer {
public:
    struct Config {
        double near_pair_distance_m = 0.65;
        double endpoint_tolerance_m = 3.0;
        double split_initial_width_max_m = 0.8;
        double split_final_width_min_m = 2.5;
        double split_width_slope_min_m_per_m = 0.03;
        double merge_initial_width_min_m = 2.5;
        double merge_final_width_max_m = 0.8;
        double merge_width_slope_max_m_per_m = -0.03;
        int min_ribbon_sample_count = 3;
    };

    FrenetJunctionAnalyzer();
    explicit FrenetJunctionAnalyzer(Config config);

    FrenetJunctionResult analyze(const BoundarySamplingResult& samples,
                                 const FrenetRibbonResult& ribbons) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
