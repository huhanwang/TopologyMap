#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "boundary_sampling/boundary_sampling_module.h"
#include "frenet_ribbon_builder.h"
#include "frenet_track_builder.h"

namespace topology_map::algorithms {

struct FrenetInferredNode {
    std::string id;
    std::string label;
    std::string track_id;
    int slice_index = -1;
    double s = 0.0;
    double l = 0.0;
    double x = 0.0;
    double y = 0.0;
    std::string source_type = "lane_line";
    std::string lane_type;
    int lane_type_value = 0;
    std::string method;
    double confidence = 0.0;
    std::string anchor_track_id;
    std::string left_anchor_track_id;
    std::string right_anchor_track_id;
    std::string ribbon_id;
    std::string left_ribbon_id;
    std::string right_ribbon_id;
    double estimated_width_m = 0.0;
    double width_support_s = 0.0;
    double width_support_distance_m = 0.0;
    double left_width_m = 0.0;
    double right_width_m = 0.0;
    double estimated_ratio = 0.0;
};

struct FrenetCompletionLink {
    std::string id;
    std::string label;
    std::string track_id;
    std::string from_kind;
    std::string to_kind;
    std::string from_node_id;
    std::string to_node_id;
    double from_s = 0.0;
    double from_l = 0.0;
    double from_x = 0.0;
    double from_y = 0.0;
    double to_s = 0.0;
    double to_l = 0.0;
    double to_x = 0.0;
    double to_y = 0.0;
    std::string method;
};

struct FrenetCompletionResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<FrenetInferredNode> inferred_nodes;
    std::vector<FrenetInferredNode> stop_nodes;
    std::vector<FrenetCompletionLink> links;
    std::unordered_map<std::string, std::vector<int>> inferred_by_track_id;
    std::vector<std::vector<int>> inferred_by_slice;
};

class FrenetLaneLineCompleter {
public:
    struct Config {
        int min_track_sample_count = 3;
        double min_track_support_length_m = 6.0;
        int max_iterations = 4;
        bool use_inferred_as_anchor = true;
        double min_width_m = 1.0;
        double max_width_m = 6.5;
        double min_lane_ribbon_width_m = 2.7;
        double max_lane_ribbon_width_m = 4.8;
        int min_lane_ribbon_sample_count = 3;
        double short_track_support_length_m = 16.0;
        double max_bilateral_residual_m = 1.2;
        double near_existing_lane_stop_l_m = 0.65;
        double merge_near_inferred_l_m = 0.35;
        double max_entity_link_gap_m = 40.0;
        double max_shared_anchor_entity_link_gap_m = 80.0;
        double max_entity_link_l_delta_m = 1.25;
        double endpoint_ribbon_tolerance_m = 3.2;
        int track_prediction_sample_count = 6;
        double track_prediction_max_extrapolation_m = 35.0;
        double track_prediction_base_weight = 0.75;
        double track_prediction_observation_residual_scale_m = 0.9;
        double endpoint_observation_base_weight = 1.0;
        double weak_endpoint_observation_weight = 0.45;
        double ribbon_width_slope_weight_scale = 0.08;
        double ribbon_center_slope_weight_scale = 0.20;
        double near_inferred_lane_stop_l_m = 0.55;
    };

    FrenetLaneLineCompleter();
    explicit FrenetLaneLineCompleter(Config config);

    FrenetCompletionResult complete(const BoundarySamplingResult& raw_samples,
                                    const FrenetTrackResult& tracks,
                                    const FrenetRibbonResult& ribbons) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
