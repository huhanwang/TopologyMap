#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "boundary_sampling/boundary_sampling_module.h"
#include "frenet_completed_boundary_builder.h"
#include "frenet_lane_line_completer.h"
#include "frenet_track_builder.h"

namespace topology_map::algorithms {

enum class FrenetLaneRegionBoundaryState {
    kObserved,
    kInferred,
    kConnected,
};

std::string frenetLaneRegionBoundaryStateToString(FrenetLaneRegionBoundaryState state);

struct FrenetLaneRegionSample {
    int slice_index = -1;
    double s = 0.0;
    double right_l = 0.0;
    double left_l = 0.0;
    double width_m = 0.0;
    double center_l_m = 0.0;
    FrenetLaneRegionBoundaryState right_state = FrenetLaneRegionBoundaryState::kObserved;
    FrenetLaneRegionBoundaryState left_state = FrenetLaneRegionBoundaryState::kObserved;
};

struct FrenetLaneRegionCandidate {
    std::string id;
    std::string label;
    std::string pair_id;
    int segment_index = 0;
    std::string right_track_id;
    std::string left_track_id;
    FrenetBoundaryType right_type = FrenetBoundaryType::kUnknown;
    FrenetBoundaryType left_type = FrenetBoundaryType::kUnknown;
    std::string right_source_type;
    std::string left_source_type;
    std::string right_lane_type;
    std::string left_lane_type;
    bool has_inferred_boundary = false;
    bool lane_line_pair = false;
    bool boundary_pair = false;
    bool lane_to_boundary_pair = false;
    bool candidate = false;
    std::string candidate_reason;
    std::string width_class;
    std::vector<FrenetLaneRegionSample> samples;
    double s_start = 0.0;
    double s_end = 0.0;
    double support_length_m = 0.0;
    double width_mean_m = 0.0;
    double width_median_m = 0.0;
    double width_min_m = 0.0;
    double width_max_m = 0.0;
    double width_std_m = 0.0;
    double inferred_sample_ratio = 0.0;
};

struct FrenetLaneRegionResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<FrenetLaneRegionCandidate> regions;
};

class FrenetLaneRegionBuilder {
public:
    struct Config {
        double gap_threshold_m = 6.5;
        double min_lane_width_m = 2.7;
        double max_lane_width_m = 4.8;
        double min_lane_line_region_support_m = 4.0;
        double min_boundary_region_support_m = 8.0;
        int min_sample_count = 2;
    };

    FrenetLaneRegionBuilder();
    explicit FrenetLaneRegionBuilder(Config config);

    FrenetLaneRegionResult build(const BoundarySamplingResult& samples,
                                 const FrenetCompletedBoundaryResult& completed) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
