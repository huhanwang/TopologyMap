#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "boundary_sampling/boundary_sampling_module.h"
#include "frenet_types.h"

namespace topology_map::algorithms {

struct FrenetTrackSample {
    double s = 0.0;
    double l = 0.0;
    double x = 0.0;
    double y = 0.0;
    int slice_index = -1;
    int section_index = -1;
    std::string source_line_id;
    std::string lane_type;
    int lane_type_value = 0;
    double confidence = 1.0;
};

struct FrenetTrack {
    std::string id;
    std::string label;
    FrenetBoundaryType type = FrenetBoundaryType::kUnknown;
    std::string source_type;
    std::string lane_type_summary;
    std::string lane_position;
    int lane_id = 0;
    int lane_index = -1;
    std::vector<FrenetTrackSample> samples;
    double s_start = 0.0;
    double s_end = 0.0;
    double l_min = 0.0;
    double l_max = 0.0;
    double l_mean = 0.0;
    double support_length_m = 0.0;
    int gap_count = 0;
};

struct FrenetTrackResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<FrenetTrack> tracks;
    std::unordered_map<std::string, std::size_t> track_index_by_id;
};

class FrenetTrackBuilder {
public:
    struct Config {
        double gap_threshold_m = 3.1;
    };

    FrenetTrackBuilder();
    explicit FrenetTrackBuilder(Config config);

    FrenetTrackResult build(const BoundarySamplingResult& samples) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
