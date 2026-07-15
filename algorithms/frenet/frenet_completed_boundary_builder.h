#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "boundary_sampling/boundary_sampling_module.h"
#include "frenet_lane_line_completer.h"
#include "frenet_track_builder.h"

namespace topology_map::algorithms {

enum class FrenetCompletedBoundaryState {
    kObserved,
    kInferred,
    kConnected,
};

std::string frenetCompletedBoundaryStateToString(FrenetCompletedBoundaryState state);

struct FrenetCompletedBoundaryNode {
    std::string completed_track_id;
    std::string source_track_id;
    FrenetBoundaryType type = FrenetBoundaryType::kUnknown;
    std::string source_type;
    std::string lane_type;
    int slice_index = -1;
    double s = 0.0;
    double l = 0.0;
    FrenetCompletedBoundaryState state = FrenetCompletedBoundaryState::kObserved;
    double confidence = 1.0;
};

struct FrenetCompletedBoundaryTrack {
    std::string id;
    std::vector<std::string> source_track_ids;
    FrenetBoundaryType type = FrenetBoundaryType::kUnknown;
    std::string source_type;
    std::vector<FrenetCompletedBoundaryNode> nodes;
};

struct FrenetCompletedBoundaryConnector {
    std::string from_track_id;
    std::string to_track_id;
    double from_s = 0.0;
    double from_l = 0.0;
    double to_s = 0.0;
    double to_l = 0.0;
};

struct FrenetCompletedBoundaryResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<FrenetCompletedBoundaryTrack> tracks;
    std::vector<FrenetCompletedBoundaryConnector> connectors;
    std::vector<std::vector<FrenetCompletedBoundaryNode>> nodes_by_slice;
    std::unordered_map<std::string, std::string> completed_id_by_source_track_id;
};

class FrenetCompletedBoundaryBuilder {
public:
    FrenetCompletedBoundaryResult build(const BoundarySamplingResult& samples,
                                        const FrenetTrackResult& tracks,
                                        const FrenetCompletionResult& completion) const;
};

}  // namespace topology_map::algorithms
