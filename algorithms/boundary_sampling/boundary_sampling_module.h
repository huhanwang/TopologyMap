#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "fused_reference/fused_reference_module.h"
#include "interface/pilot/fused_static.pb.h"

namespace topology_map::algorithms {

struct BoundaryIntersectionHit {
    std::string id;
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double offset_m = 0.0;
    std::string source;
    std::string source_line_id;
    std::string track_line_id;
    int lane_id = 0;
    int lane_index = -1;
    int section_index = -1;
    std::string source_type;
    std::string lane_type;
    int lane_type_value = 0;
    std::string lane_position;
    double confidence = 1.0;
};

struct BoundaryIntersectionSlice {
    double s = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double normal_x = 0.0;
    double normal_y = 1.0;
    std::vector<BoundaryIntersectionHit> hits;
};

struct BoundarySamplingResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<BoundaryIntersectionSlice> slices;
    int source_section_count = 0;
    int hit_count = 0;
};

class BoundarySamplingModule {
public:
    struct Config {
        double sample_spacing_m = 2.0;
        double max_abs_offset_m = 35.0;
        double half_length_m = 40.0;
    };

    BoundarySamplingModule();
    explicit BoundarySamplingModule(Config config);

    BoundarySamplingResult process(
        std::int64_t frame_id,
        const FusedReferenceResult& reference,
        const idrive::workflow::proto::FusedStaticMsg& fused_static) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
