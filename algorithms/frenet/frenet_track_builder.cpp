#include "frenet_track_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>

namespace topology_map::algorithms {

FrenetTrackBuilder::FrenetTrackBuilder()
    : cfg_(Config{}) {}

FrenetTrackBuilder::FrenetTrackBuilder(Config config)
    : cfg_(std::move(config)) {}

FrenetTrackResult FrenetTrackBuilder::build(const BoundarySamplingResult& samples) const {
    FrenetTrackResult result;
    result.frame_id = samples.frame_id;
    if (!samples.ok) {
        result.error = samples.error.empty() ? "invalid_boundary_samples" : samples.error;
        return result;
    }

    std::map<std::string, FrenetTrack> by_id;
    for (std::size_t slice_index = 0; slice_index < samples.slices.size(); ++slice_index) {
        const auto& slice = samples.slices[slice_index];
        for (const auto& hit : slice.hits) {
            const std::string track_id = hit.track_line_id.empty() ? hit.source_line_id : hit.track_line_id;
            auto& track = by_id[track_id];
            track.id = track_id;
            track.source_type = hit.source_type;
            track.type = frenetBoundaryTypeFromString(hit.source_type);
            track.lane_position = hit.lane_position;
            track.lane_id = hit.lane_id;
            track.lane_index = hit.lane_index;
            track.samples.push_back({
                hit.s,
                hit.offset_m,
                hit.x,
                hit.y,
                static_cast<int>(slice_index),
                hit.section_index,
                hit.source_line_id,
                hit.lane_type,
                hit.lane_type_value,
                hit.confidence,
            });
        }
    }

    result.tracks.reserve(by_id.size());
    for (auto& [_, track] : by_id) {
        std::sort(track.samples.begin(), track.samples.end(), [](const auto& a, const auto& b) {
            return a.s < b.s;
        });
        double min_s = std::numeric_limits<double>::infinity();
        double max_s = -std::numeric_limits<double>::infinity();
        double min_l = std::numeric_limits<double>::infinity();
        double max_l = -std::numeric_limits<double>::infinity();
        double sum_l = 0.0;
        int gap_count = 0;
        std::set<std::string> lane_types;
        for (std::size_t i = 0; i < track.samples.size(); ++i) {
            const auto& sample = track.samples[i];
            min_s = std::min(min_s, sample.s);
            max_s = std::max(max_s, sample.s);
            min_l = std::min(min_l, sample.l);
            max_l = std::max(max_l, sample.l);
            sum_l += sample.l;
            if (!sample.lane_type.empty()) lane_types.insert(sample.lane_type);
            if (i > 0 && sample.s - track.samples[i - 1].s > cfg_.gap_threshold_m) {
                ++gap_count;
            }
        }
        const double count = static_cast<double>(track.samples.size());
        track.s_start = std::isfinite(min_s) ? min_s : 0.0;
        track.s_end = std::isfinite(max_s) ? max_s : 0.0;
        track.l_min = std::isfinite(min_l) ? min_l : 0.0;
        track.l_max = std::isfinite(max_l) ? max_l : 0.0;
        track.l_mean = count > 0.0 ? sum_l / count : 0.0;
        track.support_length_m = std::max(0.0, track.s_end - track.s_start);
        track.gap_count = gap_count;
        for (const auto& lane_type : lane_types) {
            if (!track.lane_type_summary.empty()) track.lane_type_summary += "|";
            track.lane_type_summary += lane_type;
        }
        track.label = "FT" + std::to_string(result.tracks.size());
        result.track_index_by_id[track.id] = result.tracks.size();
        result.tracks.push_back(std::move(track));
    }

    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
