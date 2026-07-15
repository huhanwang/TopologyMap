#include "frenet_lane_region_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <utility>

namespace topology_map::algorithms {
namespace {

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if (values.size() % 2 == 1) return values[mid];
    return 0.5 * (values[mid - 1] + values[mid]);
}

double stddev(const std::vector<double>& values, double mean) {
    if (values.size() < 2) return 0.0;
    double sum = 0.0;
    for (double value : values) {
        const double d = value - mean;
        sum += d * d;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

std::string widthClass(double width_m) {
    if (width_m < 0.3) return "invalid";
    if (width_m < 2.2) return "shoulder";
    if (width_m < 2.7) return "narrow";
    if (width_m <= 4.8) return "lane";
    if (width_m <= 6.5) return "wide";
    return "invalid";
}

FrenetLaneRegionBoundaryState regionState(FrenetCompletedBoundaryState state) {
    switch (state) {
        case FrenetCompletedBoundaryState::kObserved:
            return FrenetLaneRegionBoundaryState::kObserved;
        case FrenetCompletedBoundaryState::kInferred:
            return FrenetLaneRegionBoundaryState::kInferred;
        case FrenetCompletedBoundaryState::kConnected:
            return FrenetLaneRegionBoundaryState::kConnected;
    }
    return FrenetLaneRegionBoundaryState::kObserved;
}

}  // namespace

std::string frenetLaneRegionBoundaryStateToString(FrenetLaneRegionBoundaryState state) {
    switch (state) {
        case FrenetLaneRegionBoundaryState::kObserved:
            return "observed";
        case FrenetLaneRegionBoundaryState::kInferred:
            return "inferred";
        case FrenetLaneRegionBoundaryState::kConnected:
            return "connected";
    }
    return "observed";
}

FrenetLaneRegionBuilder::FrenetLaneRegionBuilder()
    : cfg_(Config{}) {}

FrenetLaneRegionBuilder::FrenetLaneRegionBuilder(Config config)
    : cfg_(std::move(config)) {}

FrenetLaneRegionResult FrenetLaneRegionBuilder::build(
    const BoundarySamplingResult& samples,
    const FrenetCompletedBoundaryResult& completed) const {
    FrenetLaneRegionResult result;
    result.frame_id = samples.frame_id;
    if (!samples.ok || !completed.ok) {
        result.error = !samples.ok ? samples.error : completed.error;
        if (result.error.empty()) result.error = "invalid_frenet_lane_region_input";
        return result;
    }

    struct Accum {
        FrenetLaneRegionCandidate region;
        double last_s = 0.0;
    };
    std::map<std::string, std::vector<Accum>> grouped;

    for (std::size_t slice_index = 0; slice_index < completed.nodes_by_slice.size(); ++slice_index) {
        const auto& nodes = completed.nodes_by_slice[slice_index];
        for (std::size_t i = 1; i < nodes.size(); ++i) {
            const auto& right = nodes[i - 1];
            const auto& left = nodes[i];
            if (right.completed_track_id == left.completed_track_id) continue;
            const double width = left.l - right.l;
            if (width <= 0.0 || width > 8.0) continue;
            const std::string pair_id = right.completed_track_id + " -> " + left.completed_track_id;
            auto& segments = grouped[pair_id];
            if (segments.empty() || right.s - segments.back().last_s > cfg_.gap_threshold_m) {
                Accum next;
                next.region.pair_id = pair_id;
                next.region.segment_index = static_cast<int>(segments.size());
                next.region.right_track_id = right.completed_track_id;
                next.region.left_track_id = left.completed_track_id;
                next.region.right_type = right.type;
                next.region.left_type = left.type;
                next.region.right_source_type = right.source_type;
                next.region.left_source_type = left.source_type;
                next.region.right_lane_type = right.lane_type;
                next.region.left_lane_type = left.lane_type;
                next.region.lane_line_pair = isLaneBoundaryType(right.type) && isLaneBoundaryType(left.type);
                next.region.boundary_pair = isStrongRoadBoundaryType(right.type) || isStrongRoadBoundaryType(left.type);
                next.region.lane_to_boundary_pair =
                    (isLaneBoundaryType(right.type) && isStrongRoadBoundaryType(left.type)) ||
                    (isStrongRoadBoundaryType(right.type) && isLaneBoundaryType(left.type));
                segments.push_back(std::move(next));
            }
            auto& accum = segments.back();
            accum.last_s = right.s;
            FrenetLaneRegionSample sample;
            sample.slice_index = static_cast<int>(slice_index);
            sample.s = right.s;
            sample.right_l = right.l;
            sample.left_l = left.l;
            sample.width_m = width;
            sample.center_l_m = 0.5 * (right.l + left.l);
            sample.right_state = regionState(right.state);
            sample.left_state = regionState(left.state);
            accum.region.samples.push_back(std::move(sample));
            if (right.state != FrenetCompletedBoundaryState::kObserved ||
                left.state != FrenetCompletedBoundaryState::kObserved) {
                accum.region.has_inferred_boundary = true;
            }
        }
    }

    int label_index = 0;
    for (auto& [_, segments] : grouped) {
        for (auto& accum : segments) {
            auto& region = accum.region;
            if (region.samples.empty()) continue;
            std::vector<double> widths;
            widths.reserve(region.samples.size());
            int inferred_count = 0;
            for (const auto& sample : region.samples) {
                widths.push_back(sample.width_m);
                if (sample.right_state != FrenetLaneRegionBoundaryState::kObserved ||
                    sample.left_state != FrenetLaneRegionBoundaryState::kObserved) {
                    ++inferred_count;
                }
            }
            region.s_start = region.samples.front().s;
            region.s_end = region.samples.back().s;
            region.support_length_m = std::max(0.0, region.s_end - region.s_start);
            region.width_mean_m = std::accumulate(widths.begin(), widths.end(), 0.0) /
                static_cast<double>(widths.size());
            region.width_median_m = median(widths);
            region.width_min_m = *std::min_element(widths.begin(), widths.end());
            region.width_max_m = *std::max_element(widths.begin(), widths.end());
            region.width_std_m = stddev(widths, region.width_mean_m);
            region.width_class = widthClass(region.width_median_m);
            region.inferred_sample_ratio = static_cast<double>(inferred_count) /
                static_cast<double>(region.samples.size());

            const bool enough_samples = static_cast<int>(region.samples.size()) >= cfg_.min_sample_count;
            if (region.lane_line_pair) {
                region.candidate = enough_samples &&
                    region.support_length_m >= cfg_.min_lane_line_region_support_m &&
                    region.width_median_m >= cfg_.min_lane_width_m &&
                    region.width_median_m <= cfg_.max_lane_width_m;
                region.candidate_reason = region.candidate ? "lane_line_pair_width" : "lane_line_pair_rejected";
            } else if (region.lane_to_boundary_pair) {
                region.candidate = enough_samples &&
                    region.support_length_m >= cfg_.min_boundary_region_support_m &&
                    region.width_median_m >= cfg_.min_lane_width_m &&
                    region.width_median_m <= cfg_.max_lane_width_m &&
                    region.width_std_m <= 0.6;
                region.candidate_reason = region.candidate ? "lane_to_boundary_strict_width" : "lane_to_boundary_rejected";
            } else {
                region.candidate = false;
                region.candidate_reason = "boundary_only_or_unknown";
            }

            region.id = region.pair_id + "#" + std::to_string(region.segment_index);
            region.label = "LR" + std::to_string(label_index++);
            result.regions.push_back(std::move(region));
        }
    }

    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
