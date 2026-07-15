#include "frenet_ribbon_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>

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
        const double delta = value - mean;
        sum += delta * delta;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

FrenetRibbonKind classifyRibbon(FrenetBoundaryType right, FrenetBoundaryType left) {
    const bool right_lane = isLaneBoundaryType(right);
    const bool left_lane = isLaneBoundaryType(left);
    if (right_lane && left_lane) return FrenetRibbonKind::kTopologyLane;
    if (right_lane || left_lane) return FrenetRibbonKind::kLaneToBoundaryRef;
    return FrenetRibbonKind::kBoundaryOnlyRef;
}

}  // namespace

std::string frenetRibbonKindToString(FrenetRibbonKind kind) {
    switch (kind) {
        case FrenetRibbonKind::kTopologyLane:
            return "topology_lane";
        case FrenetRibbonKind::kLaneToBoundaryRef:
            return "lane_to_boundary_ref";
        case FrenetRibbonKind::kBoundaryOnlyRef:
        default:
            return "boundary_only_ref";
    }
}

const FrenetRibbonTrack* FrenetRibbonResult::ribbonById(const std::string& ribbon_id) const {
    auto it = ribbon_index_by_id.find(ribbon_id);
    if (it == ribbon_index_by_id.end() || it->second >= ribbons.size()) return nullptr;
    return &ribbons[it->second];
}

const TrackRibbonRefs* FrenetRibbonResult::ribbonsOfTrack(const std::string& track_id) const {
    auto it = ribbons_by_track_id.find(track_id);
    if (it == ribbons_by_track_id.end()) return nullptr;
    return &it->second;
}

std::vector<RibbonSampleRef> FrenetRibbonResult::ribbonsOfTrackAtSlice(
    const std::string& track_id,
    int slice_index) const {
    std::vector<RibbonSampleRef> refs;
    if (slice_index < 0 || static_cast<std::size_t>(slice_index) >= ribbons_by_slice.size()) {
        return refs;
    }
    for (const auto& ref : ribbons_by_slice[static_cast<std::size_t>(slice_index)]) {
        if (ref.self_track_id == track_id) refs.push_back(ref);
    }
    return refs;
}

std::vector<const FrenetRibbonTrack*> FrenetRibbonResult::topologyRibbonsAtEndpoint(
    const FrenetTrack& track,
    FrenetRibbonEndpointDirection direction,
    double endpoint_tolerance_m) const {
    std::vector<const FrenetRibbonTrack*> result;
    if (track.samples.empty()) return result;
    const double endpoint_s = direction == FrenetRibbonEndpointDirection::kAfterTrackTail
        ? track.samples.back().s
        : track.samples.front().s;

    const auto* refs = ribbonsOfTrack(track.id);
    if (!refs) return result;
    std::set<std::string> visited;
    auto collect = [&](const std::vector<RibbonSampleRef>& side_refs) {
        for (const auto& ref : side_refs) {
            if (!visited.insert(ref.ribbon_id).second) continue;
            const auto* ribbon = ribbonById(ref.ribbon_id);
            if (!ribbon || ribbon->kind != FrenetRibbonKind::kTopologyLane || ribbon->samples.empty()) continue;
            if (ribbon->left_track_id != track.id && ribbon->right_track_id != track.id) continue;
            const double ribbon_endpoint_s = direction == FrenetRibbonEndpointDirection::kAfterTrackTail
                ? ribbon->samples.back().s
                : ribbon->samples.front().s;
            if (std::abs(ribbon_endpoint_s - endpoint_s) <= endpoint_tolerance_m) {
                result.push_back(ribbon);
            }
        }
    };
    collect(refs->left_side);
    collect(refs->right_side);
    return result;
}

RibbonStats FrenetRibbonResult::computeStats(
    const std::string& ribbon_id,
    const RibbonStatsOptions& options) const {
    RibbonStats stats;
    const auto* ribbon = ribbonById(ribbon_id);
    if (!ribbon) return stats;

    std::vector<double> widths;
    std::vector<int> sample_indices;
    for (std::size_t i = 0; i < ribbon->samples.size(); ++i) {
        const auto& sample = ribbon->samples[i];
        if (sample.s < options.s_min || sample.s > options.s_max) continue;
        widths.push_back(sample.width_m);
        sample_indices.push_back(static_cast<int>(i));
    }
    stats.sample_count = static_cast<int>(widths.size());
    if (widths.empty()) return stats;

    stats.width_median = median(widths);
    std::vector<double> used = widths;
    if (options.reject_outliers && widths.size() >= 4) {
        std::vector<double> abs_dev;
        abs_dev.reserve(widths.size());
        for (double width : widths) abs_dev.push_back(std::abs(width - stats.width_median));
        const double mad = median(abs_dev);
        const double threshold = std::max(options.min_abs_outlier_threshold_m, options.mad_scale * 1.4826 * mad);
        used.clear();
        for (std::size_t i = 0; i < widths.size(); ++i) {
            if (std::abs(widths[i] - stats.width_median) <= threshold) {
                used.push_back(widths[i]);
            } else {
                stats.outlier_sample_indices.push_back(sample_indices[i]);
            }
        }
        if (used.empty()) used = widths;
    }

    stats.used_sample_count = static_cast<int>(used.size());
    stats.width_mean = std::accumulate(widths.begin(), widths.end(), 0.0) /
        static_cast<double>(widths.size());
    stats.width_trimmed_mean = std::accumulate(used.begin(), used.end(), 0.0) /
        static_cast<double>(used.size());
    stats.width_std = stddev(used, stats.width_trimmed_mean);
    return stats;
}

std::optional<RibbonWidthEstimate> FrenetRibbonResult::estimateWidth(
    const std::string& ribbon_id,
    double s,
    const RibbonStatsOptions& options) const {
    const auto* ribbon = ribbonById(ribbon_id);
    if (!ribbon) return std::nullopt;
    const FrenetRibbonSample* nearest = nullptr;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto& sample : ribbon->samples) {
        if (sample.s < options.s_min || sample.s > options.s_max) continue;
        const double distance = std::abs(sample.s - s);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = &sample;
        }
    }
    if (nearest) {
        RibbonWidthEstimate estimate;
        estimate.width_m = nearest->width_m;
        estimate.confidence = std::max(0.2, 1.0 - 0.03 * nearest_distance);
        estimate.method = "nearest_sample";
        estimate.support_s = nearest->s;
        estimate.support_distance_m = nearest_distance;
        return estimate;
    }

    const auto stats = computeStats(ribbon_id, options);
    if (stats.used_sample_count <= 0) return std::nullopt;
    RibbonWidthEstimate estimate;
    estimate.width_m = stats.width_trimmed_mean;
    estimate.confidence = std::min(1.0, 0.35 + 0.04 * static_cast<double>(stats.used_sample_count));
    estimate.method = "trimmed_mean";
    return estimate;
}

RibbonTrend FrenetRibbonResult::computeTrend(
    const std::string& ribbon_id,
    const RibbonTrendOptions& options) const {
    RibbonTrend trend;
    const auto* ribbon = ribbonById(ribbon_id);
    if (!ribbon) return trend;

    std::vector<const FrenetRibbonSample*> samples;
    for (const auto& sample : ribbon->samples) {
        if (sample.s < options.s_min || sample.s > options.s_max) continue;
        samples.push_back(&sample);
    }
    trend.sample_count = static_cast<int>(samples.size());
    if (samples.size() < 2) return trend;

    const double s0 = samples.front()->s;
    double sum_x = 0.0;
    double sum_w = 0.0;
    double sum_c = 0.0;
    for (const auto* sample : samples) {
        const double x = sample->s - s0;
        sum_x += x;
        sum_w += sample->width_m;
        sum_c += sample->center_l_m;
    }
    const double n = static_cast<double>(samples.size());
    const double mean_x = sum_x / n;
    const double mean_w = sum_w / n;
    const double mean_c = sum_c / n;
    double var_x = 0.0;
    double cov_w = 0.0;
    double cov_c = 0.0;
    for (const auto* sample : samples) {
        const double x = sample->s - s0;
        const double dx = x - mean_x;
        var_x += dx * dx;
        cov_w += dx * (sample->width_m - mean_w);
        cov_c += dx * (sample->center_l_m - mean_c);
    }
    if (var_x > 1e-9) {
        trend.width_slope_m_per_m = cov_w / var_x;
        trend.center_slope_m_per_m = cov_c / var_x;
    }
    if (std::abs(trend.width_slope_m_per_m) <= options.stable_width_slope_threshold) {
        trend.width_trend = "stable";
    } else if (trend.width_slope_m_per_m > 0.0) {
        trend.width_trend = "widening";
    } else {
        trend.width_trend = "narrowing";
    }
    if (std::abs(trend.center_slope_m_per_m) <= options.stable_center_slope_threshold) {
        trend.center_trend = "stable";
    } else if (trend.center_slope_m_per_m > 0.0) {
        trend.center_trend = "shift_left";
    } else {
        trend.center_trend = "shift_right";
    }
    return trend;
}

FrenetRibbonBuilder::FrenetRibbonBuilder()
    : cfg_(Config{}) {}

FrenetRibbonBuilder::FrenetRibbonBuilder(Config config)
    : cfg_(std::move(config)) {}

FrenetRibbonResult FrenetRibbonBuilder::build(
    const BoundarySamplingResult& samples,
    const FrenetTrackResult& tracks) const {
    FrenetRibbonResult result;
    result.frame_id = samples.frame_id;
    if (!samples.ok || !tracks.ok) {
        result.error = !samples.ok ? samples.error : tracks.error;
        if (result.error.empty()) result.error = "invalid_frenet_ribbon_input";
        return result;
    }
    result.ribbons_by_slice.resize(samples.slices.size());

    struct Accum {
        FrenetRibbonTrack track;
        double last_s = 0.0;
    };
    std::map<std::string, std::vector<Accum>> grouped;

    auto trackType = [&](const std::string& id) {
        auto it = tracks.track_index_by_id.find(id);
        if (it == tracks.track_index_by_id.end()) return FrenetBoundaryType::kUnknown;
        return tracks.tracks[it->second].type;
    };
    auto trackIdForHit = [](const BoundaryIntersectionHit& hit) {
        return hit.track_line_id.empty() ? hit.source_line_id : hit.track_line_id;
    };

    for (std::size_t slice_index = 0; slice_index < samples.slices.size(); ++slice_index) {
        const auto& slice = samples.slices[slice_index];
        if (slice.hits.size() < 2) continue;
        for (std::size_t i = 1; i < slice.hits.size(); ++i) {
            const auto& right = slice.hits[i - 1];
            const auto& left = slice.hits[i];
            const std::string right_track_id = trackIdForHit(right);
            const std::string left_track_id = trackIdForHit(left);
            const std::string pair_id = right_track_id + " -> " + left_track_id;
            auto& segments = grouped[pair_id];
            if (segments.empty() || slice.s - segments.back().last_s > cfg_.gap_threshold_m) {
                Accum next;
                next.track.pair_id = pair_id;
                next.track.segment_index = static_cast<int>(segments.size());
                next.track.right_track_id = right_track_id;
                next.track.left_track_id = left_track_id;
                next.track.right_type = trackType(right_track_id);
                next.track.left_type = trackType(left_track_id);
                next.track.right_source_type = right.source_type;
                next.track.left_source_type = left.source_type;
                next.track.kind = classifyRibbon(next.track.right_type, next.track.left_type);
                segments.push_back(std::move(next));
            }
            auto& accum = segments.back();
            accum.last_s = slice.s;
            accum.track.samples.push_back({
                static_cast<int>(slice_index),
                slice.s,
                right_track_id,
                left_track_id,
                right.offset_m,
                left.offset_m,
                left.offset_m - right.offset_m,
                0.5 * (left.offset_m + right.offset_m),
            });
        }
    }

    int label_index = 0;
    for (auto& [_, segments] : grouped) {
        for (auto& accum : segments) {
            auto& ribbon = accum.track;
            ribbon.id = ribbon.pair_id + "#" + std::to_string(ribbon.segment_index);
            ribbon.label = "RB" + std::to_string(label_index++);
            const int ribbon_index = static_cast<int>(result.ribbons.size());
            result.ribbon_index_by_id[ribbon.id] = result.ribbons.size();

            for (std::size_t sample_index = 0; sample_index < ribbon.samples.size(); ++sample_index) {
                const auto& sample = ribbon.samples[sample_index];
                RibbonSampleRef right_ref;
                right_ref.ribbon_id = ribbon.id;
                right_ref.ribbon_index = ribbon_index;
                right_ref.ribbon_sample_index = static_cast<int>(sample_index);
                right_ref.slice_index = sample.slice_index;
                right_ref.s = sample.s;
                right_ref.self_track_id = sample.right_track_id;
                right_ref.neighbor_track_id = sample.left_track_id;
                right_ref.self_is_right_boundary = true;

                RibbonSampleRef left_ref = right_ref;
                left_ref.self_track_id = sample.left_track_id;
                left_ref.neighbor_track_id = sample.right_track_id;
                left_ref.self_is_right_boundary = false;
                left_ref.self_is_left_boundary = true;

                result.ribbons_by_track_id[sample.right_track_id].left_side.push_back(right_ref);
                result.ribbons_by_track_id[sample.left_track_id].right_side.push_back(left_ref);
                if (sample.slice_index >= 0 &&
                    static_cast<std::size_t>(sample.slice_index) < result.ribbons_by_slice.size()) {
                    result.ribbons_by_slice[static_cast<std::size_t>(sample.slice_index)].push_back(right_ref);
                    result.ribbons_by_slice[static_cast<std::size_t>(sample.slice_index)].push_back(left_ref);
                }
            }

            result.ribbons.push_back(std::move(ribbon));
        }
    }

    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
