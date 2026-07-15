#include "frenet_junction_analyzer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace topology_map::algorithms {
namespace {

std::string trackIdForHit(const BoundaryIntersectionHit& hit) {
    return hit.track_line_id.empty() ? hit.source_line_id : hit.track_line_id;
}

bool sameTrackPair(const FrenetRibbonTrack& ribbon,
                   const std::string& right_track_id,
                   const std::string& left_track_id) {
    return (ribbon.right_track_id == right_track_id && ribbon.left_track_id == left_track_id) ||
           (ribbon.right_track_id == left_track_id && ribbon.left_track_id == right_track_id);
}

double sampleWidthAtOrNear(const FrenetRibbonTrack& ribbon, double s) {
    if (ribbon.samples.empty()) return 0.0;
    const auto* best = &ribbon.samples.front();
    double best_distance = std::abs(best->s - s);
    for (const auto& sample : ribbon.samples) {
        const double distance = std::abs(sample.s - s);
        if (distance < best_distance) {
            best = &sample;
            best_distance = distance;
        }
    }
    return best->width_m;
}

const BoundaryIntersectionHit* hitForTrackAtSlice(const BoundaryIntersectionSlice& slice,
                                                  const std::string& track_id) {
    for (const auto& hit : slice.hits) {
        if (trackIdForHit(hit) == track_id) return &hit;
    }
    return nullptr;
}

bool hasCandidateForRibbonEvent(const std::vector<FrenetJunctionCandidate>& candidates,
                                const std::string& ribbon_id,
                                const std::string& event_hint) {
    for (const auto& candidate : candidates) {
        if (candidate.ribbon_id == ribbon_id && candidate.event_hint == event_hint) return true;
    }
    return false;
}

}  // namespace

FrenetJunctionAnalyzer::FrenetJunctionAnalyzer()
    : cfg_(Config{}) {}

FrenetJunctionAnalyzer::FrenetJunctionAnalyzer(Config config)
    : cfg_(std::move(config)) {}

FrenetJunctionResult FrenetJunctionAnalyzer::analyze(
    const BoundarySamplingResult& samples,
    const FrenetRibbonResult& ribbons) const {
    FrenetJunctionResult result;
    result.frame_id = samples.frame_id;
    if (!samples.ok || !ribbons.ok) {
        result.error = !samples.ok ? samples.error : ribbons.error;
        if (result.error.empty()) result.error = "invalid_frenet_junction_input";
        return result;
    }

    std::map<std::string, const FrenetRibbonTrack*> topology_ribbon_by_pair;
    for (const auto& ribbon : ribbons.ribbons) {
        if (ribbon.kind != FrenetRibbonKind::kTopologyLane || ribbon.samples.empty()) continue;
        const std::string key = ribbon.right_track_id < ribbon.left_track_id
            ? ribbon.right_track_id + "\n" + ribbon.left_track_id
            : ribbon.left_track_id + "\n" + ribbon.right_track_id;
        auto it = topology_ribbon_by_pair.find(key);
        if (it == topology_ribbon_by_pair.end() ||
            ribbon.samples.size() > it->second->samples.size()) {
            topology_ribbon_by_pair[key] = &ribbon;
        }
    }

    auto ribbonForPair = [&](const std::string& right_track_id,
                             const std::string& left_track_id) -> const FrenetRibbonTrack* {
        const std::string key = right_track_id < left_track_id
            ? right_track_id + "\n" + left_track_id
            : left_track_id + "\n" + right_track_id;
        auto it = topology_ribbon_by_pair.find(key);
        if (it == topology_ribbon_by_pair.end()) return nullptr;
        if (!sameTrackPair(*it->second, right_track_id, left_track_id)) return nullptr;
        return it->second;
    };

    for (std::size_t slice_index = 0; slice_index < samples.slices.size(); ++slice_index) {
        const auto& slice = samples.slices[slice_index];
        if (slice.hits.size() < 2) continue;
        const int current_slice_index = static_cast<int>(slice_index);
        for (std::size_t i = 1; i < slice.hits.size(); ++i) {
            const auto& right = slice.hits[i - 1];
            const auto& left = slice.hits[i];
            if (!isLaneBoundaryType(frenetBoundaryTypeFromString(right.source_type)) ||
                !isLaneBoundaryType(frenetBoundaryTypeFromString(left.source_type))) {
                continue;
            }
            const std::string right_track_id = trackIdForHit(right);
            const std::string left_track_id = trackIdForHit(left);
            if (right_track_id == left_track_id) continue;

            const double distance = left.offset_m - right.offset_m;
            if (distance < 0.0 || distance > cfg_.near_pair_distance_m) continue;

            FrenetJunctionCandidate candidate;
            candidate.s = slice.s;
            candidate.l = 0.5 * (right.offset_m + left.offset_m);
            candidate.x = 0.5 * (right.x + left.x);
            candidate.y = 0.5 * (right.y + left.y);
            candidate.right_line_id = right.source_line_id;
            candidate.left_line_id = left.source_line_id;
            candidate.right_track_id = right_track_id;
            candidate.left_track_id = left_track_id;
            candidate.distance_m = distance;

            if (current_slice_index > 0) {
                const auto& prev_slice = samples.slices[static_cast<std::size_t>(current_slice_index - 1)];
                const auto* prev_right = hitForTrackAtSlice(prev_slice, right_track_id);
                const auto* prev_left = hitForTrackAtSlice(prev_slice, left_track_id);
                if (prev_right && prev_left) {
                    candidate.prev_distance_m = std::abs(prev_left->offset_m - prev_right->offset_m);
                    candidate.has_prev_distance = true;
                }
            }
            if (current_slice_index + 1 < static_cast<int>(samples.slices.size())) {
                const auto& next_slice = samples.slices[static_cast<std::size_t>(current_slice_index + 1)];
                const auto* next_right = hitForTrackAtSlice(next_slice, right_track_id);
                const auto* next_left = hitForTrackAtSlice(next_slice, left_track_id);
                if (next_right && next_left) {
                    candidate.next_distance_m = std::abs(next_left->offset_m - next_right->offset_m);
                    candidate.has_next_distance = true;
                }
            }

            const auto* ribbon = ribbonForPair(right_track_id, left_track_id);
            if (ribbon) {
                const auto& first = ribbon->samples.front();
                const auto& last = ribbon->samples.back();
                const auto trend = ribbons.computeTrend(ribbon->id);
                candidate.ribbon_id = ribbon->id;
                candidate.ribbon_label = ribbon->label;
                candidate.ribbon_start_s = first.s;
                candidate.ribbon_end_s = last.s;
                candidate.ribbon_width_start_m = first.width_m;
                candidate.ribbon_width_end_m = last.width_m;
                candidate.ribbon_width_slope_m_per_m = trend.width_slope_m_per_m;
                candidate.ribbon_sample_count = trend.sample_count;

                const double start_width_near_candidate = sampleWidthAtOrNear(*ribbon, first.s);
                const double end_width_near_candidate = sampleWidthAtOrNear(*ribbon, last.s);
                const bool enough_samples = trend.sample_count >= cfg_.min_ribbon_sample_count;
                const bool near_start = std::abs(slice.s - first.s) <= cfg_.endpoint_tolerance_m;
                const bool near_end = std::abs(slice.s - last.s) <= cfg_.endpoint_tolerance_m;
                if (enough_samples &&
                    near_start &&
                    start_width_near_candidate <= cfg_.split_initial_width_max_m &&
                    end_width_near_candidate >= cfg_.split_final_width_min_m &&
                    trend.width_slope_m_per_m >= cfg_.split_width_slope_min_m_per_m) {
                    candidate.event_hint = "split_start";
                } else if (enough_samples &&
                           near_end &&
                           start_width_near_candidate >= cfg_.merge_initial_width_min_m &&
                           end_width_near_candidate <= cfg_.merge_final_width_max_m &&
                           trend.width_slope_m_per_m <= cfg_.merge_width_slope_max_m_per_m) {
                    candidate.event_hint = "merge_end";
                }
            }

            candidate.id = "J" + std::to_string(result.candidates.size());
            candidate.label = candidate.id;
            result.candidates.push_back(std::move(candidate));
        }
    }

    for (const auto& ribbon : ribbons.ribbons) {
        if (ribbon.kind != FrenetRibbonKind::kTopologyLane || ribbon.samples.empty()) continue;
        const auto trend = ribbons.computeTrend(ribbon.id);
        if (trend.sample_count < cfg_.min_ribbon_sample_count) continue;
        const auto& first = ribbon.samples.front();
        const auto& last = ribbon.samples.back();
        if (hasCandidateForRibbonEvent(result.candidates, ribbon.id, "merge_end")) continue;
        const bool ribbon_narrows_to_merge =
            first.width_m >= cfg_.merge_initial_width_min_m &&
            last.width_m <= 2.0 &&
            first.width_m - last.width_m >= 0.8 &&
            trend.width_slope_m_per_m <= cfg_.merge_width_slope_max_m_per_m;
        if (!ribbon_narrows_to_merge) continue;
        if (last.slice_index < 0 ||
            static_cast<std::size_t>(last.slice_index) >= samples.slices.size()) {
            continue;
        }
        const auto& slice = samples.slices[static_cast<std::size_t>(last.slice_index)];
        FrenetJunctionCandidate candidate;
        candidate.s = last.s;
        candidate.l = 0.5 * (last.right_l + last.left_l);
        candidate.x = slice.origin_x + candidate.l * slice.normal_x;
        candidate.y = slice.origin_y + candidate.l * slice.normal_y;
        candidate.right_track_id = ribbon.right_track_id;
        candidate.left_track_id = ribbon.left_track_id;
        candidate.right_line_id = ribbon.right_track_id;
        candidate.left_line_id = ribbon.left_track_id;
        if (const auto* right_hit = hitForTrackAtSlice(slice, ribbon.right_track_id)) {
            candidate.right_line_id = right_hit->source_line_id;
        }
        if (const auto* left_hit = hitForTrackAtSlice(slice, ribbon.left_track_id)) {
            candidate.left_line_id = left_hit->source_line_id;
        }
        candidate.distance_m = last.width_m;
        if (ribbon.samples.size() >= 2) {
            candidate.prev_distance_m = ribbon.samples[ribbon.samples.size() - 2].width_m;
            candidate.has_prev_distance = true;
        }
        candidate.event_hint = "merge_end";
        candidate.ribbon_id = ribbon.id;
        candidate.ribbon_label = ribbon.label;
        candidate.ribbon_start_s = first.s;
        candidate.ribbon_end_s = last.s;
        candidate.ribbon_width_start_m = first.width_m;
        candidate.ribbon_width_end_m = last.width_m;
        candidate.ribbon_width_slope_m_per_m = trend.width_slope_m_per_m;
        candidate.ribbon_sample_count = trend.sample_count;
        candidate.id = "J" + std::to_string(result.candidates.size());
        candidate.label = candidate.id;
        result.candidates.push_back(std::move(candidate));
    }

    struct BestEvent {
        std::size_t index = 0;
        double endpoint_distance_m = std::numeric_limits<double>::infinity();
    };
    std::map<std::string, BestEvent> best_split_by_ribbon;
    std::map<std::string, BestEvent> best_merge_by_ribbon;
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        const auto& candidate = result.candidates[i];
        if (candidate.ribbon_id.empty()) continue;
        if (candidate.event_hint == "split_start") {
            const double distance = std::abs(candidate.s - candidate.ribbon_start_s);
            auto& best = best_split_by_ribbon[candidate.ribbon_id];
            if (distance < best.endpoint_distance_m) {
                best.index = i;
                best.endpoint_distance_m = distance;
            }
        } else if (candidate.event_hint == "merge_end") {
            const double distance = std::abs(candidate.s - candidate.ribbon_end_s);
            auto& best = best_merge_by_ribbon[candidate.ribbon_id];
            if (distance < best.endpoint_distance_m) {
                best.index = i;
                best.endpoint_distance_m = distance;
            }
        }
    }
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        auto& candidate = result.candidates[i];
        if (candidate.ribbon_id.empty()) continue;
        if (candidate.event_hint == "split_start") {
            const auto it = best_split_by_ribbon.find(candidate.ribbon_id);
            if (it != best_split_by_ribbon.end() && it->second.index != i) {
                candidate.event_hint = "split_continuation";
            }
        } else if (candidate.event_hint == "merge_end") {
            const auto it = best_merge_by_ribbon.find(candidate.ribbon_id);
            if (it != best_merge_by_ribbon.end() && it->second.index != i) {
                candidate.event_hint = "merge_continuation";
            }
        }
    }

    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
