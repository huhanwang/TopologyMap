#include "frenet_lane_line_completer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace topology_map::algorithms {
namespace {

struct WorkingNode {
    std::string track_id;
    std::string lane_type;
    int lane_type_value = 0;
    double s = 0.0;
    double l = 0.0;
    double x = 0.0;
    double y = 0.0;
    double confidence = 1.0;
    bool inferred = false;
};

using WorkingTrackMap = std::map<std::string, std::map<int, WorkingNode>>;
using ExistingLaneSliceMap = std::map<int, std::vector<WorkingNode>>;

const FrenetTrack* findTrack(const FrenetTrackResult& tracks, const std::string& track_id) {
    const auto it = tracks.track_index_by_id.find(track_id);
    if (it == tracks.track_index_by_id.end() || it->second >= tracks.tracks.size()) return nullptr;
    return &tracks.tracks[it->second];
}

const WorkingNode* workingNodeAt(const WorkingTrackMap& working,
                                 const std::string& track_id,
                                 int slice_index) {
    const auto track_it = working.find(track_id);
    if (track_it == working.end()) return nullptr;
    const auto sample_it = track_it->second.find(slice_index);
    if (sample_it == track_it->second.end()) return nullptr;
    return &sample_it->second;
}

bool validWidth(double width_m, double min_width_m, double max_width_m) {
    return std::isfinite(width_m) && width_m >= min_width_m && width_m <= max_width_m;
}

bool isLaneWidthRibbon(const FrenetRibbonResult& ribbons,
                       const FrenetRibbonTrack& ribbon,
                       double min_lane_width_m,
                       double max_lane_width_m,
                       int min_sample_count) {
    if (ribbon.kind != FrenetRibbonKind::kTopologyLane) return false;
    if (static_cast<int>(ribbon.samples.size()) < min_sample_count) return false;
    const auto stats = ribbons.computeStats(ribbon.id);
    return stats.used_sample_count >= min_sample_count &&
           stats.width_median >= min_lane_width_m &&
           stats.width_median <= max_lane_width_m;
}

bool hasLaneWidthTopologyRibbon(const FrenetRibbonResult& ribbons,
                                const std::string& track_id,
                                double min_lane_width_m,
                                double max_lane_width_m,
                                int min_sample_count) {
    const auto* refs = ribbons.ribbonsOfTrack(track_id);
    if (!refs) return false;
    std::set<std::string> visited;
    auto checkRefs = [&](const std::vector<RibbonSampleRef>& side_refs) {
        for (const auto& ref : side_refs) {
            if (!visited.insert(ref.ribbon_id).second) continue;
            const auto* ribbon = ribbons.ribbonById(ref.ribbon_id);
            if (!ribbon) continue;
            if (isLaneWidthRibbon(ribbons, *ribbon, min_lane_width_m, max_lane_width_m, min_sample_count)) {
                return true;
            }
        }
        return false;
    };
    return checkRefs(refs->left_side) || checkRefs(refs->right_side);
}

bool hasNearExistingLaneNode(const ExistingLaneSliceMap& existing_lane_by_slice,
                             int slice_index,
                             const std::string& target_track_id,
                             double l,
                             double threshold_m) {
    const auto it = existing_lane_by_slice.find(slice_index);
    if (it == existing_lane_by_slice.end()) return false;
    for (const auto& node : it->second) {
        if (node.track_id == target_track_id) continue;
        if (std::abs(node.l - l) <= threshold_m) return true;
    }
    return false;
}

bool hasNearInferredLaneNode(const FrenetCompletionResult& result,
                             int slice_index,
                             const std::string& target_track_id,
                             double l,
                             double threshold_m) {
    if (slice_index < 0 || static_cast<std::size_t>(slice_index) >= result.inferred_by_slice.size()) return false;
    for (int idx : result.inferred_by_slice[static_cast<std::size_t>(slice_index)]) {
        const auto& node = result.inferred_nodes[static_cast<std::size_t>(idx)];
        bool same_track = false;
        std::size_t start = 0;
        while (start <= node.track_id.size()) {
            const std::size_t end = node.track_id.find('|', start);
            const std::string token = node.track_id.substr(
                start, end == std::string::npos ? std::string::npos : end - start);
            if (token == target_track_id) {
                same_track = true;
                break;
            }
            if (end == std::string::npos) break;
            start = end + 1;
        }
        if (same_track) continue;
        if (std::abs(node.l - l) <= threshold_m) return true;
    }
    return false;
}

std::vector<int> missingSlicesBeforeTrack(const FrenetTrack& track,
                                          const BoundarySamplingResult& raw_samples,
                                          const WorkingTrackMap& working) {
    std::vector<int> slices;
    if (track.samples.empty()) return slices;
    const int first_slice = track.samples.front().slice_index;
    for (int si = first_slice - 1; si >= 0; --si) {
        const auto* node = workingNodeAt(working, track.id, si);
        if (node && !node->inferred) break;
        slices.push_back(si);
    }
    return slices;
}

std::vector<int> missingSlicesAfterTrack(const FrenetTrack& track,
                                         const BoundarySamplingResult& raw_samples,
                                         const WorkingTrackMap& working) {
    std::vector<int> slices;
    if (track.samples.empty()) return slices;
    const int last_slice = track.samples.back().slice_index;
    for (int si = last_slice + 1; si < static_cast<int>(raw_samples.slices.size()); ++si) {
        const auto* node = workingNodeAt(working, track.id, si);
        if (node && !node->inferred) break;
        slices.push_back(si);
    }
    return slices;
}

struct SideCandidate {
    const FrenetRibbonTrack* ribbon = nullptr;
    const WorkingNode* anchor = nullptr;
    std::string anchor_track_id;
    std::optional<RibbonWidthEstimate> width;
    bool target_is_right_boundary = false;
    double confidence = 0.0;
};

struct TrackPrediction {
    bool valid = false;
    double l = 0.0;
    double weight = 0.0;
    double extrapolation_m = 0.0;
};

struct LateralObservation {
    const SideCandidate* candidate = nullptr;
    double l = 0.0;
    double weight = 0.0;
};

TrackPrediction predictTrackEndpointL(const FrenetTrack& track,
                                      double target_s,
                                      bool before_track,
                                      int sample_count,
                                      double max_extrapolation_m,
                                      double base_weight) {
    TrackPrediction prediction;
    if (track.samples.size() < 2 || sample_count < 2) return prediction;
    const double endpoint_s = before_track ? track.samples.front().s : track.samples.back().s;
    prediction.extrapolation_m = std::abs(target_s - endpoint_s);
    if (prediction.extrapolation_m > max_extrapolation_m) return prediction;

    std::vector<const FrenetTrackSample*> samples;
    const int n = std::min(sample_count, static_cast<int>(track.samples.size()));
    samples.reserve(static_cast<std::size_t>(n));
    if (before_track) {
        for (int i = 0; i < n; ++i) samples.push_back(&track.samples[static_cast<std::size_t>(i)]);
    } else {
        for (int i = static_cast<int>(track.samples.size()) - n; i < static_cast<int>(track.samples.size()); ++i) {
            samples.push_back(&track.samples[static_cast<std::size_t>(i)]);
        }
    }

    double sum_s = 0.0;
    double sum_l = 0.0;
    for (const auto* sample : samples) {
        sum_s += sample->s;
        sum_l += sample->l;
    }
    const double count = static_cast<double>(samples.size());
    const double mean_s = sum_s / count;
    const double mean_l = sum_l / count;
    double var_s = 0.0;
    double cov_sl = 0.0;
    for (const auto* sample : samples) {
        const double ds = sample->s - mean_s;
        var_s += ds * ds;
        cov_sl += ds * (sample->l - mean_l);
    }
    const double slope = var_s > 1e-9 ? cov_sl / var_s : 0.0;
    const double intercept = mean_l - slope * mean_s;
    prediction.valid = true;
    prediction.l = intercept + slope * target_s;
    const double normalized = std::max(0.0, 1.0 - prediction.extrapolation_m / max_extrapolation_m);
    prediction.weight = base_weight * normalized * normalized;
    return prediction;
}

bool isEndpointLaneWidthRibbon(const FrenetRibbonTrack& ribbon,
                               double min_lane_width_m,
                               double max_lane_width_m) {
    if (ribbon.kind != FrenetRibbonKind::kTopologyLane || ribbon.samples.empty()) return false;
    for (const auto& sample : ribbon.samples) {
        if (validWidth(sample.width_m, min_lane_width_m, max_lane_width_m)) return true;
    }
    return false;
}

bool hasNarrowingMergeEndpointRibbon(const FrenetRibbonResult& ribbons,
                                     const std::vector<const FrenetRibbonTrack*>& endpoint_ribbons,
                                     double min_lane_width_m,
                                     int min_sample_count) {
    for (const auto* ribbon : endpoint_ribbons) {
        if (!ribbon || ribbon->kind != FrenetRibbonKind::kTopologyLane) continue;
        if (static_cast<int>(ribbon->samples.size()) < min_sample_count) continue;
        const auto trend = ribbons.computeTrend(ribbon->id);
        if (trend.sample_count < min_sample_count) continue;
        const double first_width = ribbon->samples.front().width_m;
        const double last_width = ribbon->samples.back().width_m;
        if (first_width >= min_lane_width_m &&
            last_width < min_lane_width_m &&
            first_width - last_width >= 0.8 &&
            trend.width_slope_m_per_m <= -0.02) {
            return true;
        }
    }
    return false;
}

void maybeAddCandidate(const FrenetRibbonResult& ribbons,
                       const WorkingTrackMap& working,
                       const std::string& target_track_id,
                       const FrenetRibbonTrack& ribbon,
                       int slice_index,
                       double s,
                       double min_width_m,
                       double max_width_m,
                       double min_lane_width_m,
                       double max_lane_width_m,
                       int min_lane_sample_count,
                       bool allow_weak_endpoint_ribbon,
                       std::vector<SideCandidate>* left_candidates,
                       std::vector<SideCandidate>* right_candidates) {
    const bool stable_lane_width = isLaneWidthRibbon(
        ribbons, ribbon, min_lane_width_m, max_lane_width_m, min_lane_sample_count);
    if (!stable_lane_width &&
        (!allow_weak_endpoint_ribbon ||
         !isEndpointLaneWidthRibbon(ribbon, min_lane_width_m, max_lane_width_m))) {
        return;
    }
    std::string neighbor_track_id;
    bool self_is_right_boundary = false;
    bool self_is_left_boundary = false;
    if (target_track_id == ribbon.right_track_id) {
        neighbor_track_id = ribbon.left_track_id;
        self_is_right_boundary = true;
    } else if (target_track_id == ribbon.left_track_id) {
        neighbor_track_id = ribbon.right_track_id;
        self_is_left_boundary = true;
    } else {
        return;
    }

    const auto* anchor = workingNodeAt(working, neighbor_track_id, slice_index);
    if (!anchor) return;
    auto width = ribbons.estimateWidth(ribbon.id, s);
    if (!width || !validWidth(width->width_m, min_width_m, max_width_m)) return;

    SideCandidate candidate;
    candidate.ribbon = &ribbon;
    candidate.anchor = anchor;
    candidate.anchor_track_id = neighbor_track_id;
    candidate.width = width;
    candidate.target_is_right_boundary = self_is_right_boundary;
    candidate.confidence = std::min(anchor->confidence, width->confidence);
    if (!stable_lane_width) candidate.confidence *= 0.45;

    if (self_is_right_boundary) {
        left_candidates->push_back(candidate);
    } else if (self_is_left_boundary) {
        right_candidates->push_back(candidate);
    }
}

std::vector<LateralObservation> buildObservations(const std::vector<SideCandidate>& candidates,
                                                  bool target_from_left_anchor,
                                                  double base_weight,
                                                  double weak_weight,
                                                  int min_lane_sample_count,
                                                  double width_slope_scale,
                                                  double center_slope_scale) {
    std::vector<LateralObservation> observations;
    observations.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        if (!candidate.anchor || !candidate.width || !candidate.ribbon) continue;
        LateralObservation obs;
        obs.candidate = &candidate;
        obs.l = target_from_left_anchor
            ? candidate.anchor->l - candidate.width->width_m
            : candidate.anchor->l + candidate.width->width_m;
        const bool weak = static_cast<int>(candidate.ribbon->samples.size()) < min_lane_sample_count;
        const double support_weight = std::max(0.2, 1.0 - 0.025 * candidate.width->support_distance_m);
        double stability_weight = 1.0;
        if (candidate.ribbon->samples.size() >= 2) {
            const auto& first = candidate.ribbon->samples.front();
            const auto& last = candidate.ribbon->samples.back();
            const double ds = std::abs(last.s - first.s);
            if (ds > 1e-6) {
                const double width_slope = std::abs(last.width_m - first.width_m) / ds;
                const double center_slope = std::abs(last.center_l_m - first.center_l_m) / ds;
                stability_weight =
                    1.0 / (1.0 + width_slope / std::max(1e-6, width_slope_scale) +
                           center_slope / std::max(1e-6, center_slope_scale));
            }
        }
        obs.weight = (weak ? weak_weight : base_weight) * candidate.confidence * support_weight * stability_weight;
        if (std::isfinite(obs.l) && obs.weight > 1e-6) observations.push_back(obs);
    }
    return observations;
}

const SideCandidate* strongestCandidate(const std::vector<SideCandidate>& left_candidates,
                                        const std::vector<SideCandidate>& right_candidates) {
    const SideCandidate* best = nullptr;
    auto visit = [&](const std::vector<SideCandidate>& candidates) {
        for (const auto& candidate : candidates) {
            if (!best || candidate.confidence > best->confidence) best = &candidate;
        }
    };
    visit(left_candidates);
    visit(right_candidates);
    return best;
}

std::optional<double> fusePredictionAndObservations(const TrackPrediction& prediction,
                                                    const std::vector<LateralObservation>& observations,
                                                    double prediction_residual_scale_m) {
    double weighted_sum = 0.0;
    double weight_sum = 0.0;
    if (prediction.valid && prediction.weight > 1e-6 && std::isfinite(prediction.l)) {
        double prediction_weight = prediction.weight;
        double obs_sum = 0.0;
        double obs_weight = 0.0;
        for (const auto& obs : observations) {
            if (!std::isfinite(obs.l) || obs.weight <= 1e-6) continue;
            obs_sum += obs.weight * obs.l;
            obs_weight += obs.weight;
        }
        if (obs_weight > 1e-6) {
            const double obs_mean = obs_sum / obs_weight;
            const double residual = std::abs(prediction.l - obs_mean);
            const double scale = std::max(1e-6, prediction_residual_scale_m);
            prediction_weight *= 1.0 / (1.0 + (residual / scale) * (residual / scale));
        }
        weighted_sum += prediction_weight * prediction.l;
        weight_sum += prediction_weight;
    }
    for (const auto& obs : observations) {
        if (!std::isfinite(obs.l) || obs.weight <= 1e-6) continue;
        weighted_sum += obs.weight * obs.l;
        weight_sum += obs.weight;
    }
    if (weight_sum <= 1e-6) return std::nullopt;
    return weighted_sum / weight_sum;
}

void rebuildCompletionIndexes(FrenetCompletionResult* result, std::size_t slice_count) {
    if (!result) return;
    result->inferred_by_track_id.clear();
    result->inferred_by_slice.assign(slice_count, {});
    for (std::size_t i = 0; i < result->inferred_nodes.size(); ++i) {
        auto& node = result->inferred_nodes[i];
        node.label = "C" + std::to_string(i);
        result->inferred_by_track_id[node.track_id].push_back(static_cast<int>(i));
        if (node.slice_index >= 0 && static_cast<std::size_t>(node.slice_index) < result->inferred_by_slice.size()) {
            result->inferred_by_slice[static_cast<std::size_t>(node.slice_index)].push_back(static_cast<int>(i));
        }
    }
}

const FrenetTrackSample* firstSample(const FrenetTrack& track) {
    if (track.samples.empty()) return nullptr;
    return &track.samples.front();
}

const FrenetTrackSample* lastSample(const FrenetTrack& track) {
    if (track.samples.empty()) return nullptr;
    return &track.samples.back();
}

bool shareLaneWidthRibbon(const FrenetRibbonResult& ribbons,
                          const std::string& a_track_id,
                          const std::string& b_track_id,
                          double min_lane_width_m,
                          double max_lane_width_m,
                          int min_sample_count) {
    const auto* refs = ribbons.ribbonsOfTrack(a_track_id);
    if (!refs) return false;
    std::set<std::string> visited;
    auto checkRefs = [&](const std::vector<RibbonSampleRef>& side_refs) {
        for (const auto& ref : side_refs) {
            if (ref.neighbor_track_id != b_track_id) continue;
            if (!visited.insert(ref.ribbon_id).second) continue;
            const auto* ribbon = ribbons.ribbonById(ref.ribbon_id);
            if (!ribbon) continue;
            if (isLaneWidthRibbon(ribbons, *ribbon, min_lane_width_m, max_lane_width_m, min_sample_count)) {
                return true;
            }
        }
        return false;
    };
    return checkRefs(refs->left_side) || checkRefs(refs->right_side);
}

std::set<std::string> laneWidthAnchorNeighbors(const FrenetRibbonResult& ribbons,
                                               const std::string& track_id,
                                               double min_lane_width_m,
                                               double max_lane_width_m,
                                               int min_sample_count) {
    std::set<std::string> neighbors;
    const auto* refs = ribbons.ribbonsOfTrack(track_id);
    if (!refs) return neighbors;
    std::set<std::string> visited;
    auto collect = [&](const std::vector<RibbonSampleRef>& side_refs) {
        for (const auto& ref : side_refs) {
            if (!visited.insert(ref.ribbon_id).second) continue;
            const auto* ribbon = ribbons.ribbonById(ref.ribbon_id);
            if (!ribbon) continue;
            if (isLaneWidthRibbon(ribbons, *ribbon, min_lane_width_m, max_lane_width_m, min_sample_count)) {
                neighbors.insert(ref.neighbor_track_id);
            }
        }
    };
    collect(refs->left_side);
    collect(refs->right_side);
    return neighbors;
}

bool shareLaneWidthAnchor(const FrenetRibbonResult& ribbons,
                          const std::string& a_track_id,
                          const std::string& b_track_id,
                          double min_lane_width_m,
                          double max_lane_width_m,
                          int min_sample_count) {
    const auto a_neighbors = laneWidthAnchorNeighbors(
        ribbons, a_track_id, min_lane_width_m, max_lane_width_m, min_sample_count);
    if (a_neighbors.empty()) return false;
    const auto b_neighbors = laneWidthAnchorNeighbors(
        ribbons, b_track_id, min_lane_width_m, max_lane_width_m, min_sample_count);
    for (const auto& id : a_neighbors) {
        if (b_neighbors.find(id) != b_neighbors.end()) return true;
    }
    return false;
}

bool canEntityLinkTracks(const FrenetRibbonResult& ribbons,
                         const std::string& a_track_id,
                         const std::string& b_track_id,
                         double min_lane_width_m,
                         double max_lane_width_m,
                         int min_sample_count) {
    return shareLaneWidthRibbon(ribbons, a_track_id, b_track_id,
                                min_lane_width_m, max_lane_width_m, min_sample_count) ||
           shareLaneWidthAnchor(ribbons, a_track_id, b_track_id,
                                min_lane_width_m, max_lane_width_m, min_sample_count);
}

bool containsTrackToken(const std::string& track_tokens, const std::string& track_id) {
    if (track_tokens == track_id) return true;
    std::size_t start = 0;
    while (start <= track_tokens.size()) {
        const std::size_t end = track_tokens.find('|', start);
        const std::string token = track_tokens.substr(
            start, end == std::string::npos ? std::string::npos : end - start);
        if (token == track_id) return true;
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return false;
}

struct EntityLinkCandidate {
    std::string from_track_id;
    std::string to_track_id;
    const FrenetTrackSample* from_tail = nullptr;
    const FrenetTrackSample* to_head = nullptr;
    double gap_m = 0.0;
};

std::set<std::string> nodeAnchorTokens(const FrenetInferredNode& node) {
    std::set<std::string> anchors;
    auto addTokens = [&](const std::string& tokens) {
        std::size_t start = 0;
        while (start <= tokens.size()) {
            const std::size_t end = tokens.find('|', start);
            const std::string token = tokens.substr(
                start, end == std::string::npos ? std::string::npos : end - start);
            if (!token.empty()) anchors.insert(token);
            if (end == std::string::npos) break;
            start = end + 1;
        }
    };
    addTokens(node.anchor_track_id);
    addTokens(node.left_anchor_track_id);
    addTokens(node.right_anchor_track_id);
    return anchors;
}

bool shareCompletionAnchorInGap(const FrenetCompletionResult& result,
                                const std::vector<int>& from_node_indices,
                                const std::vector<int>& to_node_indices,
                                double from_s,
                                double to_s) {
    std::set<std::string> from_anchors;
    for (int idx : from_node_indices) {
        const auto& node = result.inferred_nodes[static_cast<std::size_t>(idx)];
        if (node.s <= from_s || node.s >= to_s) continue;
        const auto anchors = nodeAnchorTokens(node);
        from_anchors.insert(anchors.begin(), anchors.end());
    }
    if (from_anchors.empty()) return false;
    for (int idx : to_node_indices) {
        const auto& node = result.inferred_nodes[static_cast<std::size_t>(idx)];
        if (node.s <= from_s || node.s >= to_s) continue;
        const auto anchors = nodeAnchorTokens(node);
        for (const auto& anchor : anchors) {
            if (from_anchors.find(anchor) != from_anchors.end()) return true;
        }
    }
    return false;
}

FrenetCompletionLink makeLink(const std::string& track_id,
                              const std::string& from_kind,
                              const std::string& to_kind,
                              const std::string& from_id,
                              const std::string& to_id,
                              double from_s,
                              double from_l,
                              double from_x,
                              double from_y,
                              double to_s,
                              double to_l,
                              double to_x,
                              double to_y,
                              const std::string& method,
                              int index) {
    FrenetCompletionLink link;
    link.id = "completion_link:" + std::to_string(index);
    link.label = "CL" + std::to_string(index);
    link.track_id = track_id;
    link.from_kind = from_kind;
    link.to_kind = to_kind;
    link.from_node_id = from_id;
    link.to_node_id = to_id;
    link.from_s = from_s;
    link.from_l = from_l;
    link.from_x = from_x;
    link.from_y = from_y;
    link.to_s = to_s;
    link.to_l = to_l;
    link.to_x = to_x;
    link.to_y = to_y;
    link.method = method;
    return link;
}

void refineEntityLinksAndBuildLinks(const BoundarySamplingResult& raw_samples,
                                    const FrenetTrackResult& tracks,
                                    const FrenetRibbonResult& ribbons,
                                    double min_lane_width_m,
                                    double max_lane_width_m,
                                    int min_sample_count,
                                    double max_gap_m,
                                    double max_shared_anchor_gap_m,
                                    double max_l_delta_m,
                                    FrenetCompletionResult* result) {
    if (!result) return;
    std::map<std::string, const FrenetTrack*> track_by_id;
    for (const auto& track : tracks.tracks) {
        if (isLaneBoundaryType(track.type) && !track.samples.empty()) {
            track_by_id[track.id] = &track;
        }
    }

    std::set<std::string> remove_keys;
    std::vector<EntityLinkCandidate> entity_links;
    for (const auto& [track_id, node_indices] : result->inferred_by_track_id) {
        const auto track_it = track_by_id.find(track_id);
        if (track_it == track_by_id.end()) continue;
        const auto& track = *track_it->second;
        const auto* tail = lastSample(track);
        const auto* head = firstSample(track);
        if (!tail || !head) continue;

        const FrenetTrack* next_track = nullptr;
        const FrenetTrack* prev_track = nullptr;
        double next_gap = std::numeric_limits<double>::infinity();
        double prev_gap = std::numeric_limits<double>::infinity();
        for (const auto& [other_id, other] : track_by_id) {
            if (other_id == track_id) continue;
            const auto* other_head = firstSample(*other);
            const auto* other_tail = lastSample(*other);
            if (other_head && other_head->s > tail->s) {
                const double gap = other_head->s - tail->s;
                const double l_delta = std::abs(other_head->l - tail->l);
                const auto other_nodes_it = result->inferred_by_track_id.find(other_id);
                const bool direct_relation = canEntityLinkTracks(
                    ribbons, track_id, other_id, min_lane_width_m, max_lane_width_m, min_sample_count);
                const bool shared_completion_anchor = other_nodes_it != result->inferred_by_track_id.end() &&
                    shareCompletionAnchorInGap(*result, node_indices, other_nodes_it->second,
                                               tail->s, other_head->s);
                const bool allow_direct = direct_relation && gap <= max_gap_m && l_delta <= max_l_delta_m;
                const bool allow_shared_anchor = shared_completion_anchor &&
                    gap <= max_shared_anchor_gap_m &&
                    l_delta <= max_l_delta_m;
                if (gap < next_gap && (allow_direct || allow_shared_anchor)) {
                    next_gap = gap;
                    next_track = other;
                }
            }
            if (other_tail && other_tail->s < head->s) {
                const double gap = head->s - other_tail->s;
                const double l_delta = std::abs(head->l - other_tail->l);
                const auto other_nodes_it = result->inferred_by_track_id.find(other_id);
                const bool direct_relation = canEntityLinkTracks(
                    ribbons, track_id, other_id, min_lane_width_m, max_lane_width_m, min_sample_count);
                const bool shared_completion_anchor = other_nodes_it != result->inferred_by_track_id.end() &&
                    shareCompletionAnchorInGap(*result, other_nodes_it->second, node_indices,
                                               other_tail->s, head->s);
                const bool allow_direct = direct_relation && gap <= max_gap_m && l_delta <= max_l_delta_m;
                const bool allow_shared_anchor = shared_completion_anchor &&
                    gap <= max_shared_anchor_gap_m &&
                    l_delta <= max_l_delta_m;
                if (gap < prev_gap && (allow_direct || allow_shared_anchor)) {
                    prev_gap = gap;
                    prev_track = other;
                }
            }
        }

        if (next_track) {
            const auto* next_head = firstSample(*next_track);
            entity_links.push_back({
                track_id,
                next_track->id,
                tail,
                next_head,
                next_gap,
            });
            for (int idx : node_indices) {
                const auto& node = result->inferred_nodes[static_cast<std::size_t>(idx)];
                if (node.s > tail->s && node.s < next_head->s) {
                    remove_keys.insert(node.track_id + ":" + std::to_string(node.slice_index));
                } else if (node.s >= next_head->s) {
                    remove_keys.insert(node.track_id + ":" + std::to_string(node.slice_index));
                }
            }
        }
        if (prev_track) {
            const auto* prev_tail = lastSample(*prev_track);
            for (int idx : node_indices) {
                const auto& node = result->inferred_nodes[static_cast<std::size_t>(idx)];
                if (node.s > prev_tail->s && node.s < head->s) {
                    remove_keys.insert(node.track_id + ":" + std::to_string(node.slice_index));
                } else if (node.s <= prev_tail->s) {
                    remove_keys.insert(node.track_id + ":" + std::to_string(node.slice_index));
                }
            }
        }
    }

    for (const auto& link : entity_links) {
        if (!link.from_tail || !link.to_head) continue;
        for (const auto& node : result->inferred_nodes) {
            if (node.s <= link.from_tail->s || node.s >= link.to_head->s) continue;
            if (!containsTrackToken(node.track_id, link.from_track_id) &&
                !containsTrackToken(node.track_id, link.to_track_id)) {
                continue;
            }
            remove_keys.insert(node.track_id + ":" + std::to_string(node.slice_index));
        }
    }

    if (!remove_keys.empty()) {
        std::vector<FrenetInferredNode> kept;
        kept.reserve(result->inferred_nodes.size());
        for (const auto& node : result->inferred_nodes) {
            const std::string key = node.track_id + ":" + std::to_string(node.slice_index);
            if (remove_keys.find(key) == remove_keys.end()) kept.push_back(node);
        }
        result->inferred_nodes = std::move(kept);
        rebuildCompletionIndexes(result, raw_samples.slices.size());
    }

    result->links.clear();
    int link_index = 0;
    for (const auto& link : entity_links) {
        if (!link.from_tail || !link.to_head) continue;
        result->links.push_back(makeLink(
            link.from_track_id + "|" + link.to_track_id,
            "entity", "entity",
            link.from_track_id + ":tail", link.to_track_id + ":head",
            link.from_tail->s, link.from_tail->l, link.from_tail->x, link.from_tail->y,
            link.to_head->s, link.to_head->l, link.to_head->x, link.to_head->y,
            "entity_to_next_entity", link_index++));
    }
    for (const auto& [track_id, node_indices] : result->inferred_by_track_id) {
        std::vector<const FrenetInferredNode*> nodes;
        nodes.reserve(node_indices.size());
        for (int idx : node_indices) nodes.push_back(&result->inferred_nodes[static_cast<std::size_t>(idx)]);
        std::sort(nodes.begin(), nodes.end(), [](const auto* a, const auto* b) {
            return a->s < b->s;
        });
        const auto track_it = track_by_id.find(track_id);
        const FrenetTrack* track = track_it == track_by_id.end() ? nullptr : track_it->second;
        std::vector<const FrenetInferredNode*> before_nodes;
        std::vector<const FrenetInferredNode*> after_nodes;
        if (track && !nodes.empty()) {
            const auto* head = firstSample(*track);
            const auto* tail = lastSample(*track);
            for (const auto* node : nodes) {
                if (head && node->s < head->s) {
                    before_nodes.push_back(node);
                } else if (tail && node->s > tail->s) {
                    after_nodes.push_back(node);
                }
            }
            if (head && nodes.front()->s < head->s) {
                const FrenetInferredNode* last_before_head = before_nodes.empty() ? nodes.front() : before_nodes.back();
                result->links.push_back(makeLink(
                    track_id, "inferred", "entity", last_before_head->id, track_id + ":head",
                    last_before_head->s, last_before_head->l, last_before_head->x, last_before_head->y,
                    head->s, head->l, head->x, head->y,
                    "inferred_to_entity", link_index++));
            }
            if (tail && nodes.back()->s > tail->s) {
                const FrenetInferredNode* first_after_tail = after_nodes.empty() ? nodes.back() : after_nodes.front();
                result->links.push_back(makeLink(
                    track_id, "entity", "inferred", track_id + ":tail", first_after_tail->id,
                    tail->s, tail->l, tail->x, tail->y,
                    first_after_tail->s, first_after_tail->l, first_after_tail->x, first_after_tail->y,
                    "entity_to_inferred", link_index++));
            }
        }
        auto addSequenceLinks = [&](const std::vector<const FrenetInferredNode*>& group) {
            for (std::size_t i = 1; i < group.size(); ++i) {
                result->links.push_back(makeLink(
                    track_id, "inferred", "inferred", group[i - 1]->id, group[i]->id,
                    group[i - 1]->s, group[i - 1]->l, group[i - 1]->x, group[i - 1]->y,
                    group[i]->s, group[i]->l, group[i]->x, group[i]->y,
                    "inferred_sequence", link_index++));
            }
        };
        if (track) {
            addSequenceLinks(before_nodes);
            addSequenceLinks(after_nodes);
        } else {
            addSequenceLinks(nodes);
        }
        if (track && !nodes.empty()) {
            const auto* tail = lastSample(*track);
            for (const auto& [other_id, other] : track_by_id) {
                if (other_id == track_id) continue;
                const auto* other_head = firstSample(*other);
                if (!tail || !other_head || other_head->s <= tail->s) continue;
                if (!canEntityLinkTracks(ribbons, track_id, other_id,
                                         min_lane_width_m, max_lane_width_m, min_sample_count)) continue;
                const double gap = other_head->s - tail->s;
                const double l_delta = std::abs(other_head->l - nodes.back()->l);
                if (gap <= max_gap_m && l_delta <= max_l_delta_m &&
                    nodes.back()->s < other_head->s) {
                    result->links.push_back(makeLink(
                        track_id + "|" + other_id, "inferred", "entity", nodes.back()->id, other_id + ":head",
                        nodes.back()->s, nodes.back()->l, nodes.back()->x, nodes.back()->y,
                        other_head->s, other_head->l, other_head->x, other_head->y,
                        "inferred_to_next_entity", link_index++));
                    break;
                }
            }
        }
    }
}

void mergeNearInferredNodes(FrenetCompletionResult* result,
                            std::size_t slice_count,
                            double threshold_m) {
    if (!result || result->inferred_nodes.empty()) return;
    std::map<int, std::vector<FrenetInferredNode>> by_slice;
    for (const auto& node : result->inferred_nodes) {
        by_slice[node.slice_index].push_back(node);
    }

    std::vector<FrenetInferredNode> merged_nodes;
    for (auto& [_, nodes] : by_slice) {
        std::sort(nodes.begin(), nodes.end(), [](const auto& a, const auto& b) {
            return a.l < b.l;
        });
        std::size_t i = 0;
        while (i < nodes.size()) {
            FrenetInferredNode merged = nodes[i];
            std::size_t j = i + 1;
            while (j < nodes.size() && std::abs(nodes[j].l - merged.l) <= threshold_m) {
                const double old_weight = static_cast<double>(j - i);
                merged.l = (merged.l * old_weight + nodes[j].l) / (old_weight + 1.0);
                merged.x = (merged.x * old_weight + nodes[j].x) / (old_weight + 1.0);
                merged.y = (merged.y * old_weight + nodes[j].y) / (old_weight + 1.0);
                merged.confidence = std::max(merged.confidence, nodes[j].confidence);
                merged.estimated_width_m =
                    (merged.estimated_width_m * old_weight + nodes[j].estimated_width_m) / (old_weight + 1.0);
                merged.track_id += "|" + nodes[j].track_id;
                if (!nodes[j].left_anchor_track_id.empty() &&
                    merged.left_anchor_track_id.find(nodes[j].left_anchor_track_id) == std::string::npos) {
                    if (!merged.left_anchor_track_id.empty()) merged.left_anchor_track_id += "|";
                    merged.left_anchor_track_id += nodes[j].left_anchor_track_id;
                }
                if (!nodes[j].right_anchor_track_id.empty() &&
                    merged.right_anchor_track_id.find(nodes[j].right_anchor_track_id) == std::string::npos) {
                    if (!merged.right_anchor_track_id.empty()) merged.right_anchor_track_id += "|";
                    merged.right_anchor_track_id += nodes[j].right_anchor_track_id;
                }
                ++j;
            }
            if (j > i + 1) {
                merged.id = "completion:connected_track_gap:" + std::to_string(merged.slice_index) + ":" +
                    std::to_string(merged_nodes.size());
                merged.method = "connected_track_gap_near_inferred_merge";
            }
            merged_nodes.push_back(std::move(merged));
            i = j;
        }
    }

    std::sort(merged_nodes.begin(), merged_nodes.end(), [](const auto& a, const auto& b) {
        if (a.slice_index != b.slice_index) return a.slice_index < b.slice_index;
        return a.l < b.l;
    });
    result->inferred_nodes = std::move(merged_nodes);
    rebuildCompletionIndexes(result, slice_count);
}

}  // namespace

FrenetLaneLineCompleter::FrenetLaneLineCompleter()
    : cfg_(Config{}) {}

FrenetLaneLineCompleter::FrenetLaneLineCompleter(Config config)
    : cfg_(std::move(config)) {}

FrenetCompletionResult FrenetLaneLineCompleter::complete(
    const BoundarySamplingResult& raw_samples,
    const FrenetTrackResult& tracks,
    const FrenetRibbonResult& ribbons) const {
    FrenetCompletionResult result;
    result.frame_id = raw_samples.frame_id;
    result.inferred_by_slice.resize(raw_samples.slices.size());
    if (!raw_samples.ok || !tracks.ok || !ribbons.ok) {
        result.error = !raw_samples.ok ? raw_samples.error : (!tracks.ok ? tracks.error : ribbons.error);
        if (result.error.empty()) result.error = "invalid_frenet_completion_input";
        return result;
    }

    ExistingLaneSliceMap existing_lane_by_slice;
    for (std::size_t slice_index = 0; slice_index < raw_samples.slices.size(); ++slice_index) {
        const auto& slice = raw_samples.slices[slice_index];
        for (const auto& hit : slice.hits) {
            if (frenetBoundaryTypeFromString(hit.source_type) != FrenetBoundaryType::kLaneLine) continue;
            const std::string track_id = hit.track_line_id.empty() ? hit.source_line_id : hit.track_line_id;
            existing_lane_by_slice[static_cast<int>(slice_index)].push_back({
                track_id,
                hit.lane_type,
                hit.lane_type_value,
                slice.s,
                hit.offset_m,
                hit.x,
                hit.y,
                hit.confidence,
                false,
            });
        }
    }

    WorkingTrackMap working;
    std::vector<const FrenetTrack*> target_tracks;
    for (const auto& track : tracks.tracks) {
        if (!isLaneBoundaryType(track.type)) continue;
        if (static_cast<int>(track.samples.size()) < cfg_.min_track_sample_count) continue;
        if (track.support_length_m < cfg_.min_track_support_length_m) continue;
        if (track.support_length_m < cfg_.short_track_support_length_m &&
            !hasLaneWidthTopologyRibbon(ribbons, track.id,
                                        cfg_.min_lane_ribbon_width_m,
                                        cfg_.max_lane_ribbon_width_m,
                                        cfg_.min_lane_ribbon_sample_count)) {
            continue;
        }
        target_tracks.push_back(&track);
        auto& samples_by_slice = working[track.id];
        for (const auto& sample : track.samples) {
            samples_by_slice[sample.slice_index] = {
                track.id,
                sample.lane_type,
                sample.lane_type_value,
                sample.s,
                sample.l,
                sample.x,
                sample.y,
                sample.confidence,
                false,
            };
        }
    }

    std::set<std::string> emitted_keys;
    std::map<std::string, int> stop_before_slice_by_track;
    std::map<std::string, int> stop_after_slice_by_track;
    int label_index = 0;
    bool changed = true;
    int iteration = 0;
    while (changed && iteration < cfg_.max_iterations) {
        changed = false;
        ++iteration;
        for (const auto* track : target_tracks) {
            std::vector<int> slice_order = missingSlicesBeforeTrack(*track, raw_samples, working);
            auto after_slices = missingSlicesAfterTrack(*track, raw_samples, working);
            slice_order.insert(slice_order.end(), after_slices.begin(), after_slices.end());
            for (int si : slice_order) {
                if (si < 0 || si >= static_cast<int>(raw_samples.slices.size())) continue;
                if (!track->samples.empty()) {
                    const int first_slice = track->samples.front().slice_index;
                    const int last_slice = track->samples.back().slice_index;
                    const auto before_stop = stop_before_slice_by_track.find(track->id);
                    if (before_stop != stop_before_slice_by_track.end() && si <= before_stop->second) {
                        continue;
                    }
                    const auto after_stop = stop_after_slice_by_track.find(track->id);
                    if (after_stop != stop_after_slice_by_track.end() && si >= after_stop->second) {
                        continue;
                    }
                    if (si >= first_slice && si <= last_slice) continue;
                }
                if (workingNodeAt(working, track->id, si)) continue;

                const auto& slice = raw_samples.slices[static_cast<std::size_t>(si)];
                std::vector<SideCandidate> left_candidates;
                std::vector<SideCandidate> right_candidates;
                const bool before_track = !track->samples.empty() && si < track->samples.front().slice_index;
                const auto endpoint_ribbons = ribbons.topologyRibbonsAtEndpoint(
                    *track,
                    before_track ? FrenetRibbonEndpointDirection::kBeforeTrackHead
                                 : FrenetRibbonEndpointDirection::kAfterTrackTail,
                    cfg_.endpoint_ribbon_tolerance_m);
                if (!before_track &&
                    hasNarrowingMergeEndpointRibbon(ribbons, endpoint_ribbons,
                                                    cfg_.min_lane_ribbon_width_m,
                                                    cfg_.min_lane_ribbon_sample_count)) {
                    const TrackPrediction prediction = predictTrackEndpointL(
                        *track,
                        slice.s,
                        before_track,
                        cfg_.track_prediction_sample_count,
                        cfg_.track_prediction_max_extrapolation_m,
                        cfg_.track_prediction_base_weight);
                    if (prediction.valid && std::isfinite(prediction.l)) {
                        FrenetInferredNode node;
                        node.track_id = track->id;
                        node.slice_index = si;
                        node.s = slice.s;
                        node.source_type = "lane_line";
                        node.lane_type = track->lane_type_summary;
                        node.l = prediction.l;
                        node.x = slice.origin_x + node.l * slice.normal_x;
                        node.y = slice.origin_y + node.l * slice.normal_y;
                        node.id = "completion_stop:lane_line:" + track->id + ":" + std::to_string(si);
                        node.label = "S" + std::to_string(result.stop_nodes.size());
                        node.method = "track_prediction_narrowing_merge_endpoint_stop";
                        node.confidence = 0.0;
                        result.stop_nodes.push_back(node);
                        stop_after_slice_by_track[track->id] = si;
                    }
                    break;
                }
                for (const auto* ribbon : endpoint_ribbons) {
                    if (!ribbon) continue;
                    maybeAddCandidate(ribbons, working, track->id, *ribbon, si, slice.s,
                                      cfg_.min_width_m, cfg_.max_width_m,
                                      cfg_.min_lane_ribbon_width_m, cfg_.max_lane_ribbon_width_m,
                                      cfg_.min_lane_ribbon_sample_count,
                                      true,
                                      &left_candidates, &right_candidates);
                }

                const auto left_observations = buildObservations(
                    left_candidates, true,
                    cfg_.endpoint_observation_base_weight,
                    cfg_.weak_endpoint_observation_weight,
                    cfg_.min_lane_ribbon_sample_count,
                    cfg_.ribbon_width_slope_weight_scale,
                    cfg_.ribbon_center_slope_weight_scale);
                const auto right_observations = buildObservations(
                    right_candidates, false,
                    cfg_.endpoint_observation_base_weight,
                    cfg_.weak_endpoint_observation_weight,
                    cfg_.min_lane_ribbon_sample_count,
                    cfg_.ribbon_width_slope_weight_scale,
                    cfg_.ribbon_center_slope_weight_scale);
                std::vector<LateralObservation> observations = left_observations;
                observations.insert(observations.end(), right_observations.begin(), right_observations.end());
                if (observations.empty()) continue;
                const TrackPrediction prediction = predictTrackEndpointL(
                    *track,
                    slice.s,
                    before_track,
                    cfg_.track_prediction_sample_count,
                    cfg_.track_prediction_max_extrapolation_m,
                    cfg_.track_prediction_base_weight);
                auto fused_l = fusePredictionAndObservations(
                    prediction, observations, cfg_.track_prediction_observation_residual_scale_m);
                if (!fused_l) continue;
                const auto* strongest = strongestCandidate(left_candidates, right_candidates);

                FrenetInferredNode node;
                node.track_id = track->id;
                node.slice_index = si;
                node.s = slice.s;
                node.source_type = "lane_line";
                node.lane_type = track->lane_type_summary;
                node.l = *fused_l;
                if (!left_observations.empty() && !right_observations.empty()) {
                    node.method = "track_prediction_bilateral_topology_ribbon_fusion";
                } else if (!left_observations.empty()) {
                    node.method = "track_prediction_left_topology_ribbon_fusion";
                } else if (!right_observations.empty()) {
                    node.method = "track_prediction_right_topology_ribbon_fusion";
                } else {
                    node.method = "track_prediction_only";
                }
                if (strongest) {
                    node.anchor_track_id = strongest->anchor_track_id;
                    node.ribbon_id = strongest->ribbon ? strongest->ribbon->id : "";
                    node.estimated_width_m = strongest->width ? strongest->width->width_m : 0.0;
                    node.width_support_s = strongest->width ? strongest->width->support_s : 0.0;
                    node.width_support_distance_m = strongest->width ? strongest->width->support_distance_m : 0.0;
                    if (strongest->target_is_right_boundary) {
                        node.left_anchor_track_id = strongest->anchor_track_id;
                        node.left_ribbon_id = node.ribbon_id;
                        node.left_width_m = node.estimated_width_m;
                    } else {
                        node.right_anchor_track_id = strongest->anchor_track_id;
                        node.right_ribbon_id = node.ribbon_id;
                        node.right_width_m = node.estimated_width_m;
                    }
                    node.confidence = std::min(0.95, 0.45 + 0.35 * strongest->confidence);
                } else {
                    node.confidence = prediction.valid ? std::min(0.75, prediction.weight) : 0.3;
                }

                if (!std::isfinite(node.l)) continue;
                const bool near_existing_lane = hasNearExistingLaneNode(
                    existing_lane_by_slice, si, track->id, node.l, cfg_.near_existing_lane_stop_l_m);
                const bool near_inferred_lane = hasNearInferredLaneNode(
                    result, si, track->id, node.l, cfg_.near_inferred_lane_stop_l_m);
                if (near_existing_lane || near_inferred_lane) {
                    node.x = slice.origin_x + node.l * slice.normal_x;
                    node.y = slice.origin_y + node.l * slice.normal_y;
                    node.id = "completion_stop:lane_line:" + track->id + ":" + std::to_string(si);
                    node.label = "S" + std::to_string(result.stop_nodes.size());
                    node.method += near_existing_lane ? "_near_existing_lane_stop" : "_near_inferred_lane_stop";
                    node.confidence = 0.0;
                    result.stop_nodes.push_back(node);
                    if (!track->samples.empty() && si < track->samples.front().slice_index) {
                        stop_before_slice_by_track[track->id] = si;
                    } else {
                        stop_after_slice_by_track[track->id] = si;
                    }
                    break;
                }
                node.x = slice.origin_x + node.l * slice.normal_x;
                node.y = slice.origin_y + node.l * slice.normal_y;
                node.id = "completion:lane_line:" + track->id + ":" + std::to_string(si);
                node.label = "C" + std::to_string(label_index++);

                const std::string key = track->id + ":" + std::to_string(si);
                if (!emitted_keys.insert(key).second) continue;
                const int node_index = static_cast<int>(result.inferred_nodes.size());
                result.inferred_by_track_id[track->id].push_back(node_index);
                result.inferred_by_slice[static_cast<std::size_t>(si)].push_back(node_index);
                result.inferred_nodes.push_back(node);

                if (cfg_.use_inferred_as_anchor) {
                    working[track->id][si] = {
                        track->id,
                        node.lane_type,
                        node.lane_type_value,
                        node.s,
                        node.l,
                        node.x,
                        node.y,
                        node.confidence,
                        true,
                    };
                    changed = true;
                }
            }
        }
    }

    mergeNearInferredNodes(&result, raw_samples.slices.size(), cfg_.merge_near_inferred_l_m);
    refineEntityLinksAndBuildLinks(raw_samples, tracks, ribbons,
                                   cfg_.min_lane_ribbon_width_m,
                                   cfg_.max_lane_ribbon_width_m,
                                   cfg_.min_lane_ribbon_sample_count,
                                   cfg_.max_entity_link_gap_m,
                                   cfg_.max_shared_anchor_entity_link_gap_m,
                                   cfg_.max_entity_link_l_delta_m,
                                   &result);
    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
