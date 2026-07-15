#include "frenet_completed_boundary_builder.h"

#include <algorithm>
#include <map>
#include <set>
#include <utility>

namespace topology_map::algorithms {
namespace {

class DisjointSet {
public:
    void add(const std::string& id) { parent_.emplace(id, id); }

    std::string find(const std::string& id) {
        auto it = parent_.find(id);
        if (it == parent_.end()) return id;
        if (it->second == id) return id;
        it->second = find(it->second);
        return it->second;
    }

    void unite(const std::string& a, const std::string& b) {
        const std::string root_a = find(a);
        const std::string root_b = find(b);
        if (root_a == root_b) return;
        // Keep IDs deterministic so debug comparisons remain stable.
        if (root_a < root_b) parent_[root_b] = root_a;
        else parent_[root_a] = root_b;
    }

private:
    std::map<std::string, std::string> parent_;
};

bool isEntityConnector(const FrenetCompletionLink& link) {
    return link.method == "entity_to_next_entity" &&
        link.from_kind == "entity" && link.to_kind == "entity";
}

}  // namespace

std::string frenetCompletedBoundaryStateToString(FrenetCompletedBoundaryState state) {
    switch (state) {
        case FrenetCompletedBoundaryState::kObserved: return "observed";
        case FrenetCompletedBoundaryState::kInferred: return "inferred";
        case FrenetCompletedBoundaryState::kConnected: return "connected";
    }
    return "observed";
}

FrenetCompletedBoundaryResult FrenetCompletedBoundaryBuilder::build(
    const BoundarySamplingResult& samples,
    const FrenetTrackResult& tracks,
    const FrenetCompletionResult& completion) const {
    FrenetCompletedBoundaryResult result;
    result.frame_id = samples.frame_id;
    result.nodes_by_slice.resize(samples.slices.size());
    if (!samples.ok || !tracks.ok || !completion.ok) {
        result.error = !samples.ok ? samples.error : (!tracks.ok ? tracks.error : completion.error);
        if (result.error.empty()) result.error = "invalid_completed_boundary_input";
        return result;
    }

    std::map<std::string, const FrenetTrack*> tracks_by_id;
    DisjointSet components;
    for (const auto& track : tracks.tracks) {
        if (!isLaneBoundaryType(track.type)) continue;
        tracks_by_id[track.id] = &track;
        components.add(track.id);
    }
    for (const auto& link : completion.links) {
        if (!isEntityConnector(link)) continue;
        const auto separator = link.track_id.find('|');
        if (separator == std::string::npos) continue;
        const std::string from_id = link.track_id.substr(0, separator);
        const std::string to_id = link.track_id.substr(separator + 1);
        if (tracks_by_id.count(from_id) == 0 || tracks_by_id.count(to_id) == 0) continue;
        components.unite(from_id, to_id);
        result.connectors.push_back({from_id, to_id, link.from_s, link.from_l, link.to_s, link.to_l});
    }

    std::map<std::string, FrenetCompletedBoundaryTrack> completed_by_id;
    for (const auto& [source_id, track] : tracks_by_id) {
        const std::string completed_id = components.find(source_id);
        result.completed_id_by_source_track_id[source_id] = completed_id;
        auto& completed = completed_by_id[completed_id];
        completed.id = completed_id;
        completed.type = track->type;
        completed.source_type = track->source_type;
        completed.source_track_ids.push_back(source_id);
    }

    auto addNode = [&](const std::string& source_id, int slice_index, double s, double l,
                       const std::string& lane_type, double confidence,
                       FrenetCompletedBoundaryState state) {
        if (slice_index < 0 || static_cast<std::size_t>(slice_index) >= result.nodes_by_slice.size()) return;
        const auto track_it = tracks_by_id.find(source_id);
        if (track_it == tracks_by_id.end()) return;
        const std::string completed_id = result.completed_id_by_source_track_id[source_id];
        FrenetCompletedBoundaryNode node;
        node.completed_track_id = completed_id;
        node.source_track_id = source_id;
        node.type = track_it->second->type;
        node.source_type = track_it->second->source_type;
        node.lane_type = lane_type.empty() ? track_it->second->lane_type_summary : lane_type;
        node.slice_index = slice_index;
        node.s = s;
        node.l = l;
        node.state = state;
        node.confidence = confidence;
        result.nodes_by_slice[static_cast<std::size_t>(slice_index)].push_back(node);
        completed_by_id[completed_id].nodes.push_back(std::move(node));
    };

    for (const auto& [source_id, track] : tracks_by_id) {
        for (const auto& sample : track->samples) {
            addNode(source_id, sample.slice_index, sample.s, sample.l, sample.lane_type,
                    sample.confidence, FrenetCompletedBoundaryState::kObserved);
        }
    }
    for (const auto& node : completion.inferred_nodes) {
        // Merged nodes represent several hypotheses; they cannot be assigned to one boundary.
        if (node.track_id.find('|') != std::string::npos) continue;
        addNode(node.track_id, node.slice_index, node.s, node.l, node.lane_type,
                node.confidence, FrenetCompletedBoundaryState::kInferred);
    }

    // A validated entity connector makes two FT segments one boundary entity. Fill only
    // missing slices between its endpoints; stop nodes remain terminal evidence, not geometry.
    for (const auto& connector : result.connectors) {
        const auto from_it = result.completed_id_by_source_track_id.find(connector.from_track_id);
        if (from_it == result.completed_id_by_source_track_id.end()) continue;
        const double length = connector.to_s - connector.from_s;
        if (length <= 1e-6) continue;
        for (std::size_t si = 0; si < samples.slices.size(); ++si) {
            const auto& slice = samples.slices[si];
            if (slice.s <= connector.from_s || slice.s >= connector.to_s) continue;
            auto& nodes = result.nodes_by_slice[si];
            const bool occupied = std::any_of(nodes.begin(), nodes.end(), [&](const auto& node) {
                return node.completed_track_id == from_it->second;
            });
            if (occupied) continue;
            const double ratio = (slice.s - connector.from_s) / length;
            addNode(connector.from_track_id, static_cast<int>(si), slice.s,
                    connector.from_l + ratio * (connector.to_l - connector.from_l), "", 0.35,
                    FrenetCompletedBoundaryState::kConnected);
        }
    }

    for (auto& [_, completed] : completed_by_id) {
        std::sort(completed.nodes.begin(), completed.nodes.end(), [](const auto& a, const auto& b) {
            return a.s < b.s;
        });
        result.tracks.push_back(std::move(completed));
    }
    for (auto& nodes : result.nodes_by_slice) {
        std::sort(nodes.begin(), nodes.end(), [](const auto& a, const auto& b) {
            if (a.l != b.l) return a.l < b.l;
            return a.completed_track_id < b.completed_track_id;
        });
    }
    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
