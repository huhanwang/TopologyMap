#pragma once

#include <nlohmann/json.hpp>

#include "boundary_sampling/boundary_sampling_module.h"
#include "offline_replay/replay_types.h"
#include "fused_reference/fused_reference_module.h"
#include "navigation_route/navigation_route_tracker.h"
#include "proto_preprocess/proto_preprocess_module.h"
#include "visual_reference/visual_reference_module.h"

namespace offline_replay::debug {

nlohmann::json preprocessedSnapshotToJson(
    const offline_replay::algorithms::PreprocessedSnapshot& preprocessed);

nlohmann::json buildReplayVizJson(
    const SnapshotFrame& snapshot,
    const offline_replay::algorithms::PreprocessedSnapshot& preprocessed);

nlohmann::json sdRouteDebugToJson(
    const SnapshotFrame& snapshot,
    const offline_replay::algorithms::PreprocessedSnapshot& preprocessed);

nlohmann::json navigationTrackerDebugToJson(
    const topology_map::algorithms::NavigationRouteSnapshot& route,
    const topology_map::algorithms::NavigationMatchResult& match);

nlohmann::json navigationReferenceToJson(
    const topology_map::algorithms::NavigationReferenceResult& result);

nlohmann::json navigationReferenceVizLayer(
    const topology_map::algorithms::NavigationReferenceResult& result);

nlohmann::json fusedReferenceToJson(
    const topology_map::algorithms::FusedReferenceResult& result);

nlohmann::json fusedReferenceVizLayer(
    const topology_map::algorithms::FusedReferenceResult& result);

nlohmann::json boundaryIntersectionsToJson(
    const topology_map::algorithms::BoundarySamplingResult& result);

nlohmann::json boundaryIntersectionsVizLayer(
    const topology_map::algorithms::BoundarySamplingResult& result);

nlohmann::json frenetTopologyDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result);

nlohmann::json laneCenterDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result);

nlohmann::json boundaryCompletionDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result);

nlohmann::json laneRegionDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result);

nlohmann::json visualReferenceToJson(
    const topology_map::algorithms::VisualReferenceResult& result);

nlohmann::json visualReferenceVizLayer(
    const topology_map::algorithms::VisualReferenceResult& result);

}  // namespace offline_replay::debug
