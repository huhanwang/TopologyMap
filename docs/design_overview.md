# TopologyMap C++ Engineering Design

Date: 2026-07-09

Status: initial architecture draft

## 1. Project Goal

TopologyMap is a new C++ real-time road modeling project. Its goal is to build a
lane-level local road model from online perception and map/navigation inputs
using topology-first reconstruction.

The target deployment environment is mobile or domain-controller runtime, so the
algorithm core must be written as a deterministic C++ library instead of a
Python visualization pipeline.

Primary inputs:

- visual lane and boundary observations;
- navigation path;
- GNSS/localization pose;
- SD Pro map hints, added later;
- optional multi-frame temporal evidence, added later.

Primary output:

- a topology-consistent local RoadModel;
- lane boundaries, lane corridors, topology events, and confidence metadata;
- debug geometry for offline visualization, without changing algorithm output.

## 2. Core Principle

The algorithm is not a lane-section stitching system. It is a topology
reconstruction system.

Main principle:

```text
Build a reliable fused reference line.
Sample raw boundary intersections on key slices.
Regularize boundary nodes/tracks with sliding-window evidence.
Infer topology state and events.
Construct lane sections from regularized boundary roles.
Associate lane sections under topology constraints.
Assign ego-relative slots only at the end.
Export RoadModel.
```

Stable topology has inertia. Short missing detections, short false positives,
line-id switches, and temporary curb/lane-line ambiguity should first be treated
as observation defects. A topology event such as lane add, lane drop, merge, or
split requires sustained structural evidence.

## 3. Reference Line Is the First Critical Module

The fused reference line is the scan coordinate system. It is not the final lane
centerline and does not create lane boundaries.

The visual reference must not be a synthetic average of all visual points. It
must be selected from actual visual boundary evidence, following the legacy
algorithm's validated strategy:

1. Extract visual boundary lines from the preferred source.
2. Select a visual reference candidate:
   - prefer `HOST_LEFT + HOST_RIGHT` as `host_pair`;
   - otherwise use a geometrically valid ego-covering pair as
     `geometric_ego_pair`;
   - otherwise use the best long boundary as `longest_boundary_anchor`.
3. If navigation fusion is available, replace the reference geometry with the
   visual/navigation fused points, but preserve the visual reference anchor
   diagnostics:
   - left/right visual line ids;
   - base reference method;
   - selected visual source;
   - source line count;
   - confidence.

Navigation and GNSS provide road-level alignment, heading, curvature, and
extension. They do not directly create boundary nodes.

## 4. High-Level Pipeline

```text
FrameInput + NavigationInput + LocalizationInput
  -> VisualReferenceSelector
  -> NavigationReferenceBuilder
  -> ReferenceAligner
  -> FusedReferenceBuilder
  -> KeySliceSampler
  -> RawBoundaryGraph
  -> BoundaryRegularizer
  -> TopologyStateMachine
  -> LaneSectionBuilder
  -> LaneSectionAssociator
  -> SlotAssigner
  -> RoadModelBuilder
  -> RoadModel
```

## 5. Module Responsibilities

### 5.1 Input Layer

Classes:

```cpp
class VisualInputAdapter;
class NavigationInputAdapter;
class LocalizationInputAdapter;
class SdProInputAdapter;
```

Responsibilities:

- normalize source data into internal structs;
- keep original ids, source type, confidence, and timestamps;
- avoid topology or lane decisions.

Key structs:

```cpp
struct VisualBoundaryLine;
struct NavigationPath;
struct LocalizationPose;
struct SdProHint;
struct FrameInput;
```

### 5.2 Reference Layer

Classes:

```cpp
class VisualReferenceSelector;
class NavigationReferenceBuilder;
class ReferenceAligner;
class FusedReferenceBuilder;
```

Responsibilities:

- select actual visual reference candidates from visual lines;
- build local navigation reference from navigation path and GNSS;
- align navigation to visual reference;
- generate the fused reference used for key-slice scanning.

Important outputs:

```cpp
struct VisualReferenceCandidate {
  BoundaryLineId left_line_id;
  BoundaryLineId right_line_id;
  ReferenceMethod method;  // host_pair, geometric_ego_pair, longest_boundary_anchor
  Polyline2D geometry;
  Polynomial1D center_coeffs;
  Range1D s_range;
  double confidence;
};

struct FusedReferenceLine {
  Polyline2D points;
  Range1D s_range;
  ReferenceDiagnostics diagnostics;
};
```

### 5.3 Raw Key Slice Sampling

Classes:

```cpp
class KeySliceSampler;
class RawIntersectionSampler;
```

Responsibilities:

- sample key slices along the fused reference;
- intersect each key slice normal with all raw visual boundary polylines;
- output all raw intersections.

This layer must not:

- remove curb nodes;
- merge near duplicates;
- reject abnormal widths;
- infer missing nodes;
- decide topology.

Output:

```cpp
struct RawBoundaryNode {
  double s;
  double offset;
  Point2D xy;
  BoundaryLineId source_line_id;
  SourceType source_type;
  double confidence;
};

struct RawBoundaryGraph {
  FusedReferenceLine reference;
  std::vector<KeySlice> slices;
};
```

### 5.4 Boundary Regularization

Classes:

```cpp
class BoundaryTrackBuilder;
class BoundaryRegularizer;
class BoundaryGapRepairer;
class BoundaryNodeClassifier;
```

Responsibilities:

- use sliding-window statistics to align raw nodes across slices;
- build boundary tracks;
- suppress short false nodes;
- handle near-duplicate nodes;
- repair short missing boundary nodes when topology is stable;
- preserve diagnostic states: observed, inferred, suppressed, weak, occluded.

This layer operates on boundary nodes and boundary tracks, not lane tracks.

### 5.5 Topology State Machine

Classes:

```cpp
class TopologyStateMachine;
class TopologyEventDetector;
class TopologyStabilizer;
```

Responsibilities:

- infer stable lane count and boundary roles;
- distinguish observation defects from real topology events;
- output lane add/drop/merge/split/open-area candidates only with sustained
  evidence.

Output:

```cpp
struct TopologyState {
  Range1D s_range;
  int lane_count;
  int boundary_count;
  std::vector<BoundaryRole> roles;
  double confidence;
};

struct TopologyEvent {
  TopologyEventType type;
  Range1D s_range;
  double confidence;
};
```

### 5.6 Lane Construction and Association

Classes:

```cpp
class LaneSectionBuilder;
class LaneSectionAssociator;
class SlotAssigner;
```

Responsibilities:

- construct lane sections from adjacent regularized boundary roles;
- associate sections into lane tracks under topology constraints;
- assign ego-relative slots after physical lane tracks are formed.

Ego identity must not be used as the primary evidence for physical lane
continuity.

### 5.7 RoadModel and Compiler

Classes:

```cpp
class RoadModelBuilder;
class RoadModelValidator;
class RoadSceneCompiler;
```

Responsibilities:

- convert topology, boundary tracks, and lane tracks into RoadModel;
- validate consistency;
- produce compiled geometry for downstream consumers.

The compiler must not patch topology defects. It consumes RoadModel.

## 6. Debug and Visualization Contract

Debug export is required, but it is not part of the algorithm decision path.

Debug layers should include:

- raw visual boundary lines;
- visual reference candidate;
- navigation reference;
- fused reference;
- key slices;
- raw slice intersections;
- regularized boundary graph;
- topology states/events;
- lane section tracks;
- final RoadModel.

Visualization must show what the algorithm produced. It must not repair or hide
algorithm defects.

## 7. Initial C++ Directory Plan

```text
TopologyMap/
  CMakeLists.txt
  docs/
    design_overview.md
  include/topology_map/
    core/
      geometry.hpp
      types.hpp
      status.hpp
    input/
      visual_input_adapter.hpp
      navigation_input_adapter.hpp
      localization_input_adapter.hpp
    reference/
      visual_reference_selector.hpp
      navigation_reference_builder.hpp
      reference_aligner.hpp
      fused_reference_builder.hpp
    sampling/
      key_slice_sampler.hpp
      raw_intersection_sampler.hpp
    boundary/
      boundary_graph.hpp
      boundary_regularizer.hpp
      boundary_track_builder.hpp
    topology/
      topology_state_machine.hpp
      topology_event_detector.hpp
    lane/
      lane_section_builder.hpp
      lane_section_associator.hpp
      slot_assigner.hpp
    road_model/
      road_model.hpp
      road_model_builder.hpp
      road_model_validator.hpp
    debug/
      debug_exporter.hpp
  src/
    ...
  tests/
    ...
```

## 8. First Implementation Milestone

Milestone 1 should not implement full lane association. It should lock down the
reference and raw sampling foundation:

1. define core geometry and input structs;
2. port the legacy visual reference selection logic;
3. build navigation reference and visual/navigation fused reference interfaces;
4. sample key slices along fused reference;
5. output raw boundary intersections;
6. export debug JSON for 9284 and 9712 comparison.

Acceptance criteria:

- visual reference is selected from real visual boundary lines;
- fused reference preserves visual anchor diagnostics;
- key slices are generated from the fused reference;
- raw intersections include all visual-line hits without cleanup filtering;
- debug output makes reference-line failures visible before topology logic runs.
