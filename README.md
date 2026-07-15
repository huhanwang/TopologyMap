# TopologyMap

TopologyMap is an offline lane-topology analysis and visualization workspace for fused static lane data. It builds a visual reference line, samples boundaries in Frenet space, completes lane-line observations, and exports debug JSON for inspecting lane regions, ribbons, junctions, and topology-related features.

## Layout

- `algorithms/` - C++ algorithm modules.
  - `fused_reference/` builds the fused visual/navigation reference line.
  - `boundary_sampling/` samples lane, edge, and curb intersections along the reference line.
  - `frenet/` contains Frenet projection, track/ribbon building, completion, junction analysis, and lane-region candidates.
  - `navigation_route/`, `visual_reference/`, and `proto_preprocess/` provide replay preprocessing and reference inputs.
- `offline_replay/` - C++ offline replay executable and debug JSON exporters.
- `viewer/` - browser-based BEV/Frenet debug viewer.
- `docs/` - design notes and topology reasoning documents.

## Build

```bash
cmake --build offline_replay/build --target offline_replay -j4
```

The existing CMake setup expects local project dependencies from the parent workspace, including protocol headers/libraries and visualization backend generated proto libraries.

## Run Offline Replay

```bash
offline_replay/build/offline_replay offline_replay/configs/fusedstatic_gnss_sdroute.json
```

Default output:

```text
offline_replay/out/fusedstatic_gnss_sdroute_0601_rtk
```

Large generated outputs, build products, databases, and binaries are intentionally ignored by git.

## Viewer

Serve the repository root and open:

```text
http://127.0.0.1:8000/viewer/
```

The viewer supports BEV and Frenet modes, including overlays for:

- fused/visual/navigation references
- raw boundary intersections
- Frenet tracks and ribbons
- lane centers and lane tracks
- boundary completion
- lane region candidates

## Debug Outputs

Per-frame debug JSON includes:

- `fused_reference.json`
- `boundary_intersections.json`
- `frenet_topology_debug.json`
- `boundary_completion_debug.json`
- `lane_region_debug.json`
- `lane_center_debug.json`

`lane_region_debug.json` is built from completed Frenet boundary tracks and is intended as the next abstraction layer before topology event reasoning.

## Notes

This repository tracks source, configuration, viewer code, and design notes only. Replay outputs under `offline_replay/out/` and build artifacts under `offline_replay/build/` are excluded.
