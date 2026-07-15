const state = {
  datasetPath: "/offline_replay/out/fusedstatic_gnss_sdroute_0601_rtk",
  dataset: null,
  frameIndex: 0,
  current: {
    frame: null,
    viz: null,
    navigationTracker: null,
    navigationReference: null,
    boundaryIntersections: null,
    frenetTopologyDebug: null,
    laneCenterDebug: null,
    boundaryCompletionDebug: null,
    laneRegionDebug: null,
  },
  sceneMode: "bev",
  view: {
    x: 0,
    y: 0,
    scale: 8,
    dragging: false,
    lastX: 0,
    lastY: 0,
  },
  frenetView: {
    x: 0,
    y: 0,
    scale: 8,
    dragging: false,
    lastX: 0,
    lastY: 0,
  },
};

const elements = {
  datasetPath: document.getElementById("datasetPath"),
  loadDataset: document.getElementById("loadDataset"),
  bundleMeta: document.getElementById("bundleMeta"),
  frameProgress: document.getElementById("frameProgress"),
  frameProgressText: document.getElementById("frameProgressText"),
  prevFrame: document.getElementById("prevFrame"),
  nextFrame: document.getElementById("nextFrame"),
  frameJumpInput: document.getElementById("frameJumpInput"),
  frameJumpButton: document.getElementById("frameJumpButton"),
  frameJumpStatus: document.getElementById("frameJumpStatus"),
  modeBev: document.getElementById("modeBev"),
  modeFrenet: document.getElementById("modeFrenet"),
  showVehicle: document.getElementById("showVehicle"),
  showRoute: document.getElementById("showRoute"),
  showNavigationTracker: document.getElementById("showNavigationTracker"),
  showNavigationReference: document.getElementById("showNavigationReference"),
  showFusedReference: document.getElementById("showFusedReference"),
  showVisualReference: document.getElementById("showVisualReference"),
  showRawIntersection: document.getElementById("showRawIntersection"),
  showFrenetTracks: document.getElementById("showFrenetTracks"),
  showFrenetRibbons: document.getElementById("showFrenetRibbons"),
  showRibbonCenters: document.getElementById("showRibbonCenters"),
  showLaneCenters: document.getElementById("showLaneCenters"),
  showLaneTracks: document.getElementById("showLaneTracks"),
  showLaneRegions: document.getElementById("showLaneRegions"),
  showBoundaryCompletion: document.getElementById("showBoundaryCompletion"),
  showLabels: document.getElementById("showLabels"),
  showPoints: document.getElementById("showPoints"),
  fitView: document.getElementById("fitView"),
  resetView: document.getElementById("resetView"),
  frameStats: document.getElementById("frameStats"),
  navigationStatus: document.getElementById("navigationStatus"),
  debugPanel: document.getElementById("debugPanel"),
  sceneCanvas: document.getElementById("sceneCanvas"),
  sceneMeta: document.getElementById("sceneMeta"),
};

const ctx = elements.sceneCanvas.getContext("2d", { alpha: false });
const CHECKBOX_STORAGE_KEY = "offlineReplayViewer.checkboxes";
const checkboxElements = [
  "showVehicle",
  "showRoute",
  "showNavigationTracker",
  "showNavigationReference",
  "showFusedReference",
  "showVisualReference",
  "showRawIntersection",
  "showFrenetTracks",
  "showFrenetRibbons",
  "showRibbonCenters",
  "showLaneCenters",
  "showLaneTracks",
  "showLaneRegions",
  "showBoundaryCompletion",
  "showLabels",
  "showPoints",
];

function bindEvents() {
  restoreCheckboxState();
  elements.loadDataset.addEventListener("click", () => loadDataset(elements.datasetPath.value));
  elements.frameProgress.addEventListener("input", () => selectFrame(Number(elements.frameProgress.value || 0)));
  elements.prevFrame.addEventListener("click", () => selectFrame(state.frameIndex - 1));
  elements.nextFrame.addEventListener("click", () => selectFrame(state.frameIndex + 1));
  elements.frameJumpButton.addEventListener("click", jumpToFrame);
  elements.frameJumpInput.addEventListener("keydown", (event) => {
    if (event.key === "Enter") jumpToFrame();
  });
  elements.modeBev.addEventListener("click", () => setSceneMode("bev"));
  elements.modeFrenet.addEventListener("click", () => setSceneMode("frenet"));
  checkboxElements.forEach((key) => {
    elements[key].addEventListener("change", () => {
      saveCheckboxState();
      render();
    });
  });
  elements.fitView.addEventListener("click", fitView);
  elements.resetView.addEventListener("click", () => {
    const view = currentView();
    view.x = 0;
    view.y = 0;
    view.scale = 8;
    render();
  });
  elements.sceneCanvas.addEventListener("wheel", onWheel, { passive: false });
  elements.sceneCanvas.addEventListener("pointerdown", onPointerDown);
  elements.sceneCanvas.addEventListener("pointermove", onPointerMove);
  elements.sceneCanvas.addEventListener("pointerup", onPointerUp);
  elements.sceneCanvas.addEventListener("pointercancel", onPointerUp);
  elements.sceneCanvas.addEventListener("dblclick", fitView);
  window.addEventListener("resize", render);
}

function setSceneMode(mode) {
  state.sceneMode = mode === "frenet" ? "frenet" : "bev";
  elements.modeBev.classList.toggle("is-active", state.sceneMode === "bev");
  elements.modeFrenet.classList.toggle("is-active", state.sceneMode === "frenet");
  render();
}

function restoreCheckboxState() {
  let saved = null;
  try {
    saved = JSON.parse(localStorage.getItem(CHECKBOX_STORAGE_KEY) || "null");
  } catch (_) {
    saved = null;
  }
  if (!saved || typeof saved !== "object") return;
  checkboxElements.forEach((key) => {
    if (typeof saved[key] === "boolean") elements[key].checked = saved[key];
  });
}

function saveCheckboxState() {
  const values = {};
  checkboxElements.forEach((key) => {
    values[key] = elements[key].checked;
  });
  try {
    localStorage.setItem(CHECKBOX_STORAGE_KEY, JSON.stringify(values));
  } catch (_) {}
}

async function loadDataset(path) {
  state.datasetPath = path.replace(/\/$/, "");
  elements.datasetPath.value = state.datasetPath;
  const dataset = await fetchJson(`${state.datasetPath}/dataset.js`, true);
  state.dataset = dataset;
  const frameCount = dataset.frames?.length || 0;
  elements.frameProgress.min = "0";
  elements.frameProgress.max = String(Math.max(0, frameCount - 1));
  elements.bundleMeta.textContent = `${dataset.manifest?.main_topic || "dataset"} · ${frameCount} frames`;
  await selectFrame(0);
}

async function fetchJson(url, jsDataset = false) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}: ${url}`);
  const text = await response.text();
  if (!jsDataset) return JSON.parse(text);
  const prefix = "window.REPLAY_DATASET = ";
  const trimmed = text.trim();
  if (trimmed.startsWith(prefix)) {
    return JSON.parse(trimmed.slice(prefix.length).replace(/;$/, ""));
  }
  return JSON.parse(trimmed);
}

async function selectFrame(index) {
  if (!state.dataset?.frames?.length) return;
  const max = state.dataset.frames.length - 1;
  state.frameIndex = Math.max(0, Math.min(index, max));
  const item = state.dataset.frames[state.frameIndex];
  const files = item.files || {};
  const base = state.datasetPath;
  const [frame, viz, navigationTracker, navigationReference, boundaryIntersections, frenetTopologyDebug, laneCenterDebug, boundaryCompletionDebug, laneRegionDebug] = await Promise.all([
    fetchJson(`${base}/${files.frame}`),
    fetchJson(`${base}/${files.viz}`),
    files.navigation_tracker ? fetchJson(`${base}/${files.navigation_tracker}`) : Promise.resolve(null),
    files.navigation_reference ? fetchJson(`${base}/${files.navigation_reference}`) : Promise.resolve(null),
    files.boundary_intersections ? fetchJson(`${base}/${files.boundary_intersections}`) : Promise.resolve(null),
    files.frenet_topology_debug ? fetchJson(`${base}/${files.frenet_topology_debug}`) : Promise.resolve(null),
    files.lane_center_debug ? fetchJson(`${base}/${files.lane_center_debug}`) : Promise.resolve(null),
    files.boundary_completion_debug ? fetchJson(`${base}/${files.boundary_completion_debug}`) : Promise.resolve(null),
    files.lane_region_debug ? fetchJson(`${base}/${files.lane_region_debug}`) : Promise.resolve(null),
  ]);
  state.current = { frame, viz, navigationTracker, navigationReference, boundaryIntersections, frenetTopologyDebug, laneCenterDebug, boundaryCompletionDebug, laneRegionDebug };
  elements.frameProgress.value = String(state.frameIndex);
  elements.frameProgressText.textContent = `${state.frameIndex + 1} / ${state.dataset.frames.length}`;
  renderStats();
  renderNavigationStatus();
  render();
}

async function jumpToFrame() {
  const frameId = Number(elements.frameJumpInput.value);
  if (!Number.isFinite(frameId) || !state.dataset?.frames) {
    elements.frameJumpStatus.textContent = "无效帧号";
    return;
  }
  const frames = state.dataset.frames;
  let idx = frames.findIndex((f) => Number(f.main_frame_id) === frameId);
  let exact = true;
  if (idx < 0) {
    exact = false;
    idx = findNearestFrameIndex(frameId);
  }
  if (idx < 0) {
    elements.frameJumpStatus.textContent = `未找到 ${frameId}`;
    return;
  }
  await selectFrame(idx);
  const actualFrameId = state.dataset.frames[idx]?.main_frame_id ?? "-";
  elements.frameJumpStatus.textContent = exact
    ? `已跳转 ${actualFrameId}`
    : `未找到 ${frameId}，已跳到最近帧 ${actualFrameId}`;
}

function findNearestFrameIndex(frameId) {
  const frames = state.dataset?.frames || [];
  if (!frames.length) return -1;
  let bestIndex = -1;
  let bestDelta = Infinity;
  for (let i = 0; i < frames.length; ++i) {
    const candidate = Number(frames[i].main_frame_id);
    if (!Number.isFinite(candidate)) continue;
    const delta = Math.abs(candidate - frameId);
    if (delta < bestDelta) {
      bestDelta = delta;
      bestIndex = i;
    }
  }
  return bestIndex;
}

function renderStats() {
  const frame = state.current.frame || {};
  const viz = state.current.viz || {};
  const nav = state.current.navigationTracker || {};
  const navRef = state.current.navigationReference || {};
  const boundary = state.current.boundaryIntersections || {};
  const frenet = state.current.frenetTopologyDebug || {};
  const laneCenter = state.current.laneCenterDebug || {};
  const completion = state.current.boundaryCompletionDebug || {};
  const regions = state.current.laneRegionDebug || {};
  const match = nav.match || {};
  const layer = (viz.layers || [])[0] || {};
  const itemCount = (layer.items || []).length;
  const rows = [
    ["Index", frame.index ?? "-"],
    ["Frame ID", frame.main_frame_id ?? "-"],
    ["Timestamp", frame.main_time_us ?? "-"],
    ["Entries", frame.entry_count ?? "-"],
    ["Viz items", itemCount],
    ["Vehicle lanes", viz.debug?.vehicle_bev_lanes_size ?? "-"],
    ["Route items", viz.debug?.sd_route_vcs?.projected_items ?? 0],
    ["Raw slices", boundary.ok ? boundary.slice_count : "-"],
    ["Raw hits", boundary.ok ? boundary.hit_count : "-"],
    ["Frenet tracks", frenet.ok ? frenet.diagnostics?.track_count ?? "-" : "-"],
    ["Ribbon tracks", frenet.ok ? frenet.diagnostics?.ribbon_track_count ?? "-" : "-"],
    ["Lane centers", laneCenter.ok ? laneCenter.diagnostics?.lane_center_track_count ?? "-" : "-"],
    ["Lane tracks", laneCenter.ok ? laneCenter.diagnostics?.lane_track_count ?? "-" : "-"],
    ["Completion", completion.ok ? completion.diagnostics?.merged_inferred_node_count ?? "-" : "-"],
    ["Lane regions", regions.ok ? `${regions.diagnostics?.candidate_region_count ?? 0} / ${regions.diagnostics?.region_count ?? 0}` : "-"],
    ["Frenet runs", frenet.ok ? frenet.diagnostics?.stable_run_count ?? "-" : "-"],
    ["Nav segment", match.matched ? match.segment_index : "-"],
    ["Nav ref", navRef.ok ? `${navRef.forward_length_m?.toFixed?.(1) ?? "-"} m` : "off"],
  ];
  elements.frameStats.innerHTML = rows.map(([k, v]) => `<dt>${escapeHtml(k)}</dt><dd>${escapeHtml(v)}</dd>`).join("");
  elements.debugPanel.textContent = JSON.stringify({
    viz_debug: viz.debug,
    navigation_match: match,
  }, null, 2);
}

function renderNavigationStatus() {
  const nav = state.current.navigationTracker;
  if (!nav) {
    elements.navigationStatus.innerHTML = `<dt>Status</dt><dd>no navigation_tracker.json</dd>`;
    return;
  }
  const match = nav.match || {};
  const navRef = state.current.navigationReference || {};
  const event = match.current_terminal_event || {};
  const rows = [
    ["Matched", match.matched ? "true" : "false"],
    ["Segment", match.matched ? match.segment_index : "-"],
    ["Use Ref", match.can_use_navigation_reference ? "true" : "false"],
    ["Near End", match.near_segment_end ? "true" : "false"],
    ["End Dist", Number.isFinite(match.distance_to_segment_end_m) ? `${match.distance_to_segment_end_m.toFixed(1)} m` : "-"],
    ["Lat Err", Number.isFinite(match.lateral_error_m) ? `${match.lateral_error_m.toFixed(1)} m` : "-"],
    ["Event", event.type || "-"],
    ["Ref", navRef.ok ? `${navRef.backward_length_m?.toFixed?.(1) ?? "-"} / ${navRef.forward_length_m?.toFixed?.(1) ?? "-"} m` : (navRef.error || "off")],
    ["Instruction", event.instruction || "-"],
  ];
  elements.navigationStatus.innerHTML = rows
    .map(([k, v]) => `<dt>${escapeHtml(k)}</dt><dd>${escapeHtml(v)}</dd>`)
    .join("");
}

function render() {
  resizeCanvas();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#101214";
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);

  if (state.sceneMode === "frenet") {
    renderFrenet();
    return;
  }

  const items = visibleItems();
  drawGrid();
  for (const item of items) drawPolyline(item);
  drawNavigationTrackerOverlay();
  if (elements.showPoints.checked) {
    for (const item of items) drawPoints(item);
  }
  if (elements.showLabels.checked) {
    for (const item of items) drawLabel(item);
  }

  const frameId = state.current.frame?.main_frame_id ?? "-";
  const nav = state.current.navigationTracker?.match;
  const navText = nav?.matched
    ? ` · nav seg ${nav.segment_index} · end ${nav.distance_to_segment_end_m.toFixed(1)}m`
    : "";
  elements.sceneMeta.textContent = `frame ${frameId} · ${items.length} lines${navText} · scale ${state.view.scale.toFixed(2)}`;
}

function renderFrenet() {
  const points = frenetPoints();
  const debug = state.current.frenetTopologyDebug;
  drawGrid();
  drawFrenetReferenceLines();
  drawFrenetStableRuns(debug);
  drawFrenetRibbons(debug);
  drawFrenetTracks(debug, points);
  drawRibbonCenters(debug);
  drawLaneCenters(state.current.laneCenterDebug);
  drawLaneTracks(state.current.laneCenterDebug);
  drawLaneRegions(state.current.laneRegionDebug);
  drawFrenetPoints(points);
  drawBoundaryCompletion(state.current.boundaryCompletionDebug);
  if (elements.showLabels.checked && elements.showRawIntersection.checked) {
    drawFrenetLabels(points);
  }
  drawFrenetStableRunAxisLabels(debug);

  const frameId = state.current.frame?.main_frame_id ?? "-";
  const tracks = debug?.ok ? debug.tracks?.length || 0 : new Set(points.map((point) => point.sourceLineId)).size;
  const runs = debug?.ok ? debug.stable_runs?.length || 0 : 0;
  const laneCenters = state.current.laneCenterDebug?.ok
    ? state.current.laneCenterDebug.lane_center_tracks?.length || 0
    : 0;
  const laneTracks = state.current.laneCenterDebug?.ok
    ? state.current.laneCenterDebug.lane_tracks?.length || 0
    : 0;
  const completion = state.current.boundaryCompletionDebug?.ok
    ? state.current.boundaryCompletionDebug.diagnostics?.merged_inferred_node_count || 0
    : 0;
  const regions = state.current.laneRegionDebug?.ok
    ? state.current.laneRegionDebug.diagnostics?.candidate_region_count || 0
    : 0;
  elements.sceneMeta.textContent =
    `frame ${frameId} · Frenet topology debug · ${points.length} points · ${tracks} tracks · ${laneCenters} LC · ${laneTracks} LT · ${regions} regions · ${completion} inferred · ${runs} runs · scale ${currentView().scale.toFixed(2)}`;
}

function visibleItems() {
  const layers = state.current.viz?.layers || [];
  const out = [];
  for (const layer of layers) {
    for (const item of layer.items || []) {
      const source = item.properties?.source || "";
      const isRawIntersection =
        layer.id === "raw_intersection" ||
        layer.id === "boundary_intersections" ||
        source === "raw_intersection_module" ||
        source === "boundary_intersections_module";
      if (source === "vehicle_bev_lanes" && !elements.showVehicle.checked) continue;
      if (source.startsWith("AutoSDRoute") && !elements.showRoute.checked) continue;
      if (source === "navigation_reference_module" && !elements.showNavigationReference.checked) continue;
      if (source === "fused_reference_module" && !elements.showFusedReference.checked) continue;
      if (source === "visual_reference_module" && !elements.showVisualReference.checked) continue;
      if (isRawIntersection && !elements.showRawIntersection.checked) continue;
      if (item.type === "polyline" && item.points?.length >= 2) out.push(item);
    }
  }
  return out;
}

function resizeCanvas() {
  const rect = elements.sceneCanvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width * dpr));
  const height = Math.max(1, Math.floor(rect.height * dpr));
  if (elements.sceneCanvas.width !== width || elements.sceneCanvas.height !== height) {
    elements.sceneCanvas.width = width;
    elements.sceneCanvas.height = height;
  }
}

function currentView() {
  return state.sceneMode === "frenet" ? state.frenetView : state.view;
}

function worldToScreen(point) {
  const dpr = window.devicePixelRatio || 1;
  const w = elements.sceneCanvas.width;
  const h = elements.sceneCanvas.height;
  const view = currentView();
  const screenX = w / 2 - (point[1] - view.y) * view.scale * dpr;
  const screenY = h / 2 - (point[0] - view.x) * view.scale * dpr;
  return [screenX, screenY];
}

function screenToWorld(x, y) {
  const dpr = window.devicePixelRatio || 1;
  const w = elements.sceneCanvas.width;
  const h = elements.sceneCanvas.height;
  const view = currentView();
  return [
    (h / 2 - y * dpr) / (view.scale * dpr) + view.x,
    view.y - (x * dpr - w / 2) / (view.scale * dpr),
  ];
}

function drawPolyline(item) {
  const points = item.points || [];
  if (points.length < 2) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = worldToScreen(point);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = item.style?.color || "#f2c94c";
  ctx.lineWidth = Math.max(1, (item.style?.width || 0.06) * currentView().scale * dpr);
  ctx.setLineDash(item.style?.dash ? [8, 8] : []);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawPoints(item) {
  ctx.fillStyle = "#ffffff";
  for (const point of item.points || []) {
    const [x, y] = worldToScreen(point);
    ctx.beginPath();
    ctx.arc(x, y, 2.5, 0, Math.PI * 2);
    ctx.fill();
  }
}

function drawLabel(item) {
  const first = item.points?.[0];
  if (!first) return;
  const [x, y] = worldToScreen(first);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#d8dde3";
  ctx.font = "12px sans-serif";
  ctx.fillText(item.properties?.lane_id || item.id, x + 4, y - 4);
}

function frenetPoints() {
  const raw = state.current.boundaryIntersections;
  if (!raw?.ok) return [];
  const points = [];
  for (const slice of raw.slices || []) {
    for (const hit of slice.hits || []) {
      points.push({
        s: Number(hit.s ?? slice.s),
        offset: Number(hit.offset_m),
        sourceType: hit.source_type || "unknown",
        sourceLineId: hit.source_line_id || hit.id || "unknown",
        laneId: hit.lane_id,
        laneIndex: hit.lane_index,
        sectionIndex: hit.section_index,
      });
    }
  }
  return points.filter((point) => Number.isFinite(point.s) && Number.isFinite(point.offset));
}

function drawFrenetReferenceLines() {
  const dpr = window.devicePixelRatio || 1;
  const rect = elements.sceneCanvas.getBoundingClientRect();
  const range = visibleWorldRange(rect);
  const minS = range.minX;
  const maxS = range.maxX;
  const minOffset = range.minY;
  const maxOffset = range.maxY;
  const laneGuides = [];
  for (let offset = -24; offset <= 24; offset += 3.5) laneGuides.push(offset);

  ctx.save();
  ctx.strokeStyle = "#2f363d";
  ctx.lineWidth = 1 * dpr;
  ctx.setLineDash([5 * dpr, 7 * dpr]);
  for (const offset of laneGuides) {
    if (offset < minOffset || offset > maxOffset) continue;
    const [x1, y1] = worldToScreen([minS, offset]);
    const [x2, y2] = worldToScreen([maxS, offset]);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.strokeStyle = "#68717c";
  ctx.lineWidth = 1.5 * dpr;
  const [x1, y1] = worldToScreen([minS, 0]);
  const [x2, y2] = worldToScreen([maxS, 0]);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.restore();
}

function drawFrenetStableRuns(debug) {
  if (!debug?.ok || !elements.showFrenetRibbons.checked) return;
  const range = visibleWorldRange(elements.sceneCanvas.getBoundingClientRect());
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  ctx.globalAlpha = 0.12;
  for (const run of debug.stable_runs || []) {
    const sRange = run.s_range_m || [];
    if (sRange.length < 2) continue;
    const [x1, y1] = worldToScreen([Number(sRange[0]), range.minY]);
    const [x2, y2] = worldToScreen([Number(sRange[1]), range.maxY]);
    ctx.fillStyle = runColor(run.sequence_key || "");
    ctx.fillRect(Math.min(x1, x2), Math.min(y1, y2), Math.abs(x2 - x1), Math.abs(y2 - y1));
  }
  ctx.restore();
}

function drawFrenetStableRunAxisLabels(debug) {
  if (!debug?.ok || !elements.showFrenetRibbons.checked || !elements.showLabels.checked) return;
  const rect = elements.sceneCanvas.getBoundingClientRect();
  const range = visibleWorldRange(rect);
  const dpr = window.devicePixelRatio || 1;
  const gutterWidth = 160 * dpr;
  const minLabelGap = 18 * dpr;
  const labels = [];
  for (const run of debug.stable_runs || []) {
    const sRange = run.s_range_m || [];
    if (sRange.length < 2) continue;
    const runStart = Number(sRange[0]);
    const runEnd = Number(sRange[1]);
    if (!Number.isFinite(runStart) || !Number.isFinite(runEnd)) continue;
    const visibleStart = Math.max(runStart, range.minX);
    const visibleEnd = Math.min(runEnd, range.maxX);
    if (visibleEnd < visibleStart) continue;
    const text = (run.ribbon_sequence || []).join("-");
    if (!text) continue;
    const [, y] = worldToScreen([0.5 * (visibleStart + visibleEnd), 0]);
    labels.push({ y, text, color: runColor(run.sequence_key || "") });
  }

  labels.sort((a, b) => a.y - b.y);
  let lastY = -Infinity;
  ctx.save();
  ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  for (const label of labels) {
    const y = clamp(Math.max(label.y, lastY + minLabelGap), 14 * dpr, elements.sceneCanvas.height - 14 * dpr);
    lastY = y;
    const textWidth = Math.min(ctx.measureText(label.text).width, gutterWidth - 20 * dpr);
    ctx.fillStyle = "rgba(16, 20, 24, 0.9)";
    ctx.fillRect(4 * dpr, y - 8 * dpr, gutterWidth, 16 * dpr);
    ctx.strokeStyle = label.color;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.moveTo(7 * dpr, y - 6 * dpr);
    ctx.lineTo(7 * dpr, y + 6 * dpr);
    ctx.stroke();
    ctx.save();
    ctx.beginPath();
    ctx.rect(12 * dpr, y - 8 * dpr, textWidth + 2 * dpr, 16 * dpr);
    ctx.clip();
    ctx.fillStyle = "#d3d9df";
    ctx.fillText(label.text, 12 * dpr, y);
    ctx.restore();
  }
  ctx.restore();
}

function drawFrenetRibbons(debug) {
  if (!debug?.ok || !elements.showFrenetRibbons.checked) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  ctx.lineWidth = Math.max(1, 0.04 * currentView().scale * dpr);
  ctx.globalAlpha = 0.5;
  for (const slice of debug.slices || []) {
    const s = Number(slice.s);
    if (!Number.isFinite(s)) continue;
    for (const ribbon of slice.ribbons || []) {
      const left = Number(ribbon.left_l_m);
      const right = Number(ribbon.right_l_m);
      if (!Number.isFinite(left) || !Number.isFinite(right)) continue;
      const [x1, y1] = worldToScreen([s, left]);
      const [x2, y2] = worldToScreen([s, right]);
      ctx.strokeStyle = ribbonColor(ribbon.width_class);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
  }
  ctx.restore();
}

function drawFrenetTracks(debug, points) {
  if (!elements.showFrenetTracks.checked) return;
  const tracks = new Map();
  const labels = [];
  if (debug?.ok) {
    for (const track of debug.tracks || []) {
      tracks.set(track.id, (track.samples || []).map((sample) => ({
        s: Number(sample.s),
        offset: Number(sample.l),
        sourceType: track.source_type,
        label: track.label || "",
      })));
    }
  } else {
    for (const point of points) {
      if (!tracks.has(point.sourceLineId)) tracks.set(point.sourceLineId, []);
      tracks.get(point.sourceLineId).push(point);
    }
  }
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  ctx.lineWidth = 1.25 * dpr;
  for (const track of tracks.values()) {
    track.sort((a, b) => a.s - b.s);
    if (track.length < 2) continue;
    ctx.beginPath();
    track.forEach((point, index) => {
      const [x, y] = worldToScreen([point.s, point.offset]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = frenetColor(track[0].sourceType);
    ctx.globalAlpha = 0.55;
    ctx.stroke();
    if (elements.showLabels.checked && track[0].label) {
      labels.push({
        point: track[Math.floor(track.length / 2)],
        text: track[0].label,
        color: frenetColor(track[0].sourceType),
      });
    }
  }
  ctx.restore();

  if (labels.length) {
    ctx.save();
    ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    for (const label of labels) {
      const [x, y] = worldToScreen([label.point.s, label.point.offset]);
      const textWidth = ctx.measureText(label.text).width;
      ctx.fillStyle = "rgba(16, 20, 24, 0.82)";
      ctx.fillRect(x + 5 * dpr, y - 8 * dpr, textWidth + 8 * dpr, 16 * dpr);
      ctx.fillStyle = label.color;
      ctx.fillText(label.text, x + 9 * dpr, y);
    }
    ctx.restore();
  }
}

function drawRibbonCenters(debug) {
  if (!debug?.ok || !elements.showRibbonCenters.checked) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  const labels = [];
  for (const track of debug.ribbon_tracks || []) {
    const samples = (track.samples || [])
      .map((sample) => ({
        s: Number(sample.s),
        center: Number(sample.center_l_m),
      }))
      .filter((sample) => Number.isFinite(sample.s) && Number.isFinite(sample.center))
      .sort((a, b) => a.s - b.s);
    if (samples.length < 2) continue;
    ctx.beginPath();
    samples.forEach((sample, index) => {
      const [x, y] = worldToScreen([sample.s, sample.center]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = ribbonCenterColor(track.ribbon_class, track.width_class);
    ctx.globalAlpha = track.ribbon_class === "short" ? 0.35 : 0.9;
    ctx.lineWidth = ribbonCenterWidth(track.ribbon_class) * dpr;
    ctx.setLineDash(track.ribbon_class === "unstable" || track.ribbon_class === "short" ? [6 * dpr, 6 * dpr] : []);
    ctx.stroke();
    if (elements.showLabels.checked) {
      labels.push({
        point: samples[Math.floor(samples.length / 2)],
        text: `${track.label || "RT"} ${track.ribbon_class || ""} ${Number(track.width_median_m || 0).toFixed(1)}m`,
        color: ribbonCenterColor(track.ribbon_class, track.width_class),
      });
    }
  }
  ctx.setLineDash([]);
  ctx.restore();

  if (labels.length) {
    ctx.save();
    ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    for (const label of labels) {
      const [x, y] = worldToScreen([label.point.s, label.point.center]);
      const textWidth = ctx.measureText(label.text).width;
      ctx.fillStyle = "rgba(16, 18, 20, 0.82)";
      ctx.fillRect(x + 5 * dpr, y - 8 * dpr, textWidth + 8 * dpr, 16 * dpr);
      ctx.fillStyle = label.color;
      ctx.fillText(label.text, x + 9 * dpr, y);
    }
    ctx.restore();
  }
}

function drawLaneCenters(debug) {
  if (!debug?.ok || !elements.showLaneCenters.checked) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  const labels = [];
  for (const track of debug.lane_center_tracks || []) {
    const samples = (track.samples || [])
      .map((sample) => ({
        s: Number(sample.s),
        center: Number(sample.center_l_m),
      }))
      .filter((sample) => Number.isFinite(sample.s) && Number.isFinite(sample.center))
      .sort((a, b) => a.s - b.s);
    if (samples.length < 2) continue;
    ctx.beginPath();
    samples.forEach((sample, index) => {
      const [x, y] = worldToScreen([sample.s, sample.center]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = laneCenterColor(track.quality);
    ctx.globalAlpha = track.quality === "short" ? 0.42 : 0.96;
    ctx.lineWidth = laneCenterWidth(track.quality) * dpr;
    ctx.setLineDash(track.quality === "stable_lane" ? [] : [8 * dpr, 5 * dpr]);
    ctx.stroke();
    if (elements.showLabels.checked) {
      labels.push({
        point: samples[Math.floor(samples.length / 2)],
        text: `${track.label || "LC"} ${track.quality || ""} ${Number(track.width_median_m || 0).toFixed(1)}m ${Number(track.confidence || 0).toFixed(2)}`,
        color: laneCenterColor(track.quality),
      });
    }
  }
  ctx.setLineDash([]);
  ctx.restore();

  if (labels.length) {
    ctx.save();
    ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    for (const label of labels) {
      const [x, y] = worldToScreen([label.point.s, label.point.center]);
      const textWidth = ctx.measureText(label.text).width;
      ctx.fillStyle = "rgba(16, 18, 20, 0.86)";
      ctx.fillRect(x + 5 * dpr, y - 8 * dpr, textWidth + 8 * dpr, 16 * dpr);
      ctx.fillStyle = label.color;
      ctx.fillText(label.text, x + 9 * dpr, y);
    }
    ctx.restore();
  }
}

function drawLaneTracks(debug) {
  if (!debug?.ok || !elements.showLaneTracks.checked) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  const labels = [];
  for (const track of debug.lane_tracks || []) {
    const samples = (track.samples || [])
      .map((sample) => ({
        s: Number(sample.s),
        center: Number(sample.center_l_m),
      }))
      .filter((sample) => Number.isFinite(sample.s) && Number.isFinite(sample.center))
      .sort((a, b) => a.s - b.s);
    if (samples.length < 2) continue;

    const gaps = (track.gaps || []).map((gap) => {
      const range = gap.s_range_m || [];
      return { start: Number(range[0]), end: Number(range[1]) };
    }).filter((gap) => Number.isFinite(gap.start) && Number.isFinite(gap.end));

    ctx.strokeStyle = laneTrackColor(track.quality);
    ctx.globalAlpha = track.quality === "single_observation" ? 0.45 : 0.92;
    ctx.lineWidth = laneTrackWidth(track.quality) * dpr;
    ctx.setLineDash([]);
    drawSamplePolylineWithGapMask(samples, gaps, false);

    if (gaps.length) {
      ctx.globalAlpha = 0.65;
      ctx.setLineDash([10 * dpr, 7 * dpr]);
      drawSamplePolylineWithGapMask(samples, gaps, true);
      ctx.setLineDash([]);
    }

    if (elements.showLabels.checked) {
      labels.push({
        point: samples[Math.floor(samples.length / 2)],
        text: `${track.label || "LT"} ${(track.lane_center_labels || []).join("+")}`,
        color: laneTrackColor(track.quality),
      });
    }
  }
  ctx.restore();

  if (labels.length) {
    ctx.save();
    ctx.font = `${12 * dpr}px Inter, system-ui, sans-serif`;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    for (const label of labels) {
      const [x, y] = worldToScreen([label.point.s, label.point.center]);
      const textWidth = ctx.measureText(label.text).width;
      ctx.fillStyle = "rgba(16, 18, 20, 0.9)";
      ctx.fillRect(x + 5 * dpr, y - 9 * dpr, textWidth + 8 * dpr, 18 * dpr);
      ctx.fillStyle = label.color;
      ctx.fillText(label.text, x + 9 * dpr, y);
    }
    ctx.restore();
  }
}

function drawSamplePolylineWithGapMask(samples, gaps, drawGaps) {
  const isInGap = (s0, s1) => gaps.some((gap) => s0 >= gap.start - 1e-6 && s1 <= gap.end + 1e-6);
  let drawing = false;
  ctx.beginPath();
  for (let i = 0; i < samples.length; ++i) {
    const sample = samples[i];
    if (i === 0) {
      if (!drawGaps) {
        const [x, y] = worldToScreen([sample.s, sample.center]);
        ctx.moveTo(x, y);
        drawing = true;
      }
      continue;
    }
    const prev = samples[i - 1];
    const segmentIsGap = isInGap(prev.s, sample.s);
    if (segmentIsGap !== drawGaps) {
      if (!drawing && !drawGaps) {
        const [x, y] = worldToScreen([sample.s, sample.center]);
        ctx.moveTo(x, y);
        drawing = true;
      } else {
        drawing = false;
      }
      continue;
    }
    const [x0, y0] = worldToScreen([prev.s, prev.center]);
    const [x1, y1] = worldToScreen([sample.s, sample.center]);
    if (!drawing) {
      ctx.moveTo(x0, y0);
      drawing = true;
    }
    ctx.lineTo(x1, y1);
  }
  ctx.stroke();
}

function drawLaneRegions(debug) {
  if (!debug?.ok || !elements.showLaneRegions.checked) return;
  const dpr = window.devicePixelRatio || 1;
  const labels = [];
  ctx.save();
  for (const region of debug.regions || []) {
    const samples = (region.samples || [])
      .map((sample) => ({
        s: Number(sample.s),
        right: Number(sample.right_l_m),
        left: Number(sample.left_l_m),
        center: Number(sample.center_l_m),
        width: Number(sample.width_m),
        rightState: sample.right_state || "observed",
        leftState: sample.left_state || "observed",
      }))
      .filter((sample) =>
        Number.isFinite(sample.s) &&
        Number.isFinite(sample.right) &&
        Number.isFinite(sample.left) &&
        Number.isFinite(sample.center) &&
        sample.left > sample.right)
      .sort((a, b) => a.s - b.s);
    if (samples.length < 1) continue;
    const color = laneRegionColor(region);
    ctx.globalAlpha = region.candidate ? 0.18 : 0.07;
    ctx.strokeStyle = color;
    ctx.lineWidth = Math.max(1, 0.05 * currentView().scale * dpr);
    ctx.setLineDash(region.has_inferred_boundary ? [5 * dpr, 4 * dpr] : []);
    for (const sample of samples) {
      const [x1, y1] = worldToScreen([sample.s, sample.right]);
      const [x2, y2] = worldToScreen([sample.s, sample.left]);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
    if (samples.length >= 2) {
      ctx.globalAlpha = region.candidate ? 0.85 : 0.35;
      ctx.lineWidth = (region.candidate ? 1.4 : 0.9) * dpr;
      ctx.beginPath();
      samples.forEach((sample, index) => {
        const [x, y] = worldToScreen([sample.s, sample.center]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }
    if (elements.showLabels.checked && region.candidate) {
      labels.push({
        point: samples[Math.floor(samples.length / 2)],
        color,
        text: `${region.label || "LR"} ${region.width_class || ""} ${Number(region.width_median_m || 0).toFixed(1)}m${region.has_inferred_boundary ? " inferred" : ""}`,
      });
    }
  }
  ctx.setLineDash([]);
  ctx.restore();

  if (labels.length) {
    ctx.save();
    ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    for (const label of labels) {
      const [x, y] = worldToScreen([label.point.s, label.point.center]);
      const textWidth = ctx.measureText(label.text).width;
      ctx.fillStyle = "rgba(16, 18, 20, 0.86)";
      ctx.fillRect(x + 5 * dpr, y - 8 * dpr, textWidth + 8 * dpr, 16 * dpr);
      ctx.fillStyle = label.color;
      ctx.fillText(label.text, x + 9 * dpr, y);
    }
    ctx.restore();
  }
}

function drawFrenetPoints(points) {
  if (!elements.showRawIntersection.checked) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  for (const point of points) {
    const [x, y] = worldToScreen([point.s, point.offset]);
    ctx.fillStyle = frenetColor(point.sourceType);
    ctx.strokeStyle = "#101214";
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    ctx.arc(x, y, 3.2 * dpr, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

function drawBoundaryCompletion(debug) {
  if (!debug?.ok || !elements.showBoundaryCompletion.checked) return;
  const dpr = window.devicePixelRatio || 1;
  const nodes = [];
  for (const node of debug.forward_inferred_nodes || []) {
    nodes.push({ node, color: "#f2c94c", radius: 3.6, alpha: 0.35, label: "F" });
  }
  for (const node of debug.backward_inferred_nodes || []) {
    nodes.push({ node, color: "#56ccf2", radius: 3.6, alpha: 0.35, label: "B" });
  }
  for (const node of debug.merged_inferred_nodes || []) {
    nodes.push({ node, color: "#ff4fd8", radius: 5.0, alpha: 0.9, label: "M" });
  }
  ctx.save();
  ctx.lineWidth = 1.3 * dpr;
  ctx.setLineDash([7 * dpr, 5 * dpr]);
  for (const link of debug.links || []) {
    const fromS = Number(link.from_s);
    const fromL = Number(link.from_l);
    const toS = Number(link.to_s);
    const toL = Number(link.to_l);
    if (!Number.isFinite(fromS) || !Number.isFinite(fromL) || !Number.isFinite(toS) || !Number.isFinite(toL)) continue;
    const [x1, y1] = worldToScreen([fromS, fromL]);
    const [x2, y2] = worldToScreen([toS, toL]);
    ctx.globalAlpha = link.method === "inferred_sequence" ? 0.45 : 0.78;
    ctx.strokeStyle = link.method === "inferred_to_next_entity" ? "#ffb020" : "#ff4fd8";
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.lineWidth = 1.4 * dpr;
  ctx.font = `${10 * dpr}px Inter, system-ui, sans-serif`;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  for (const item of nodes) {
    const s = Number(item.node.s);
    const l = Number(item.node.l);
    if (!Number.isFinite(s) || !Number.isFinite(l)) continue;
    const [x, y] = worldToScreen([s, l]);
    ctx.globalAlpha = item.alpha;
    ctx.fillStyle = item.color;
    const size = item.radius * 1.2 * dpr;
    ctx.fillRect(x - 0.5 * size, y - 0.5 * size, size, size);
    if (elements.showLabels.checked && item.label === "M") {
      const method = item.node.method ? String(item.node.method).replace("_topology_ribbon_", "_") : "";
      const text = `${item.node.label || item.label}${method ? ` ${method}` : ""}`;
      const textWidth = ctx.measureText(text).width;
      ctx.globalAlpha = 0.88;
      ctx.fillStyle = "rgba(16, 20, 24, 0.86)";
      ctx.fillRect(x + 6 * dpr, y - 7 * dpr, textWidth + 7 * dpr, 14 * dpr);
      ctx.fillStyle = item.color;
      ctx.fillText(text, x + 9 * dpr, y);
    }
  }
  for (const item of debug.junction_candidates || []) {
    const s = Number(item.s);
    const l = Number(item.l);
    if (!Number.isFinite(s) || !Number.isFinite(l)) continue;
    const [x, y] = worldToScreen([s, l]);
    const color = item.event_hint === "split_start" ? "#ffb020" : (item.event_hint === "merge_end" ? "#56ccf2" : "#ffffff");
    ctx.globalAlpha = 0.95;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.8 * dpr;
    ctx.beginPath();
    ctx.arc(x, y, 5.4 * dpr, 0, Math.PI * 2);
    ctx.stroke();
    if (elements.showLabels.checked) {
      const text = `${item.label || "J"} ${item.event_hint || ""}`;
      const textWidth = ctx.measureText(text).width;
      ctx.fillStyle = "rgba(16, 20, 24, 0.86)";
      ctx.fillRect(x + 6 * dpr, y - 7 * dpr, textWidth + 7 * dpr, 14 * dpr);
      ctx.fillStyle = color;
      ctx.fillText(text, x + 9 * dpr, y);
    }
  }
  ctx.restore();
}

function drawFrenetLabels(points) {
  const byTrack = new Map();
  for (const point of points) {
    if (!byTrack.has(point.sourceLineId)) byTrack.set(point.sourceLineId, point);
  }
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "#d8dde3";
  for (const point of byTrack.values()) {
    const [x, y] = worldToScreen([point.s, point.offset]);
    ctx.fillText(`${point.laneId}:${point.laneIndex}:${point.sectionIndex}`, x + 5 * dpr, y - 5 * dpr);
  }
  ctx.restore();
}

function frenetColor(sourceType) {
  if (sourceType === "road_edge") return "#f2c94c";
  if (sourceType === "lane_line") return "#2dff9a";
  if (sourceType === "curb") return "#eb5757";
  return "#d8dde3";
}

function ribbonColor(widthClass) {
  if (widthClass === "lane") return "#2dff9a";
  if (widthClass === "shoulder") return "#56ccf2";
  if (widthClass === "narrow") return "#ffb020";
  if (widthClass === "wide") return "#bb6bd9";
  return "#eb5757";
}

function runColor(sequenceKey) {
  let hash = 0;
  for (let i = 0; i < sequenceKey.length; ++i) {
    hash = (hash * 31 + sequenceKey.charCodeAt(i)) >>> 0;
  }
  const colors = ["#2dff9a", "#56ccf2", "#f2c94c", "#bb6bd9", "#ff8f3d"];
  return colors[hash % colors.length];
}

function ribbonCenterColor(ribbonClass, widthClass) {
  if (ribbonClass === "stable_lane") return "#00d084";
  if (ribbonClass === "stable_shoulder") return "#56ccf2";
  if (ribbonClass === "wide_transition") return "#bb6bd9";
  return ribbonColor(widthClass);
}

function ribbonCenterWidth(ribbonClass) {
  if (ribbonClass === "stable_lane") return 3.0;
  if (ribbonClass === "stable_shoulder") return 2.2;
  if (ribbonClass === "wide_transition") return 2.4;
  return 1.4;
}

function laneCenterColor(quality) {
  if (quality === "stable_lane") return "#ffffff";
  if (quality === "candidate_lane") return "#00d084";
  if (quality === "short") return "#ffb020";
  return "#9aa4ad";
}

function laneCenterWidth(quality) {
  if (quality === "stable_lane") return 3.4;
  if (quality === "candidate_lane") return 2.4;
  return 1.6;
}

function laneTrackColor(quality) {
  if (quality === "tracked_lane") return "#56ccf2";
  if (quality === "weak_tracked_lane") return "#2dff9a";
  return "#9aa4ad";
}

function laneTrackWidth(quality) {
  if (quality === "tracked_lane") return 4.6;
  if (quality === "weak_tracked_lane") return 3.2;
  return 2.0;
}

function laneRegionColor(region) {
  if (region?.candidate && region?.lane_line_pair) return "#00d084";
  if (region?.candidate && region?.lane_to_boundary_pair) return "#56ccf2";
  if (region?.width_class === "wide") return "#bb6bd9";
  if (region?.width_class === "narrow") return "#ffb020";
  return "#6f7a84";
}

function findNavigationSegmentItem(segmentIndex) {
  const layers = state.current.viz?.layers || [];
  const targetId = `sd_route_nav_${segmentIndex}`;
  for (const layer of layers) {
    for (const item of layer.items || []) {
      if (item.id === targetId && item.points?.length >= 2) return item;
    }
  }
  return null;
}

function drawNavigationTrackerOverlay() {
  if (!elements.showNavigationTracker.checked) return;
  const tracker = state.current.navigationTracker;
  const match = tracker?.match;
  if (!match?.matched) {
    drawEgoMarker("#eb5757");
    return;
  }

  const item = findNavigationSegmentItem(match.segment_index);
  if (!item) {
    drawEgoMarker("#eb5757");
    return;
  }

  const activeColor = match.can_use_navigation_reference ? "#00d084" : "#ffb020";
  drawHighlightedPolyline(item.points, activeColor);
  drawEgoMarker(activeColor);

  const endPoint = item.points[item.points.length - 1];
  drawEndpointMarker(endPoint, activeColor);
  drawDistanceLine([0, 0, 0], endPoint, activeColor);

  const event = match.current_terminal_event || {};
  const label = [
    event.type || "event",
    Number.isFinite(match.distance_to_segment_end_m)
      ? `${match.distance_to_segment_end_m.toFixed(1)}m`
      : "",
  ].filter(Boolean).join(" · ");
  drawWorldText(endPoint, label, activeColor, 10, -10);
}

function drawHighlightedPolyline(points, color) {
  if (!points?.length) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = worldToScreen(point);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = Math.max(3 * dpr, 0.28 * currentView().scale * dpr);
  ctx.setLineDash([]);
  ctx.stroke();
  ctx.restore();
}

function drawDistanceLine(from, to, color) {
  const dpr = window.devicePixelRatio || 1;
  const [x1, y1] = worldToScreen(from);
  const [x2, y2] = worldToScreen(to);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.75;
  ctx.lineWidth = 1.5 * dpr;
  ctx.setLineDash([7 * dpr, 7 * dpr]);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.restore();
}

function drawEgoMarker(color) {
  const dpr = window.devicePixelRatio || 1;
  const [x, y] = worldToScreen([0, 0, 0]);
  ctx.save();
  ctx.fillStyle = color;
  ctx.strokeStyle = "#101214";
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath();
  ctx.moveTo(x + 10 * dpr, y);
  ctx.lineTo(x - 7 * dpr, y - 6 * dpr);
  ctx.lineTo(x - 4 * dpr, y);
  ctx.lineTo(x - 7 * dpr, y + 6 * dpr);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawEndpointMarker(point, color) {
  const dpr = window.devicePixelRatio || 1;
  const [x, y] = worldToScreen(point);
  ctx.save();
  ctx.fillStyle = color;
  ctx.strokeStyle = "#101214";
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath();
  ctx.arc(x, y, 6 * dpr, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawWorldText(point, text, color, offsetX = 0, offsetY = 0) {
  const dpr = window.devicePixelRatio || 1;
  const [x, y] = worldToScreen(point);
  ctx.save();
  ctx.font = `${12 * dpr}px Inter, system-ui, sans-serif`;
  ctx.textBaseline = "bottom";
  ctx.textAlign = "left";
  const tx = x + offsetX * dpr;
  const ty = y + offsetY * dpr;
  const metrics = ctx.measureText(text);
  ctx.fillStyle = "rgba(16, 18, 20, 0.82)";
  ctx.fillRect(tx - 4 * dpr, ty - 18 * dpr, metrics.width + 8 * dpr, 20 * dpr);
  ctx.fillStyle = color;
  ctx.fillText(text, tx, ty - 3 * dpr);
  ctx.restore();
}

function drawGrid() {
  const dpr = window.devicePixelRatio || 1;
  const rect = elements.sceneCanvas.getBoundingClientRect();
  const step = chooseGridStep();
  const { minX, minY, maxX, maxY } = visibleWorldRange(rect);
  const xTicks = [];
  const yTicks = [];

  ctx.strokeStyle = "#22272d";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.floor(minX / step) * step; x <= maxX; x += step) {
    xTicks.push(normalizeTick(x));
    const [sx1, sy1] = worldToScreen([x, minY]);
    const [sx2, sy2] = worldToScreen([x, maxY]);
    ctx.moveTo(sx1, sy1);
    ctx.lineTo(sx2, sy2);
  }
  for (let y = Math.floor(minY / step) * step; y <= maxY; y += step) {
    yTicks.push(normalizeTick(y));
    const [sx1, sy1] = worldToScreen([minX, y]);
    const [sx2, sy2] = worldToScreen([maxX, y]);
    ctx.moveTo(sx1, sy1);
    ctx.lineTo(sx2, sy2);
  }
  ctx.stroke();

  ctx.strokeStyle = "#4b5560";
  ctx.lineWidth = 1.5 * dpr;
  ctx.beginPath();
  let [x0a, y0a] = worldToScreen([0, minY]);
  let [x0b, y0b] = worldToScreen([0, maxY]);
  ctx.moveTo(x0a, y0a);
  ctx.lineTo(x0b, y0b);
  let [x1a, y1a] = worldToScreen([minX, 0]);
  let [x1b, y1b] = worldToScreen([maxX, 0]);
  ctx.moveTo(x1a, y1a);
  ctx.lineTo(x1b, y1b);
  ctx.stroke();

  drawAxisTicks(xTicks, yTicks, step, rect, dpr);
  drawOriginAndAxisLabels(rect, dpr);
}

function visibleWorldRange(rect) {
  const corners = [
    screenToWorld(0, 0),
    screenToWorld(rect.width, 0),
    screenToWorld(0, rect.height),
    screenToWorld(rect.width, rect.height),
  ];
  return {
    minX: Math.min(...corners.map((point) => point[0])),
    minY: Math.min(...corners.map((point) => point[1])),
    maxX: Math.max(...corners.map((point) => point[0])),
    maxY: Math.max(...corners.map((point) => point[1])),
  };
}

function normalizeTick(value) {
  return Math.abs(value) < 1e-9 ? 0 : value;
}

function formatTick(value, step) {
  const absStep = Math.abs(step);
  const digits = absStep >= 1 ? 0 : Math.min(2, Math.ceil(-Math.log10(absStep)));
  return Number(value).toFixed(digits);
}

function drawAxisTicks(xTicks, yTicks, step, rect, dpr) {
  const [axisOriginX, axisOriginY] = worldToScreen([0, 0]);
  const xLabelY = clamp(axisOriginY + 14 * dpr, 14 * dpr, elements.sceneCanvas.height - 8 * dpr);
  const yLabelX = clamp(axisOriginX + 6 * dpr, 4 * dpr, elements.sceneCanvas.width - 44 * dpr);

  ctx.save();
  ctx.fillStyle = "#9aa4ad";
  ctx.font = `${11 * dpr}px Inter, system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (const x of xTicks) {
    if (Math.abs(x) < 1e-9) continue;
    const [sx] = worldToScreen([x, 0]);
    if (sx < 18 * dpr || sx > elements.sceneCanvas.width - 18 * dpr) continue;
    ctx.fillText(formatTick(x, step), sx, xLabelY);
  }

  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  for (const y of yTicks) {
    if (Math.abs(y) < 1e-9) continue;
    const [, sy] = worldToScreen([0, y]);
    if (sy < 10 * dpr || sy > elements.sceneCanvas.height - 10 * dpr) continue;
    ctx.fillText(formatTick(y, step), yLabelX, sy);
  }
  ctx.restore();
}

function drawOriginAndAxisLabels(rect, dpr) {
  const origin = worldToScreen([0, 0]);
  const view = currentView();
  const xAxisEnd = worldToScreen([Math.min(2, rect.height / Math.max(1, view.scale)), 0]);
  const yAxisEnd = worldToScreen([0, Math.min(2, rect.width / Math.max(1, view.scale))]);
  const xLabel = state.sceneMode === "frenet" ? "S" : "X";
  const yLabel = state.sceneMode === "frenet" ? "Offset" : "Y";

  ctx.save();
  ctx.fillStyle = "#f1f3f5";
  ctx.strokeStyle = "#f1f3f5";
  ctx.lineWidth = 1.5 * dpr;
  ctx.beginPath();
  ctx.arc(origin[0], origin[1], 3.5 * dpr, 0, Math.PI * 2);
  ctx.fill();

  ctx.font = `${12 * dpr}px Inter, system-ui, sans-serif`;
  ctx.textBaseline = "bottom";
  ctx.textAlign = "left";
  ctx.fillText("O", origin[0] + 6 * dpr, origin[1] - 6 * dpr);
  ctx.fillText(xLabel, xAxisEnd[0] + 6 * dpr, xAxisEnd[1] - 8 * dpr);
  ctx.fillText(yLabel, yAxisEnd[0] - 28 * dpr, yAxisEnd[1] - 6 * dpr);
  ctx.restore();
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function chooseGridStep() {
  const targetPx = 80;
  const raw = targetPx / Math.max(0.001, currentView().scale);
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / pow;
  if (n < 2) return pow;
  if (n < 5) return 2 * pow;
  return 5 * pow;
}

function fitView() {
  const bounds = state.sceneMode === "frenet"
    ? computeFrenetBounds(frenetPoints())
    : computeBounds(visibleItems());
  if (!bounds) return;
  const rect = elements.sceneCanvas.getBoundingClientRect();
  const width = Math.max(1, rect.width);
  const height = Math.max(1, rect.height);
  const spanX = Math.max(1, bounds.maxX - bounds.minX);
  const spanY = Math.max(1, bounds.maxY - bounds.minY);
  const view = currentView();
  view.x = (bounds.minX + bounds.maxX) / 2;
  view.y = (bounds.minY + bounds.maxY) / 2;
  view.scale = Math.max(0.5, Math.min(height / spanX, width / spanY) * 0.82);
  render();
}

function computeBounds(items) {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const item of items) {
    for (const p of item.points || []) {
      minX = Math.min(minX, p[0]);
      minY = Math.min(minY, p[1]);
      maxX = Math.max(maxX, p[0]);
      maxY = Math.max(maxY, p[1]);
    }
  }
  if (!Number.isFinite(minX)) return null;
  return { minX, minY, maxX, maxY };
}

function computeFrenetBounds(points) {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const point of points) {
    minX = Math.min(minX, point.s);
    minY = Math.min(minY, point.offset);
    maxX = Math.max(maxX, point.s);
    maxY = Math.max(maxY, point.offset);
  }
  if (!Number.isFinite(minX)) return null;
  const padX = Math.max(8, (maxX - minX) * 0.05);
  const padY = Math.max(2, (maxY - minY) * 0.2);
  return { minX: minX - padX, minY: minY - padY, maxX: maxX + padX, maxY: maxY + padY };
}

function onWheel(event) {
  event.preventDefault();
  const rect = elements.sceneCanvas.getBoundingClientRect();
  const sx = event.clientX - rect.left;
  const sy = event.clientY - rect.top;
  const anchor = screenToWorld(sx, sy);
  const factor = event.deltaY > 0 ? 0.9 : 1.1;
  const view = currentView();
  view.scale = Math.max(0.1, Math.min(600, view.scale * factor));
  const after = screenToWorld(sx, sy);
  view.x += anchor[0] - after[0];
  view.y += anchor[1] - after[1];
  render();
}

function onPointerDown(event) {
  const view = currentView();
  view.dragging = true;
  view.lastX = event.clientX;
  view.lastY = event.clientY;
  elements.sceneCanvas.classList.add("is-panning");
  elements.sceneCanvas.setPointerCapture?.(event.pointerId);
}

function onPointerMove(event) {
  const view = currentView();
  if (!view.dragging) return;
  const dx = event.clientX - view.lastX;
  const dy = event.clientY - view.lastY;
  view.x += dy / view.scale;
  view.y += dx / view.scale;
  view.lastX = event.clientX;
  view.lastY = event.clientY;
  render();
}

function onPointerUp(event) {
  state.view.dragging = false;
  state.frenetView.dragging = false;
  elements.sceneCanvas.classList.remove("is-panning");
  try { elements.sceneCanvas.releasePointerCapture?.(event.pointerId); } catch (_) {}
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

bindEvents();
loadDataset(state.datasetPath).then(fitView).catch((error) => {
  console.error(error);
  elements.bundleMeta.textContent = `Load failed: ${error.message}`;
});
