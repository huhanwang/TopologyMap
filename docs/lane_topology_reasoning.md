# Lane Topology Reasoning Design

Date: 2026-07-13

Status: reasoning draft

## 1. 背景

当前已经有以下中间结果：

- fused reference：把视觉近场和导航远端趋势融合成 Frenet 参考轴。
- raw intersections：沿 fused reference 法线采样，与 lane line、edge line、curb 求交点。
- Frenet ribbons：每个 slice 上相邻交点夹出的横向区域。
- ribbon tracks / lane center tracks：把 lane-like ribbon 的中心线沿 s 方向连接。

这些结果已经能看出很多几何结构，但还不能稳定得到拓扑。主要原因是：

```text
只追踪车道中心会丢掉车道之间的共享边界关系。
只看单个 ribbon 会丢掉左右邻接关系。
直接检测 add/drop/split 会把观测噪声、车道分配和拓扑事件混在一起。
```

因此需要把问题拆成三个相互依赖但职责不同的层：

```text
1. 车道分配 / 追踪
2. Ribbon 特征统计
3. 拓扑事件检测
```

这份文档定义这三层的职责边界、核心数据表达和建议实现顺序。

## 2. 核心结论

拓扑不是单条 lane center 的属性，而是 lane region 之间的关系。

一个可行驶车道区域应该表达为：

```text
LaneRegion = (right_boundary, left_boundary, center_l, width, s)
```

两个相邻车道必须通过边界关系成立：

```text
EGO.left_boundary ~= L1.right_boundary
EGO.right_boundary ~= R1.left_boundary
```

其中 `~=` 可以是严格同一条边界，也可以是几何上连续、距离很小、被判定为同一边界的断裂/换 id。

如果某个区域在 EGO 左侧，但不和 EGO 共享边界或近似共享边界，它不能直接叫 L1。它只能是：

```text
left_parallel_candidate
possible_missing_boundary
possible_split_branch
unattached_region
```

这条约束非常关键。否则算法会把“看起来在左侧的中心线”误认为“左一车道”，从而把几何关系错当拓扑关系。

## 3. 三层职责

### 3.1 车道分配 / 追踪层

目标：

```text
回答谁是谁，以及谁和谁相邻。
```

这一层不负责判断 add/drop/split/merge，只负责建立稳定的 lane identity 和邻接关系。

输入：

- 每个 slice 的 raw intersections。
- 每个 slice 的 lane-like ribbons。
- boundary id、source type、offset、width、confidence。

输出：

- LaneRegionTrack：纵向追踪出的 lane region。
- RegionAdjacency：region 之间的共享边界/近似共享边界关系。
- Ego-relative assignment：从 EGO 出发命名 L1/R1/L2/R2。

必须保留的信息：

```text
region_id
right_boundary_track_id
left_boundary_track_id
center_l(s)
width(s)
right_neighbor / left_neighbor
shared_boundary_score
boundary_gap_m
assignment_role: EGO / L1 / R1 / L2 / R2 / unattached
```

这一层判断 L1/R1 的原则：

```text
L1 = EGO 左侧、并与 EGO 共享左边界的 lane region。
R1 = EGO 右侧、并与 EGO 共享右边界的 lane region。
```

如果不共享边界，即使 center_l 合理，也不能直接赋为 L1/R1。

### 3.2 Ribbon 特征统计层

目标：

```text
回答每个 ribbon / region 本身怎么变化。
```

这一层不负责命名 L1/R1，也不直接输出拓扑事件。它只输出统计特征和状态提示。

每个 RegionTrack 需要统计：

```text
support_length_m
support_ratio
gap_ratio

center_l_min/max/range
center_l_mean
center_l_slope
center_l_residual_std

left_l_min/max/range
left_l_slope

right_l_min/max/range
right_l_slope

width_min/max/range
width_mean
width_slope
width_residual_std

left_boundary_stability
right_boundary_stability
neighbor_left_consistency
neighbor_right_consistency
```

这一层可以输出状态提示，但提示不是最终拓扑：

```text
stable_parallel
drifting_transition
birth_like
death_like
wide_transition
noisy
short
unattached_parallel
```

典型特征：

稳定车道：

```text
center_l_range 小
width_range 小
boundary stability 高
neighbor consistency 高
```

过渡/分叉候选：

```text
center_l_range 大
center_l_slope 明显
center_l_residual_std 不大
```

噪声：

```text
support 短
gap 多
center_l_residual_std 大
boundary stability 低
```

新增候选：

```text
support 从中后段开始
width 从小变正常
neighbor relation 逐渐稳定
```

减少候选：

```text
support 在中途结束
width 从正常变小
neighbor relation 逐渐消失
```

### 3.3 拓扑事件检测层

目标：

```text
把车道分配结果和 ribbon 特征解释为拓扑事件。
```

输入：

- assignment role：EGO/L1/R1/L2/R2/unattached。
- RegionAdjacency：共享边界关系。
- RibbonFeature：center/width/boundary/neighbor 统计。

输出：

```text
stable_N_lanes
left_lane_add
right_lane_add
left_lane_drop
right_lane_drop
split_candidate
merge_candidate
unknown_transition
```

判断原则：

稳定多车道：

```text
EGO/L1/R1 等角色稳定存在
相邻角色共享边界关系稳定
center_l 和 width 稳定
```

新增车道：

```text
某侧新的 assigned region 出现
与已有外侧车道形成共享边界或近似共享边界
support 从某个 s 之后持续存在
width 从异常/窄/短变成稳定车道宽
```

减少车道：

```text
某侧 assigned region 在某个 s 后持续缺失
消失前 width/neighbor relation 有收敛或消失趋势
```

split / branch：

```text
某个 assigned region 的 center_l 出现持续横向漂移
共享边界关系断裂、分叉或无法维持普通相邻结构
存在 unattached_parallel / branch-like region
```

merge：

```text
两个 region 的横向距离或共享边界关系逐渐收敛
其中一个 region 消失或并入另一个 region
```

## 4. 推荐中间表达

### 4.1 SliceRegion

每个 s slice 上，由相邻 boundary hits 生成 region：

```cpp
struct SliceRegion {
  double s;
  int region_index;  // right-to-left order within slice
  BoundaryRef right_boundary;
  BoundaryRef left_boundary;
  double right_l;
  double left_l;
  double center_l;
  double width;
  RegionWidthClass width_class;  // lane, shoulder, wide, narrow, invalid
  double confidence;
};
```

注意：

```text
SliceRegion 必须由相邻边界构成，不能跨过中间 hit。
```

### 4.2 RegionTrack

沿 s 方向关联 SliceRegion：

```cpp
struct RegionTrack {
  std::string id;
  std::vector<SliceRegionSample> samples;
  BoundaryTrackRef right_boundary_track;
  BoundaryTrackRef left_boundary_track;
  TrackQuality quality;
};
```

RegionTrack 关联代价应该包含：

```text
same/connected right boundary
same/connected left boundary
endpoint center_l continuity
endpoint width continuity
slice order consistency
neighbor consistency
gap penalty
conflict penalty
```

这里必须使用端点连续性，而不是整段 median。2014 帧已经证明：

```text
整段 median 会被后半段横向漂移拉偏，从而错误拆断本应连续的 region。
```

### 4.3 RegionAdjacency

相邻关系是显式图边：

```cpp
struct RegionAdjacency {
  RegionTrackId right_region;
  RegionTrackId left_region;
  BoundaryTrackRef shared_boundary;
  double shared_boundary_score;
  double boundary_gap_m;
  Range1D s_range;
};
```

`shared_boundary_score` 的含义：

```text
1.0: 完全同一条 boundary id
0.7~1.0: boundary id 切换但几何连续
0.3~0.7: 近似相邻，有短缺失或弱证据
<0.3: 不应认为是直接相邻车道
```

### 4.4 LaneAssignment

从 EGO region 出发命名：

```cpp
struct LaneAssignment {
  RegionTrackId ego;
  std::vector<AssignedLane> left_lanes;   // L1, L2...
  std::vector<AssignedLane> right_lanes;  // R1, R2...
  std::vector<RegionTrackId> unattached_regions;
};

struct AssignedLane {
  int slot_index;  // EGO=0, left positive, right negative
  RegionTrackId region_track;
  double assignment_confidence;
  double adjacency_confidence_to_inner_lane;
};
```

命名规则：

```text
EGO: 包含 l=0 的稳定 lane-like RegionTrack，或近场最接近 l=0 的稳定 region。
L1: EGO 左侧共享边界邻居。
R1: EGO 右侧共享边界邻居。
L2/R2: 继续沿共享边界图向外扩展。
```

如果 region 在左侧但不是共享边界邻居：

```text
不要命名为 L1/L2。
标记为 unattached_left_candidate。
```

## 5. 层间数据契约

为了避免算法混乱，每层只做自己的判断。

### 5.1 车道分配层不能做的事

不能输出：

```text
right_lane_add
split
merge
drop
```

它只能输出：

```text
EGO/L1/R1 assignment
adjacency score
unattached candidate
missing range
```

### 5.2 Ribbon 特征层不能做的事

不能直接命名：

```text
this is L1
this is R1
```

它只能输出：

```text
stable/drifting/birth/death/noisy hints
statistics
```

### 5.3 拓扑检测层不能绕过前两层

拓扑事件必须引用证据：

```text
which assignment changed
which feature changed
which adjacency changed
```

事件输出必须带 evidence：

```json
{
  "type": "right_lane_add_candidate",
  "s_range": [60, 100],
  "evidence": [
    "new_R2_assignment",
    "shared_boundary_with_R1",
    "width_birth_to_stable",
    "support_ratio_increase"
  ]
}
```

## 6. Debug JSON 建议

建议新增：

```text
lane_structure_debug.json
```

结构：

```json
{
  "schema_version": "topology-map.lane-structure-debug.v1",
  "frame_id": 2014,
  "slice_regions": [],
  "region_tracks": [],
  "region_adjacency": [],
  "lane_assignment": {},
  "ribbon_features": [],
  "topology_hints": []
}
```

### 6.1 slice_regions

用于检查每个截面的横向 region 是否正确：

```json
{
  "s": 20,
  "regions": [
    {
      "region_index": 0,
      "right_boundary": "B0",
      "left_boundary": "B1",
      "center_l": -3.8,
      "width": 3.7,
      "width_class": "lane"
    }
  ]
}
```

### 6.2 region_tracks

用于检查纵向是否串对：

```json
{
  "id": "RG2",
  "samples": [],
  "s_range_m": [34, 122],
  "right_boundary_track": "B3",
  "left_boundary_track": "B2398",
  "quality": "tracked_region"
}
```

### 6.3 region_adjacency

用于检查共享边界关系：

```json
{
  "right_region": "EGO_RG",
  "left_region": "L1_RG",
  "shared_boundary": "B1",
  "shared_boundary_score": 0.95,
  "boundary_gap_m": 0.0,
  "s_range_m": [6, 120]
}
```

### 6.4 lane_assignment

用于检查 EGO/L1/R1 是否满足共享边界约束：

```json
{
  "ego": "RG0",
  "left_lanes": [
    {
      "slot_index": 1,
      "role": "L1",
      "region_track": "RG1",
      "adjacency_confidence_to_inner_lane": 0.93
    }
  ],
  "right_lanes": [],
  "unattached_regions": [
    {
      "region_track": "RG5",
      "side": "left",
      "reason": "no_shared_boundary_with_assigned_lane"
    }
  ]
}
```

### 6.5 ribbon_features

用于检查几何变化：

```json
{
  "region_track": "RG1",
  "center_l_range_m": 0.42,
  "width_range_m": 0.31,
  "center_l_slope": 0.002,
  "width_slope": 0.001,
  "boundary_stability": 0.92,
  "state_hint": "stable_parallel"
}
```

### 6.6 topology_hints

这里只输出候选，不做最终结论：

```json
{
  "type": "right_lane_add_candidate",
  "s_range_m": [60, 110],
  "confidence": 0.72,
  "evidence": [
    "new_outer_right_assignment",
    "width_birth_to_stable",
    "shared_boundary_with_R1"
  ]
}
```

## 7. 实现顺序建议

### Step 1: SliceRegionGraph

从已有 `frenet_topology_debug` / raw intersections 生成每个 slice 的 lane-like regions。

先只保留：

```text
lane / shoulder / wide / invalid
```

验收：

```text
每个 slice 横向 region 顺序正确。
region 不跨越中间边界。
```

### Step 2: RegionTrackBuilder

把 SliceRegion 沿 s 串起来。

第一版匹配代价：

```text
boundary continuity
endpoint center_l continuity
endpoint width continuity
order consistency
gap penalty
```

验收：

```text
1823: 左侧 region 能跨 id/gap 串起来。
2014: 原 LT2/LT3 这类端点连续 region 能串起来。
不能把非相邻、不同横向层级的 region 串错。
```

### Step 3: RegionAdjacencyBuilder

根据共享边界建立 region track 之间的横向邻接图。

验收：

```text
L1 和 EGO 如果不共享边界，不能形成 high-confidence adjacency。
共享边界断裂时需要输出 boundary_gap / weak adjacency。
```

### Step 4: EgoRelativeLaneAssigner

从 EGO region 出发，沿 RegionAdjacency 图向左右命名 L1/R1/L2/R2。

验收：

```text
所有 L1/R1 assignment 都必须有 adjacency evidence。
unattached region 不能被强行命名为 L1/R1。
```

### Step 5: RibbonFeatureAnalyzer

对 assigned / unassigned region tracks 计算统计特征。

验收：

```text
stable lane 的 center_l_range 和 width_range 小。
transition 的 center_l_slope 或 width_slope 明显。
噪声 short/gap 明显。
```

### Step 6: TopologyHintDetector

只输出候选事件，不立即生成最终 RoadModel。

验收：

```text
每个 hint 都能回溯到 assignment + feature + adjacency evidence。
```

## 8. 风险和注意点

### 8.1 不要过早硬编码 L1/R1

L1/R1 是共享边界图上的命名结果，不是 offset 排序结果。

### 8.2 不要只用 center_l

center_l 连续只能证明几何接近，不能证明拓扑相邻。

### 8.3 不要把 wide transition 直接当 lane

wide transition 可以参与特征统计和事件候选，但不能默认分配为稳定车道。

### 8.4 不要让拓扑事件修正分配错误

如果 assignment 层已经错了，event detector 不应该用特殊规则让图看起来合理。应回到 assignment/adjacency 修。

### 8.5 端点连续比整段 median 更适合关联

纵向关联看的是前后段连接处是否连续。整段统计用于特征分析，不应用于判断两个段能否连接。

## 9. 当前推荐方向

下一步不应该继续增强 `LaneTrack`，也不应该直接写 add/drop/split 规则。

更合理的下一步是实现：

```text
lane_structure_debug.json
```

并优先可视化：

```text
1. RegionTrack
2. RegionAdjacency shared boundary
3. Ego/L1/R1 assignment
4. unattached candidates
5. ribbon feature hints
```

这个 debug 层能把问题定位清楚：

```text
如果 L1 分错：看 assignment / adjacency。
如果 L1 分对但形态异常：看 ribbon_features。
如果事件判断错：看 topology_hints。
```

最终目标不是让单条中心线看起来连续，而是得到一张可解释的局部车道关系图：

```text
RegionTrack graph + shared-boundary adjacency + ego-relative assignment
```

这张图才是后续拓扑事件检测和几何优化的基础。
