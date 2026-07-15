#include "boundary_sampling_module.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "frenet/frenet_projector.h"
#include "interface/pilot/perception/fused_road_geometry.pb.h"

namespace topology_map::algorithms {
namespace {

using idrive::workflow::proto::BevLaneLineType_kCurb;
using idrive::workflow::proto::BevLaneLineType_kCurbBarrier;
using idrive::workflow::proto::BevLaneLineType_kHorizontalCurb;
using idrive::workflow::proto::BevLaneLineType_kMovableCurb;
using idrive::workflow::proto::BevLaneLineType_kStopLine;
using idrive::workflow::proto::FusedRoadGeometry;
using idrive::workflow::proto::ParameterizedLaneInfo;
using idrive::workflow::proto::PositionIndex_Name;
using idrive::workflow::proto::PositionIndex_ROAD_EDGE_LEFT;
using idrive::workflow::proto::PositionIndex_ROAD_EDGE_RIGHT;

struct RawSection {
    std::string source;
    std::string source_line_id;
    std::string track_line_id;
    int lane_id = 0;
    int lane_index = -1;
    int section_index = -1;
    std::string source_type;
    std::string lane_type;
    int lane_type_value = 0;
    std::string lane_position;
    double confidence = 1.0;
    std::vector<Vec2> points;
};

double cross(double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
}

std::string sourceType(const ParameterizedLaneInfo& lane, int section_index) {
    if (lane.lane_position_index() == PositionIndex_ROAD_EDGE_LEFT ||
        lane.lane_position_index() == PositionIndex_ROAD_EDGE_RIGHT) {
        return "road_edge";
    }
    if (section_index >= 0 && section_index < lane.lanes_size()) {
        const auto type = lane.lanes(section_index).bev_lane_type();
        if (type == BevLaneLineType_kStopLine) return "stopline";
        if (type == BevLaneLineType_kCurb ||
            type == BevLaneLineType_kCurbBarrier ||
            type == BevLaneLineType_kHorizontalCurb ||
            type == BevLaneLineType_kMovableCurb) {
            return "curb";
        }
    }
    return "lane_line";
}

bool keepSourceType(const std::string& source_type) {
    return source_type == "lane_line" || source_type == "road_edge" || source_type == "curb";
}

template <typename LaneList>
void appendRawSections(
    const LaneList& lanes,
    const std::string& source,
    std::int64_t frame_id,
    std::vector<RawSection>* out) {
    if (!out) return;
    for (int lane_index = 0; lane_index < lanes.size(); ++lane_index) {
        const auto& lane = lanes.Get(lane_index);
        const std::string lane_position = PositionIndex_Name(lane.lane_position_index());
        for (int section_index = 0; section_index < lane.lanes_size(); ++section_index) {
            const auto& section = lane.lanes(section_index);
            std::vector<Vec2> points;
            points.reserve(section.bev_points_size());
            for (const auto& point : section.bev_points()) {
                points.push_back({point.x(), point.y()});
            }
            if (points.size() < 2) continue;

            const std::string type = sourceType(lane, section_index);
            if (!keepSourceType(type)) continue;

            RawSection raw;
            raw.source = source;
            raw.lane_id = lane.lane_id();
            raw.lane_index = lane_index;
            raw.section_index = section_index;
            raw.source_type = type;
            raw.lane_type = idrive::workflow::proto::BevLaneLineType_Name(section.bev_lane_type());
            raw.lane_type_value = static_cast<int>(section.bev_lane_type());
            raw.lane_position = lane_position;
            raw.points = std::move(points);
            raw.track_line_id = "vision:fused_static:" + std::to_string(frame_id) + ":" +
                                source + ":" + std::to_string(raw.lane_id) + ":" +
                                std::to_string(lane_index);
            raw.source_line_id = raw.track_line_id + ":" + std::to_string(section_index);
            out->push_back(std::move(raw));
        }
    }
}

std::vector<RawSection> extractRawSections(
    std::int64_t frame_id,
    const FusedRoadGeometry& geometry) {
    std::vector<RawSection> sections;
    appendRawSections(geometry.vehicle_bev_lanes(), "vehicle_bev_lanes", frame_id, &sections);
    appendRawSections(geometry.smooth_bev_lanes(), "smooth_bev_lanes", frame_id, &sections);
    return sections;
}

bool intersectSegmentWithNormalLine(
    const Vec2& a,
    const Vec2& b,
    double origin_x,
    double origin_y,
    double normal_x,
    double normal_y,
    double half_length_m,
    double* hit_x,
    double* hit_y,
    double* offset_m) {
    const double sx = b.x - a.x;
    const double sy = b.y - a.y;
    const double denom = cross(normal_x, normal_y, sx, sy);
    if (std::abs(denom) < 1e-9) return false;

    const double rel_x = a.x - origin_x;
    const double rel_y = a.y - origin_y;
    const double t = cross(rel_x, rel_y, sx, sy) / denom;
    const double u = cross(rel_x, rel_y, normal_x, normal_y) / denom;
    if (u < -1e-6 || u > 1.0 + 1e-6) return false;
    if (std::abs(t) > half_length_m + 1e-6) return false;
    if (!hit_x || !hit_y || !offset_m) return false;
    *hit_x = origin_x + normal_x * t;
    *hit_y = origin_y + normal_y * t;
    *offset_m = t;
    return true;
}

}  // namespace

BoundarySamplingModule::BoundarySamplingModule()
    : cfg_(Config{}) {}

BoundarySamplingModule::BoundarySamplingModule(Config config)
    : cfg_(std::move(config)) {}

BoundarySamplingResult BoundarySamplingModule::process(
    std::int64_t frame_id,
    const FusedReferenceResult& reference,
    const idrive::workflow::proto::FusedStaticMsg& fused_static) const {
    BoundarySamplingResult result;
    result.frame_id = frame_id;
    if (!reference.ok || reference.points.size() < 2) {
        result.error = "invalid_fused_reference";
        return result;
    }
    if (!fused_static.has_fused_road_geometry()) {
        result.error = "missing_fused_road_geometry";
        return result;
    }

    const auto sections = extractRawSections(frame_id, fused_static.fused_road_geometry());
    result.source_section_count = static_cast<int>(sections.size());
    const FrenetProjector projector(reference);
    if (!projector.ok()) {
        result.error = projector.error();
        return result;
    }
    const auto s_values = projector.sampleSValues(cfg_.sample_spacing_m);
    for (double s : s_values) {
        BoundaryIntersectionSlice slice;
        slice.s = s;
        FrenetPose pose;
        if (!projector.poseAt(s, &pose)) {
            continue;
        }
        slice.origin_x = pose.x;
        slice.origin_y = pose.y;
        slice.normal_x = pose.normal_x;
        slice.normal_y = pose.normal_y;
        for (const auto& section : sections) {
            for (std::size_t i = 1; i < section.points.size(); ++i) {
                double hit_x = 0.0;
                double hit_y = 0.0;
                double offset = 0.0;
                if (!intersectSegmentWithNormalLine(
                        section.points[i - 1],
                        section.points[i],
                        slice.origin_x,
                        slice.origin_y,
                        slice.normal_x,
                        slice.normal_y,
                        cfg_.half_length_m,
                        &hit_x,
                        &hit_y,
                        &offset)) {
                    continue;
                }
                if (std::abs(offset) > cfg_.max_abs_offset_m) continue;

                BoundaryIntersectionHit hit;
                hit.s = s;
                hit.x = hit_x;
                hit.y = hit_y;
                hit.offset_m = offset;
                hit.source = section.source;
                hit.source_line_id = section.source_line_id;
                hit.track_line_id = section.track_line_id;
                hit.lane_id = section.lane_id;
                hit.lane_index = section.lane_index;
                hit.section_index = section.section_index;
                hit.source_type = section.source_type;
                hit.lane_type = section.lane_type;
                hit.lane_type_value = section.lane_type_value;
                hit.lane_position = section.lane_position;
                hit.confidence = section.confidence;
                hit.id = "raw_hit:" + std::to_string(frame_id) + ":" +
                         std::to_string(static_cast<int>(std::round(s * 1000.0))) + ":" +
                         section.source + ":" + std::to_string(section.lane_id) + ":" +
                         std::to_string(section.lane_index) + ":" +
                         std::to_string(section.section_index) + ":" +
                         std::to_string(i - 1);
                slice.hits.push_back(std::move(hit));
            }
        }
        std::sort(slice.hits.begin(), slice.hits.end(), [](const auto& a, const auto& b) {
            return a.offset_m < b.offset_m;
        });
        result.hit_count += static_cast<int>(slice.hits.size());
        result.slices.push_back(std::move(slice));
    }
    result.ok = true;
    return result;
}

}  // namespace topology_map::algorithms
