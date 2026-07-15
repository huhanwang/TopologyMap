#include "visual_reference_module.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>

#include "interface/pilot/perception/fused_road_geometry.pb.h"

namespace topology_map::algorithms {
namespace {

using idrive::workflow::proto::BevLaneLineType;
using idrive::workflow::proto::BevLaneLineType_Name;
using idrive::workflow::proto::BevLaneLineType_kCurb;
using idrive::workflow::proto::BevLaneLineType_kCurbBarrier;
using idrive::workflow::proto::BevLaneLineType_kHorizontalCurb;
using idrive::workflow::proto::BevLaneLineType_kMovableCurb;
using idrive::workflow::proto::BevLaneLineType_kStopLine;
using idrive::workflow::proto::FusedRoadGeometry;
using idrive::workflow::proto::ParameterizedLaneInfo;
using idrive::workflow::proto::PositionIndex;
using idrive::workflow::proto::PositionIndex_Name;
using idrive::workflow::proto::PositionIndex_HOST_LEFT;
using idrive::workflow::proto::PositionIndex_HOST_RIGHT;
using idrive::workflow::proto::PositionIndex_ROAD_EDGE_LEFT;
using idrive::workflow::proto::PositionIndex_ROAD_EDGE_RIGHT;

struct BoundaryLine {
    std::string id;
    std::string source;
    int lane_id = 0;
    PositionIndex lane_position = idrive::workflow::proto::PositionIndex_POSITION_INDEX_DEFAULT;
    std::string lane_position_name;
    std::string source_type = "lane_line";
    std::pair<double, double> s_range{0.0, 0.0};
    std::pair<double, double> d_range{0.0, 0.0};
    double length_m = 0.0;
    std::vector<double> coeffs;
    double confidence = 1.0;
    int point_count = 0;
    std::vector<std::string> section_types;
};

struct ReferenceCandidate {
    const BoundaryLine* left = nullptr;
    const BoundaryLine* right = nullptr;
    std::pair<double, double> s_range{0.0, 0.0};
    std::vector<double> center_coeffs;
    double confidence = 0.0;
    std::string method;
};

double polyValue(const std::vector<double>& coeffs, double x) {
    double total = 0.0;
    double power = 1.0;
    for (double coeff : coeffs) {
        total += coeff * power;
        power *= x;
    }
    return total;
}

double polylineLength(const std::vector<Vec2>& points) {
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        length += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    return length;
}

std::vector<double> meanCoeffs(const std::vector<double>& left, const std::vector<double>& right) {
    const std::size_t n = std::max(left.size(), right.size());
    std::vector<double> out(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const double a = i < left.size() ? left[i] : 0.0;
        const double b = i < right.size() ? right[i] : 0.0;
        out[i] = 0.5 * (a + b);
    }
    return out;
}

std::vector<double> laneCoeffs(const ParameterizedLaneInfo& lane,
                               const std::vector<Vec2>& points) {
    std::vector<double> coeffs;
    coeffs.reserve(lane.coeffs_size());
    for (int i = 0; i < lane.coeffs_size(); ++i) {
        coeffs.push_back(static_cast<double>(lane.coeffs(i)));
    }
    if (!coeffs.empty()) {
        return coeffs;
    }
    if (points.empty()) {
        return {};
    }
    const double sum_y = std::accumulate(
        points.begin(), points.end(), 0.0,
        [](double acc, const Vec2& point) { return acc + point.y; });
    return {sum_y / static_cast<double>(points.size()), 0.0, 0.0, 0.0};
}

std::string sourceType(const ParameterizedLaneInfo& lane) {
    if (lane.lane_position_index() == PositionIndex_ROAD_EDGE_LEFT ||
        lane.lane_position_index() == PositionIndex_ROAD_EDGE_RIGHT) {
        return "road_edge";
    }
    bool has_curb = false;
    bool has_stopline = false;
    for (const auto& section : lane.lanes()) {
        const auto type = section.bev_lane_type();
        if (type == BevLaneLineType_kCurb ||
            type == BevLaneLineType_kCurbBarrier ||
            type == BevLaneLineType_kHorizontalCurb ||
            type == BevLaneLineType_kMovableCurb) {
            has_curb = true;
        }
        if (type == BevLaneLineType_kStopLine) {
            has_stopline = true;
        }
    }
    if (has_curb) return "curb";
    if (has_stopline) return "stopline";
    return "lane_line";
}

std::vector<Vec2> lanePoints(const ParameterizedLaneInfo& lane) {
    std::vector<Vec2> points;
    for (const auto& section : lane.lanes()) {
        for (const auto& point : section.bev_points()) {
            points.push_back({point.x(), point.y()});
        }
    }
    return points;
}

std::vector<std::string> laneSectionTypes(const ParameterizedLaneInfo& lane) {
    std::vector<std::string> types;
    for (const auto& section : lane.lanes()) {
        types.push_back(BevLaneLineType_Name(section.bev_lane_type()));
    }
    return types;
}

template <typename LaneList>
void appendBoundaryLines(const LaneList& lanes, const std::string& source,
                         std::int64_t frame_id, std::vector<BoundaryLine>* out) {
    for (int i = 0; i < lanes.size(); ++i) {
        const auto& lane = lanes.Get(i);
        const auto points = lanePoints(lane);
        if (points.size() < 2) {
            continue;
        }
        double min_x = points.front().x;
        double max_x = points.front().x;
        double min_y = points.front().y;
        double max_y = points.front().y;
        for (const auto& point : points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }

        auto coeffs = laneCoeffs(lane, points);
        if (coeffs.empty()) {
            continue;
        }

        BoundaryLine line;
        line.source = source;
        line.lane_id = lane.lane_id();
        line.lane_position = lane.lane_position_index();
        line.lane_position_name = PositionIndex_Name(lane.lane_position_index());
        line.source_type = sourceType(lane);
        line.s_range = {min_x, max_x};
        line.d_range = {min_y, max_y};
        line.length_m = polylineLength(points);
        line.coeffs = std::move(coeffs);
        line.confidence = 1.0;
        line.point_count = static_cast<int>(points.size());
        line.section_types = laneSectionTypes(lane);
        line.id = "vision:fused_static:" + std::to_string(frame_id) + ":" +
                  source + ":" + std::to_string(line.lane_id) + ":" + std::to_string(i);
        out->push_back(std::move(line));
    }
}

bool isLongitudinalBoundary(const BoundaryLine& line,
                            const VisualReferenceModule::Config& cfg) {
    const double x_span = line.s_range.second - line.s_range.first;
    if (x_span < cfg.min_line_x_span_m) return false;
    if (std::max(std::abs(line.d_range.first), std::abs(line.d_range.second)) > cfg.max_abs_d_m) {
        return false;
    }
    if (line.source_type != "lane_line") return false;
    return true;
}

std::string selectSource(const std::vector<BoundaryLine>& lines,
                         const VisualReferenceModule::Config& cfg) {
    for (const auto& source : cfg.preferred_sources) {
        int count = 0;
        for (const auto& line : lines) {
            if (line.source == source && isLongitudinalBoundary(line, cfg)) {
                ++count;
            }
        }
        if (count >= 2) {
            return source;
        }
    }
    return cfg.preferred_sources.empty() ? "smooth_bev_lanes" : cfg.preferred_sources.front();
}

const BoundaryLine* bestHostLine(const std::vector<const BoundaryLine*>& lines,
                                 PositionIndex position) {
    const BoundaryLine* best = nullptr;
    for (const auto* line : lines) {
        if (line->lane_position != position) continue;
        if (!best || std::tie(line->length_m, line->confidence) >
                         std::tie(best->length_m, best->confidence)) {
            best = line;
        }
    }
    return best;
}

std::pair<double, double> extendedRange(
    double start,
    double end,
    const VisualReferenceModule::Config& cfg) {
    const double out_start = start - cfg.max_reference_extension_m;
    const double out_end = std::min(cfg.max_length_m, end + cfg.max_reference_extension_m);
    return {out_start, out_end};
}

std::optional<ReferenceCandidate> pairReference(
    const BoundaryLine& left,
    const BoundaryLine& right,
    const VisualReferenceModule::Config& cfg) {
    const double overlap_start = std::max(left.s_range.first, right.s_range.first);
    const double overlap_end = std::min(left.s_range.second, right.s_range.second);
    if (overlap_end - overlap_start < cfg.min_reference_length_m) {
        return std::nullopt;
    }
    ReferenceCandidate candidate;
    candidate.left = &left;
    candidate.right = &right;
    candidate.s_range = extendedRange(overlap_start, overlap_end, cfg);
    candidate.center_coeffs = meanCoeffs(left.coeffs, right.coeffs);
    candidate.confidence = std::min(left.confidence, right.confidence);
    candidate.method = "host_pair";
    return candidate;
}

std::optional<ReferenceCandidate> singleLineReference(
    const BoundaryLine& line,
    const VisualReferenceModule::Config& cfg,
    const std::string& method) {
    if (line.s_range.second - line.s_range.first < cfg.min_reference_length_m) {
        return std::nullopt;
    }
    ReferenceCandidate candidate;
    candidate.left = &line;
    candidate.right = &line;
    candidate.s_range = extendedRange(line.s_range.first, line.s_range.second, cfg);
    candidate.center_coeffs = line.coeffs;
    candidate.confidence = line.confidence;
    candidate.method = method;
    return candidate;
}

std::optional<ReferenceCandidate> selectReference(
    const std::vector<BoundaryLine>& source_lines,
    const VisualReferenceModule::Config& cfg) {
    std::vector<const BoundaryLine*> candidates;
    for (const auto& line : source_lines) {
        if (isLongitudinalBoundary(line, cfg)) {
            candidates.push_back(&line);
        }
    }
    const auto* host_left = bestHostLine(candidates, PositionIndex_HOST_LEFT);
    const auto* host_right = bestHostLine(candidates, PositionIndex_HOST_RIGHT);
    if (host_left && host_right) {
        auto candidate = pairReference(*host_left, *host_right, cfg);
        if (candidate) {
            return candidate;
        }
    }
    if (host_left) {
        return singleLineReference(*host_left, cfg, "host_left_single");
    }
    if (host_right) {
        return singleLineReference(*host_right, cfg, "host_right_single");
    }
    if (candidates.empty()) {
        return std::nullopt;
    }
    const auto* longest = *std::max_element(
        candidates.begin(),
        candidates.end(),
        [](const auto* a, const auto* b) {
            return std::tie(a->length_m, a->point_count) < std::tie(b->length_m, b->point_count);
        });
    if (!longest) {
        return std::nullopt;
    }
    return singleLineReference(*longest, cfg, "longest_lane_line");
}

std::vector<VisualReferencePoint> sampleReferencePoints(const ReferenceCandidate& ref,
                                                        double spacing_m) {
    std::vector<VisualReferencePoint> points;
    const double start = ref.s_range.first;
    const double end = ref.s_range.second;
    if (end <= start) {
        points.push_back({start, start, polyValue(ref.center_coeffs, start)});
        return points;
    }
    const int count = static_cast<int>(std::floor((end - start) / std::max(1e-6, spacing_m)));
    for (int i = 0; i <= count; ++i) {
        const double s = start + i * spacing_m;
        points.push_back({s, s, polyValue(ref.center_coeffs, s)});
    }
    if (std::abs(points.back().s - end) > 1e-6) {
        points.push_back({end, end, polyValue(ref.center_coeffs, end)});
    }
    return points;
}

}  // namespace

VisualReferenceResult VisualReferenceModule::process(
    std::int64_t frame_id,
    const idrive::workflow::proto::FusedStaticMsg& fused_static) const {
    VisualReferenceResult result;
    result.frame_id = frame_id;
    if (!fused_static.has_fused_road_geometry()) {
        result.error = "missing_fused_road_geometry";
        return result;
    }

    const auto& geo = fused_static.fused_road_geometry();
    std::vector<BoundaryLine> lines;
    appendBoundaryLines(geo.smooth_bev_lanes(), "smooth_bev_lanes", frame_id, &lines);
    appendBoundaryLines(geo.vehicle_bev_lanes(), "vehicle_bev_lanes", frame_id, &lines);

    const auto selected_source = selectSource(lines, cfg_);
    std::vector<BoundaryLine> source_lines;
    for (const auto& line : lines) {
        if (line.source == selected_source) {
            source_lines.push_back(line);
        }
    }
    auto reference = selectReference(source_lines, cfg_);
    if (!reference) {
        result.error = "no_visual_reference_candidate";
        result.selected_source = selected_source;
        result.input_line_count = static_cast<int>(lines.size());
        result.selected_source_line_count = static_cast<int>(source_lines.size());
        return result;
    }

    result.ok = true;
    result.selected_source = selected_source;
    result.method = reference->method;
    result.left_line_id = reference->left ? reference->left->id : "";
    result.right_line_id = reference->right ? reference->right->id : "";
    result.confidence = reference->confidence;
    result.s_range_m = reference->s_range;
    result.center_coeffs = reference->center_coeffs;
    result.points = sampleReferencePoints(*reference, cfg_.slice_spacing_m);
    result.input_line_count = static_cast<int>(lines.size());
    result.selected_source_line_count = static_cast<int>(source_lines.size());
    result.left_lane_position = reference->left ? reference->left->lane_position_name : "";
    result.right_lane_position = reference->right ? reference->right->lane_position_name : "";
    result.left_source_type = reference->left ? reference->left->source_type : "";
    result.right_source_type = reference->right ? reference->right->source_type : "";
    result.left_line_x_span_m = reference->left ? reference->left->s_range.second - reference->left->s_range.first : 0.0;
    result.right_line_x_span_m = reference->right ? reference->right->s_range.second - reference->right->s_range.first : 0.0;
    result.left_line_length_m = reference->left ? reference->left->length_m : 0.0;
    result.right_line_length_m = reference->right ? reference->right->length_m : 0.0;
    return result;
}

}  // namespace topology_map::algorithms
