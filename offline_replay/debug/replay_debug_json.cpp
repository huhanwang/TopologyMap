#include "replay_debug_json.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "frenet/frenet_lane_line_completer.h"
#include "frenet/frenet_lane_region_builder.h"
#include "frenet/frenet_completed_boundary_builder.h"
#include "frenet/frenet_junction_analyzer.h"
#include "frenet/frenet_ribbon_builder.h"
#include "frenet/frenet_track_builder.h"
#include "interface/pilot/fused_static.pb.h"
#include "interface/pilot/perception/bev_lane_base.pb.h"
#include "interface/pilot/perception/fused_road_geometry.pb.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace offline_replay::debug {
namespace {

using json = nlohmann::json;
constexpr double kEarthRadius = 6378137.0;

json makeColor(const char* value) {
    return value;
}

bool projectGeoPointToVcsFromGnss(double lat, double lon, double alt,
                                  const snoah::GnssRawReadingProto& gnss,
                                  json* out_point);

std::string ribbonWidthClass(double width_m) {
    if (width_m < 0.3) return "invalid";
    if (width_m < 2.2) return "shoulder";
    if (width_m < 2.7) return "narrow";
    if (width_m <= 4.8) return "lane";
    if (width_m <= 6.5) return "wide";
    return "invalid";
}

std::string ribbonSequenceKey(const json& ribbons) {
    std::string key;
    if (!ribbons.is_array()) return key;
    for (const auto& ribbon : ribbons) {
        if (!key.empty()) key += "|";
        key += ribbon.value("width_class", "unknown");
    }
    return key;
}

double medianValue(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if (values.size() % 2 == 1) return values[mid];
    return 0.5 * (values[mid - 1] + values[mid]);
}

double standardDeviation(const std::vector<double>& values, double mean) {
    if (values.size() < 2) return 0.0;
    double sum = 0.0;
    for (double value : values) {
        const double delta = value - mean;
        sum += delta * delta;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

struct CompletionObservation {
    int slice_index = -1;
    double s = 0.0;
    double l = 0.0;
    double x = 0.0;
    double y = 0.0;
    double normal_x = 0.0;
    double normal_y = 1.0;
    std::string line_id;
    std::string source_type;
    double confidence = 1.0;
};

struct CompletionPairSample {
    int slice_index = -1;
    double s = 0.0;
    std::string right_id;
    std::string left_id;
    double right_l = 0.0;
    double left_l = 0.0;
    double width = 0.0;
    std::string right_source_type;
    std::string left_source_type;
};

struct CompletionPairRelation {
    std::string id;
    std::string right_id;
    std::string left_id;
    std::string right_source_type;
    std::string left_source_type;
    std::vector<CompletionPairSample> samples;
};

struct CompletionRatioRelation {
    std::string id;
    std::string right_id;
    std::string middle_id;
    std::string left_id;
    std::string middle_source_type;
    std::vector<int> slice_indices;
    std::vector<double> ratios;
};

std::string completionPairId(const std::string& right_id, const std::string& left_id) {
    return right_id + " -> " + left_id;
}

const CompletionObservation* observationAt(
    const std::map<std::string, std::map<int, CompletionObservation>>& by_line_slice,
    const std::string& line_id,
    int slice_index) {
    auto line_it = by_line_slice.find(line_id);
    if (line_it == by_line_slice.end()) return nullptr;
    auto slice_it = line_it->second.find(slice_index);
    if (slice_it == line_it->second.end()) return nullptr;
    return &slice_it->second;
}

bool completionCanSupport(const CompletionObservation* obs) {
    if (!obs) return false;
    const std::string type = obs->source_type;
    return type != "curb" && type != "road_edge" && type != "edge";
}

bool completionCanBeInferred(const std::string& source_type) {
    return source_type == "lane_line" || source_type == "lane_boundary" ||
           source_type == "regularized_missing_boundary_role" || source_type.empty();
}

void putCompletionObservation(
    std::map<std::string, std::map<int, CompletionObservation>>& by_line_slice,
    const json& node) {
    CompletionObservation obs;
    obs.slice_index = node.value("slice_index", -1);
    obs.s = node.value("s", 0.0);
    obs.l = node.value("l", 0.0);
    obs.x = node.value("x", 0.0);
    obs.y = node.value("y", 0.0);
    obs.line_id = node.value("line_id", "");
    obs.source_type = node.value("source_type", "");
    obs.confidence = node.value("confidence", 0.0);
    by_line_slice[obs.line_id][obs.slice_index] = obs;
}

json completionNodeJson(
    const CompletionPairRelation& relation,
    const CompletionPairSample& sample,
    const CompletionObservation& anchor,
    bool infer_left,
    const std::string& pass) {
    const double inferred_l = infer_left
        ? anchor.l + sample.width
        : anchor.l - sample.width;
    return {
        {"id", "completion:" + pass + ":" + relation.id + ":" + std::to_string(sample.slice_index) + ":" + (infer_left ? "left" : "right")},
        {"s", sample.s},
        {"l", inferred_l},
        {"x", anchor.x + (inferred_l - anchor.l) * anchor.normal_x},
        {"y", anchor.y + (inferred_l - anchor.l) * anchor.normal_y},
        {"slice_index", sample.slice_index},
        {"line_id", infer_left ? relation.left_id : relation.right_id},
        {"source_type", infer_left ? relation.left_source_type : relation.right_source_type},
        {"state", infer_left ? "inferred_left_from_pair_width" : "inferred_right_from_pair_width"},
        {"pass", pass},
        {"relation_id", relation.id},
        {"anchor_line_id", anchor.line_id},
        {"anchor_l_m", anchor.l},
        {"expected_width_m", sample.width},
        {"confidence", anchor.confidence * 0.55}
    };
}

json geoPointToJson(const snoah::mapping::GeoPointProto& point) {
    return {
        {"latitude", point.latitude()},
        {"longitude", point.longitude()},
        {"altitude", point.altitude()}
    };
}

double polylineLength2d(const json& points) {
    if (!points.is_array() || points.size() < 2) return 0.0;
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double dx = points[i][0].get<double>() - points[i - 1][0].get<double>();
        const double dy = points[i][1].get<double>() - points[i - 1][1].get<double>();
        length += std::hypot(dx, dy);
    }
    return length;
}

template <typename PointRange>
json projectGeoPolylineToJson(const PointRange& points,
                              const snoah::GnssRawReadingProto* gnss) {
    json projected = json::array();
    if (!gnss || !gnss->has_latitude() || !gnss->has_longitude()) {
        return projected;
    }
    for (const auto& point : points) {
        json vcs_point;
        if (projectGeoPointToVcsFromGnss(
                point.latitude(), point.longitude(), point.altitude(), *gnss, &vcs_point)) {
            projected.push_back(std::move(vcs_point));
        }
    }
    return projected;
}

json projectedSummaryToJson(const json& projected) {
    json summary;
    summary["projected_point_count"] = projected.is_array() ? projected.size() : 0;
    summary["projected_length_m"] = polylineLength2d(projected);
    if (projected.is_array() && !projected.empty()) {
        summary["first_vcs"] = projected.front();
        summary["last_vcs"] = projected.back();
    }
    return summary;
}

json navigationEventToJson(
    const topology_map::algorithms::NavigationSemanticEvent& event) {
    return {
        {"type", topology_map::algorithms::navigationEventTypeName(event.type)},
        {"label", event.label},
        {"instruction", event.instruction},
        {"segment_index", event.segment_index},
        {"distance_to_event_m", event.distance_to_event_m}
    };
}

const char* laneColor(idrive::workflow::proto::BevLaneLineType type) {
    using idrive::workflow::proto::BevLaneLineType_kCurb;
    using idrive::workflow::proto::BevLaneLineType_kCurbBarrier;
    using idrive::workflow::proto::BevLaneLineType_kHorizontalCurb;
    using idrive::workflow::proto::BevLaneLineType_kLeftDashedRightSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kMovableCurb;
    using idrive::workflow::proto::BevLaneLineType_kRightDashedLeftSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kSpeedBump;
    using idrive::workflow::proto::BevLaneLineType_kStopLine;
    using idrive::workflow::proto::BevLaneLineType_kWhiteDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kWhiteDoubleDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kWhiteDoubleSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kWhiteSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kYellowDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kYellowDoubleDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kYellowDoubleSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kYellowSolidLine;

    switch (type) {
        case BevLaneLineType_kWhiteSolidLine:
        case BevLaneLineType_kWhiteDashedLine:
        case BevLaneLineType_kWhiteDoubleSolidLine:
        case BevLaneLineType_kWhiteDoubleDashedLine:
        case BevLaneLineType_kLeftDashedRightSolidLine:
        case BevLaneLineType_kRightDashedLeftSolidLine:
            return "#f4f4f2";
        case BevLaneLineType_kYellowSolidLine:
        case BevLaneLineType_kYellowDashedLine:
        case BevLaneLineType_kYellowDoubleSolidLine:
        case BevLaneLineType_kYellowDoubleDashedLine:
            return "#f2c94c";
        case BevLaneLineType_kCurb:
        case BevLaneLineType_kCurbBarrier:
        case BevLaneLineType_kHorizontalCurb:
        case BevLaneLineType_kMovableCurb:
            return "#eb5757";
        case BevLaneLineType_kStopLine:
            return "#56ccf2";
        case BevLaneLineType_kSpeedBump:
            return "#bb6bd9";
        default:
            return "#f2c94c";
    }
}

bool isDashed(idrive::workflow::proto::BevLaneLineType type) {
    using idrive::workflow::proto::BevLaneLineType_kLeftDashedRightSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kRightDashedLeftSolidLine;
    using idrive::workflow::proto::BevLaneLineType_kWhiteDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kWhiteDoubleDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kYellowDashedLine;
    using idrive::workflow::proto::BevLaneLineType_kYellowDoubleDashedLine;
    return type == BevLaneLineType_kWhiteDashedLine ||
           type == BevLaneLineType_kYellowDashedLine ||
           type == BevLaneLineType_kWhiteDoubleDashedLine ||
           type == BevLaneLineType_kYellowDoubleDashedLine ||
           type == BevLaneLineType_kLeftDashedRightSolidLine ||
           type == BevLaneLineType_kRightDashedLeftSolidLine;
}

double lineWidth(idrive::workflow::proto::BevLaneLineType type) {
    using idrive::workflow::proto::BevLaneLineType_kCurb;
    using idrive::workflow::proto::BevLaneLineType_kCurbBarrier;
    using idrive::workflow::proto::BevLaneLineType_kHorizontalCurb;
    using idrive::workflow::proto::BevLaneLineType_kMovableCurb;
    if (type == BevLaneLineType_kCurb ||
        type == BevLaneLineType_kCurbBarrier ||
        type == BevLaneLineType_kHorizontalCurb ||
        type == BevLaneLineType_kMovableCurb) {
        return 0.12;
    }
    return 0.06;
}

bool projectGeoPointToVcsFromGnss(double lat, double lon, double alt,
                                  const snoah::GnssRawReadingProto& gnss,
                                  json* out_point) {
    if (!out_point || !gnss.has_latitude() || !gnss.has_longitude()) return false;
    const double ego_lat = gnss.latitude();
    const double ego_lon = gnss.longitude();
    const double ego_yaw = gnss.has_yaw() ? gnss.yaw() : 0.0;

    const double dx = (lon - ego_lon) * std::cos(ego_lat) * kEarthRadius;
    const double dy = (lat - ego_lat) * kEarthRadius;
    const double c = std::cos(ego_yaw);
    const double s = std::sin(ego_yaw);
    const double vx = dx * c + dy * s;
    const double vy = -dx * s + dy * c;
    *out_point = {vx, vy, alt};
    return true;
}

}  // namespace

json preprocessedSnapshotToJson(
    const offline_replay::algorithms::PreprocessedSnapshot& preprocessed) {
    json j = json::object();
    for (const auto& [topic, entry] : preprocessed.entries) {
        json item;
        item["ok"] = entry.ok;
        item["proto_type"] = entry.proto_type;
        if (!entry.error.empty()) item["error"] = entry.error;
        j[topic] = std::move(item);
    }
    return j;
}

json buildReplayVizJson(
    const SnapshotFrame& snapshot,
    const offline_replay::algorithms::PreprocessedSnapshot& preprocessed) {
    json viz;
    viz["frame_id"] = snapshot.main_frame_id;
    viz["timestamp_us"] = snapshot.main_time_us;
    viz["layers"] = json::array();

    const auto it = preprocessed.entries.find("FusedStatic");
    if (it == preprocessed.entries.end() || !it->second.ok || !it->second.message) {
        return viz;
    }

    auto* fused = dynamic_cast<idrive::workflow::proto::FusedStaticMsg*>(
        it->second.message.get());
    if (!fused || !fused->has_fused_road_geometry()) {
        return viz;
    }

    json layer;
    layer["id"] = "fused_static_lanes";
    layer["name"] = "FusedStatic lanes";
    layer["visible"] = true;
    layer["items"] = json::array();

    const auto& geo = fused->fused_road_geometry();
    viz["debug"] = {
        {"vehicle_bev_lanes_size", geo.vehicle_bev_lanes_size()},
        {"smooth_bev_lanes_size", geo.smooth_bev_lanes_size()},
        {"center_lines_size", geo.center_lines_size()},
        {"center_lines_smooth_size", geo.center_lines_smooth_size()}
    };

    auto append_lane_infos = [&](const auto& lane_infos, const std::string& source) {
        for (int i = 0; i < lane_infos.size(); ++i) {
            const auto& lane_info = lane_infos.Get(i);
            for (int k = 0; k < lane_info.lanes_size(); ++k) {
                const auto& section = lane_info.lanes(k);
                if (section.bev_points_size() < 2) continue;
                const auto lane_type = section.bev_lane_type();

                json points = json::array();
                for (int j = 0; j < section.bev_points_size(); ++j) {
                    const auto& point = section.bev_points(j);
                    points.push_back({point.x(), point.y(), point.z()});
                }

                const std::string lane_id =
                    lane_info.lane_id() != 0 ? std::to_string(lane_info.lane_id()) : "no_id";
                json item;
                item["type"] = "polyline";
                item["id"] = source + "_boundary_" + lane_id + "_" + std::to_string(i) +
                             "_" + std::to_string(k);
                item["name"] = item["id"];
                item["points"] = std::move(points);
                item["style"] = {
                    {"color", makeColor(laneColor(lane_type))},
                    {"width", lineWidth(lane_type)},
                    {"dash", isDashed(lane_type)}
                };
                item["properties"] = {
                    {"source", source},
                    {"lane_id", lane_id},
                    {"lane_type", idrive::workflow::proto::BevLaneLineType_Name(lane_type)},
                    {"lane_type_value", static_cast<int>(lane_type)},
                    {"param_lane_index", i},
                    {"section_index", k}
                };
                layer["items"].push_back(std::move(item));
            }
        }
    };

    append_lane_infos(geo.vehicle_bev_lanes(), "vehicle_bev_lanes");
    viz["layers"].push_back(std::move(layer));

    const auto gnss_it = preprocessed.entries.find("AutoSensorGnss");
    const auto route_it = preprocessed.entries.find("AutoSDRoute");
    const auto* gnss = (gnss_it != preprocessed.entries.end() && gnss_it->second.ok)
        ? dynamic_cast<snoah::GnssRawReadingProto*>(gnss_it->second.message.get())
        : nullptr;
    const auto* route = (route_it != preprocessed.entries.end() && route_it->second.ok)
        ? dynamic_cast<snoah::SDRouteProto*>(route_it->second.message.get())
        : nullptr;

    json route_debug = {
        {"has_gnss", gnss != nullptr},
        {"has_route", route != nullptr},
        {"sd_sections_size", route ? route->sd_sections_size() : 0},
        {"navigation_segments_size", route ? route->navigation_segments_size() : 0},
        {"projected_items", 0}
    };
    if (gnss && route && gnss->has_latitude() && gnss->has_longitude()) {
        json route_layer;
        route_layer["id"] = "sd_route_vcs";
        route_layer["name"] = "AutoSDRoute projected to VCS";
        route_layer["visible"] = true;
        route_layer["items"] = json::array();

        auto append_route_points = [&](const auto& points, const std::string& id,
                                       const std::string& name, const char* color,
                                       bool dash, double width,
                                       const std::string& source,
                                       const std::string& instruction) {
            if (points.size() < 2) return;
            json projected = json::array();
            for (const auto& point : points) {
                json vcs_point;
                if (projectGeoPointToVcsFromGnss(
                        point.latitude(), point.longitude(), point.altitude(),
                        *gnss, &vcs_point)) {
                    projected.push_back(std::move(vcs_point));
                }
            }
            if (projected.size() < 2) return;
            json item;
            item["type"] = "polyline";
            item["id"] = id;
            item["name"] = name;
            item["points"] = std::move(projected);
            item["style"] = {
                {"color", makeColor(color)},
                {"width", width},
                {"dash", dash}
            };
            item["properties"] = {
                {"source", source},
                {"route_id", std::to_string(route->route_id())}
            };
            if (!instruction.empty()) item["properties"]["instruction"] = instruction;
            route_layer["items"].push_back(std::move(item));
            route_debug["projected_items"] = route_debug["projected_items"].get<int>() + 1;
        };

        int section_idx = 0;
        for (const auto& section : route->sd_sections()) {
            int link_idx = 0;
            for (const auto& link : section.sd_links()) {
                append_route_points(
                    link.points(),
                    "sd_route_link_" + std::to_string(section_idx) + "_" + std::to_string(link_idx),
                    "SD Route Link " + std::to_string(link.link_id()),
                    "#ff9628",
                    false,
                    0.16,
                    "AutoSDRoute.sd_links",
                    "");
                ++link_idx;
            }
            ++section_idx;
        }

        int nav_idx = 0;
        for (const auto& segment : route->navigation_segments()) {
            append_route_points(
                segment.points(),
                "sd_route_nav_" + std::to_string(nav_idx),
                "Navigation Segment " + std::to_string(nav_idx),
                "#ff28a0",
                true,
                0.24,
                "AutoSDRoute.navigation_segments",
                segment.instruction());
            ++nav_idx;
        }

        if (!route_layer["items"].empty()) {
            viz["layers"].push_back(std::move(route_layer));
        }
    }
    viz["debug"]["sd_route_vcs"] = std::move(route_debug);
    return viz;
}

json sdRouteDebugToJson(
    const SnapshotFrame& snapshot,
    const offline_replay::algorithms::PreprocessedSnapshot& preprocessed) {
    json j;
    j["schema_version"] = "topology-map.sd-route-debug.v1";
    j["frame_id"] = snapshot.main_frame_id;
    j["timestamp_us"] = snapshot.main_time_us;

    const auto gnss_it = preprocessed.entries.find("AutoSensorGnss");
    const auto route_it = preprocessed.entries.find("AutoSDRoute");
    const auto* gnss = (gnss_it != preprocessed.entries.end() && gnss_it->second.ok)
        ? dynamic_cast<const snoah::GnssRawReadingProto*>(gnss_it->second.message.get())
        : nullptr;
    const auto* route = (route_it != preprocessed.entries.end() && route_it->second.ok)
        ? dynamic_cast<const snoah::SDRouteProto*>(route_it->second.message.get())
        : nullptr;

    j["has_gnss"] = gnss != nullptr;
    j["has_route"] = route != nullptr;
    if (!route) {
        j["error"] = "missing_or_unparsed_auto_sd_route";
        return j;
    }

    j["route"] = {
        {"route_id", route->has_route_id() ? json(route->route_id()) : json(nullptr)},
        {"status", route->has_status() ? json(snoah::SDRouteProto_Status_Name(route->status())) : json(nullptr)},
        {"status_value", route->has_status() ? json(static_cast<int>(route->status())) : json(nullptr)},
        {"length", route->has_length() ? json(route->length()) : json(nullptr)},
        {"eta", route->has_eta() ? json(route->eta()) : json(nullptr)},
        {"request_reason", route->has_request_reason()
             ? json(snoah::SDRouteProto_RequestReason_Name(route->request_reason()))
             : json(nullptr)},
        {"request_reason_value", route->has_request_reason()
             ? json(static_cast<int>(route->request_reason()))
             : json(nullptr)},
        {"way_points_size", route->way_points_size()},
        {"sd_sections_size", route->sd_sections_size()},
        {"navigation_segments_size", route->navigation_segments_size()},
        {"traffic_message_channels_size", route->traffic_message_channels_size()}
    };

    j["way_points"] = json::array();
    for (int i = 0; i < route->way_points_size(); ++i) {
        const auto& point = route->way_points(i);
        json item = {
            {"index", i},
            {"geo", geoPointToJson(point)}
        };
        if (gnss) {
            json vcs_point;
            if (projectGeoPointToVcsFromGnss(
                    point.latitude(), point.longitude(), point.altitude(), *gnss, &vcs_point)) {
                item["vcs"] = std::move(vcs_point);
            }
        }
        j["way_points"].push_back(std::move(item));
    }

    j["navigation_segments"] = json::array();
    for (int i = 0; i < route->navigation_segments_size(); ++i) {
        const auto& segment = route->navigation_segments(i);
        const json projected = projectGeoPolylineToJson(segment.points(), gnss);
        json item = {
            {"index", i},
            {"point_count", segment.points_size()},
            {"instruction", segment.has_instruction() ? json(segment.instruction()) : json(nullptr)},
            {"exit_direction_info", segment.has_exit_direction_info()
                 ? json(segment.exit_direction_info())
                 : json(nullptr)},
            {"exit_name", segment.has_exit_name() ? json(segment.exit_name()) : json(nullptr)},
            {"crossing_name", segment.has_crossing_name() ? json(segment.crossing_name()) : json(nullptr)}
        };
        if (segment.points_size() > 0) {
            item["first_geo"] = geoPointToJson(segment.points(0));
            item["last_geo"] = geoPointToJson(segment.points(segment.points_size() - 1));
        }
        item["projection"] = projectedSummaryToJson(projected);
        item["projected_points_vcs"] = projected;
        j["navigation_segments"].push_back(std::move(item));
    }

    j["sd_sections"] = json::array();
    for (int section_idx = 0; section_idx < route->sd_sections_size(); ++section_idx) {
        const auto& section = route->sd_sections(section_idx);
        json section_json = {
            {"index", section_idx},
            {"length", section.has_length() ? json(section.length()) : json(nullptr)},
            {"sd_links_size", section.sd_links_size()},
            {"links", json::array()}
        };
        if (section.has_start_point()) section_json["start_point"] = geoPointToJson(section.start_point());
        if (section.has_end_point()) section_json["end_point"] = geoPointToJson(section.end_point());

        for (int link_idx = 0; link_idx < section.sd_links_size(); ++link_idx) {
            const auto& link = section.sd_links(link_idx);
            const json projected = projectGeoPolylineToJson(link.points(), gnss);
            json link_json = {
                {"index", link_idx},
                {"link_id", link.has_link_id() ? json(link.link_id()) : json(nullptr)},
                {"point_count", link.points_size()},
                {"length", link.has_length() ? json(link.length()) : json(nullptr)},
                {"road_name", link.has_road_name() ? json(link.road_name()) : json(nullptr)},
                {"service_name", link.has_service_name() ? json(link.service_name()) : json(nullptr)},
                {"road_class", link.has_road_class()
                     ? json(snoah::SDLink_FunctionalRoadClass_Name(link.road_class()))
                     : json(nullptr)},
                {"form_of_way", link.has_form_of_way()
                     ? json(snoah::SDLink_FormOfWay_Name(link.form_of_way()))
                     : json(nullptr)},
                {"road_type", link.has_road_type()
                     ? json(snoah::SDLink_Type_Name(link.road_type()))
                     : json(nullptr)},
                {"lane_num", link.has_lane_num() ? json(link.lane_num()) : json(nullptr)},
                {"turn_info_size", link.turn_info_size()},
                {"facilities_size", link.facilities_size()},
                {"camera_monitors_size", link.camera_monitors_size()}
            };
            if (link.points_size() > 0) {
                link_json["first_geo"] = geoPointToJson(link.points(0));
                link_json["last_geo"] = geoPointToJson(link.points(link.points_size() - 1));
            }
            link_json["projection"] = projectedSummaryToJson(projected);
            link_json["projected_points_vcs"] = projected;
            section_json["links"].push_back(std::move(link_json));
        }
        j["sd_sections"].push_back(std::move(section_json));
    }

    return j;
}

json navigationTrackerDebugToJson(
    const topology_map::algorithms::NavigationRouteSnapshot& route,
    const topology_map::algorithms::NavigationMatchResult& match) {
    json j;
    j["schema_version"] = "topology-map.navigation-tracker-debug.v1";
    j["route"] = {
        {"valid", route.valid},
        {"route_changed", route.route_changed},
        {"route_id", route.route_id},
        {"route_signature", route.route_signature},
        {"origin_latitude", route.origin_latitude},
        {"origin_longitude", route.origin_longitude},
        {"segment_count", route.segment_count}
    };
    j["match"] = {
        {"matched", match.matched},
        {"error", match.error},
        {"segment_index", match.segment_index},
        {"along_s_m", match.along_s_m},
        {"lateral_error_m", match.lateral_error_m},
        {"distance_to_segment_end_m", match.distance_to_segment_end_m},
        {"near_segment_end", match.near_segment_end},
        {"can_use_navigation_reference", match.can_use_navigation_reference},
        {"current_terminal_event", navigationEventToJson(match.current_terminal_event)},
        {"upcoming_event", navigationEventToJson(match.upcoming_event)}
    };
    j["segments"] = json::array();
    for (const auto& segment : route.segments) {
        json item = {
            {"index", segment.index},
            {"instruction", segment.instruction},
            {"crossing_name", segment.crossing_name},
            {"exit_direction_info", segment.exit_direction_info},
            {"exit_name", segment.exit_name},
            {"length_m", segment.length_m},
            {"point_count", segment.points.size()},
            {"terminal_event", navigationEventToJson(segment.terminal_event)}
        };
        if (!segment.points.empty()) {
            const auto& first = segment.points.front();
            const auto& last = segment.points.back();
            item["first_point"] = {
                {"latitude", first.latitude},
                {"longitude", first.longitude},
                {"altitude", first.altitude},
                {"x_m", first.x_m},
                {"y_m", first.y_m},
                {"s_m", first.s_m}
            };
            item["last_point"] = {
                {"latitude", last.latitude},
                {"longitude", last.longitude},
                {"altitude", last.altitude},
                {"x_m", last.x_m},
                {"y_m", last.y_m},
                {"s_m", last.s_m}
            };
        }
        j["segments"].push_back(std::move(item));
    }
    return j;
}

json navigationReferenceToJson(
    const topology_map::algorithms::NavigationReferenceResult& result) {
    json j;
    j["schema_version"] = "topology-map.navigation-reference.v1";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["backward_length_m"] = result.backward_length_m;
    j["forward_length_m"] = result.forward_length_m;
    j["lateral_error_m"] = result.lateral_error_m;
    j["heading_error_rad"] = result.heading_error_rad;
    j["stop_reason_forward"] = result.stop_reason_forward;
    j["stop_reason_backward"] = result.stop_reason_backward;
    j["points"] = json::array();
    for (const auto& point : result.vcs_points) {
        j["points"].push_back({point.x, point.y, 0.0});
    }
    return j;
}

json navigationReferenceVizLayer(
    const topology_map::algorithms::NavigationReferenceResult& result) {
    json layer;
    layer["id"] = "navigation_reference";
    layer["name"] = "Navigation reference";
    layer["visible"] = true;
    layer["items"] = json::array();
    if (!result.ok || result.vcs_points.size() < 2) {
        return layer;
    }

    json points = json::array();
    for (const auto& point : result.vcs_points) {
        points.push_back({point.x, point.y, 0.0});
    }

    json item;
    item["type"] = "polyline";
    item["id"] = "navigation_reference_center";
    item["name"] = "Navigation reference";
    item["points"] = std::move(points);
    item["style"] = {
        {"color", "#2d9cdb"},
        {"width", 0.16},
        {"dash", false}
    };
    item["properties"] = {
        {"source", "navigation_reference_module"},
        {"forward_length_m", result.forward_length_m},
        {"backward_length_m", result.backward_length_m},
        {"lateral_error_m", result.lateral_error_m},
        {"heading_error_rad", result.heading_error_rad},
        {"stop_reason_forward", result.stop_reason_forward},
        {"stop_reason_backward", result.stop_reason_backward}
    };
    layer["items"].push_back(std::move(item));
    return layer;
}

json fusedReferenceToJson(
    const topology_map::algorithms::FusedReferenceResult& result) {
    json j;
    j["schema_version"] = "topology-map.fused-reference.v1";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["method"] = result.method;
    j["confidence"] = result.confidence;
    j["used_navigation"] = result.used_navigation;
    j["lateral_offset_m"] = result.lateral_offset_m;
    j["heading_error_rad"] = result.heading_error_rad;
    j["overlap_length_m"] = result.overlap_length_m;
    j["visual_end_x_m"] = result.visual_end_x_m;
    j["fused_start_x_m"] = result.fused_start_x_m;
    j["fused_end_x_m"] = result.fused_end_x_m;
    j["points"] = json::array();
    for (const auto& point : result.points) {
        j["points"].push_back({
            {"s", point.s},
            {"x", point.x},
            {"y", point.y},
            {"heading_rad", point.heading_rad},
            {"curvature_m_inv", point.curvature_m_inv},
            {"source", point.source}
        });
    }
    return j;
}

json fusedReferenceVizLayer(
    const topology_map::algorithms::FusedReferenceResult& result) {
    json layer;
    layer["id"] = "fused_reference";
    layer["name"] = "Fused reference";
    layer["visible"] = true;
    layer["items"] = json::array();
    if (!result.ok || result.points.size() < 2) {
        return layer;
    }

    auto append_segment = [&](std::size_t begin, std::size_t end, const std::string& segment_source) {
        if (end <= begin || end - begin < 2) return;
        json points = json::array();
        for (std::size_t i = begin; i < end; ++i) {
            const auto& point = result.points[i];
            points.push_back({point.x, point.y, 0.0});
        }

        const bool is_visual = segment_source == "visual";
        json item;
        item["type"] = "polyline";
        item["id"] = "fused_reference_" + segment_source + "_" + std::to_string(begin);
        item["name"] = is_visual ? "Fused reference visual anchor"
                                  : "Fused reference navigation trend extension";
        item["points"] = std::move(points);
        item["style"] = {
            {"color", is_visual ? "#ff5c8a" : "#39c5ff"},
            {"width", is_visual ? 0.22 : 0.18},
            {"dash", !is_visual}
        };
        item["properties"] = {
            {"source", "fused_reference_module"},
            {"segment_source", segment_source},
            {"method", result.method},
            {"used_navigation", result.used_navigation},
            {"lateral_offset_m", result.lateral_offset_m},
            {"heading_error_rad", result.heading_error_rad},
            {"overlap_length_m", result.overlap_length_m},
            {"confidence", result.confidence}
        };
        layer["items"].push_back(std::move(item));
    };

    std::size_t segment_begin = 0;
    std::string segment_source = result.points.front().source;
    for (std::size_t i = 1; i < result.points.size(); ++i) {
        const std::string& source = result.points[i].source;
        if (source == segment_source) continue;
        append_segment(segment_begin, i, segment_source);
        segment_begin = i - 1;
        segment_source = source;
    }
    append_segment(segment_begin, result.points.size(), segment_source);
    return layer;
}

json boundaryIntersectionsToJson(
    const topology_map::algorithms::BoundarySamplingResult& result) {
    json j;
    j["schema_version"] = "topology-map.boundary-intersections.v1";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["frame_id"] = result.frame_id;
    j["source_section_count"] = result.source_section_count;
    j["slice_count"] = result.slices.size();
    j["hit_count"] = result.hit_count;
    j["slices"] = json::array();
    for (const auto& slice : result.slices) {
        json item;
        item["s"] = slice.s;
        item["origin"] = {{"x", slice.origin_x}, {"y", slice.origin_y}};
        item["normal"] = {{"x", slice.normal_x}, {"y", slice.normal_y}};
        item["hits"] = json::array();
        for (const auto& hit : slice.hits) {
            item["hits"].push_back({
                {"id", hit.id},
                {"s", hit.s},
                {"x", hit.x},
                {"y", hit.y},
                {"offset_m", hit.offset_m},
                {"source", hit.source},
                {"source_line_id", hit.source_line_id},
                {"track_line_id", hit.track_line_id},
                {"lane_id", hit.lane_id},
                {"lane_index", hit.lane_index},
                {"section_index", hit.section_index},
                {"source_type", hit.source_type},
                {"lane_type", hit.lane_type},
                {"lane_type_value", hit.lane_type_value},
                {"lane_position", hit.lane_position},
                {"confidence", hit.confidence}
            });
        }
        j["slices"].push_back(std::move(item));
    }
    return j;
}

json boundaryIntersectionsVizLayer(
    const topology_map::algorithms::BoundarySamplingResult& result) {
    json layer;
    layer["id"] = "raw_intersection";
    layer["name"] = "Raw intersection";
    layer["visible"] = true;
    layer["items"] = json::array();
    if (!result.ok) return layer;

    for (std::size_t i = 0; i < result.slices.size(); ++i) {
        const auto& slice = result.slices[i];
        if (!slice.hits.empty()) {
            json normal;
            normal["type"] = "polyline";
            normal["id"] = "boundary_slice_normal_" + std::to_string(i);
            normal["name"] = normal["id"];
            normal["points"] = {
                {slice.origin_x - slice.normal_x * 12.0, slice.origin_y - slice.normal_y * 12.0, 0.0},
                {slice.origin_x + slice.normal_x * 12.0, slice.origin_y + slice.normal_y * 12.0, 0.0}
            };
            normal["style"] = {
                {"color", "#5f6b7a"},
                {"width", 0.04},
                {"dash", true}
            };
            normal["properties"] = {
                {"source", "raw_intersection_module"},
                {"item_role", "slice_normal"},
                {"s", slice.s},
                {"hit_count", slice.hits.size()}
            };
            layer["items"].push_back(std::move(normal));
        }
        for (const auto& hit : slice.hits) {
            const double r = 0.25;
            json marker;
            marker["type"] = "polyline";
            marker["id"] = hit.id;
            marker["name"] = hit.source_line_id;
            marker["points"] = {
                {hit.x - r, hit.y, 0.0},
                {hit.x + r, hit.y, 0.0},
                {hit.x, hit.y, 0.0},
                {hit.x, hit.y - r, 0.0},
                {hit.x, hit.y + r, 0.0}
            };
            marker["style"] = {
                {"color", hit.source_type == "curb" ? "#eb5757" :
                          (hit.source_type == "road_edge" ? "#f2c94c" : "#2dff9a")},
                {"width", 0.08},
                {"dash", false}
            };
            marker["properties"] = {
                {"source", "raw_intersection_module"},
                {"item_role", "raw_intersection"},
                {"s", hit.s},
                {"offset_m", hit.offset_m},
                {"source_line_id", hit.source_line_id},
                {"track_line_id", hit.track_line_id},
                {"boundary_source", hit.source},
                {"lane_id", hit.lane_id},
                {"lane_index", hit.lane_index},
                {"section_index", hit.section_index},
                {"source_type", hit.source_type},
                {"lane_type", hit.lane_type},
                {"lane_type_value", hit.lane_type_value},
                {"lane_position", hit.lane_position}
            };
            layer["items"].push_back(std::move(marker));
        }
    }
    return layer;
}

json frenetTopologyDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result) {
    json j;
    j["schema_version"] = "topology-map.frenet-topology-debug.v1";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["frame_id"] = result.frame_id;
    j["source"] = "boundary_intersections";
    j["tracks"] = json::array();
    j["ribbon_tracks"] = json::array();
    j["track_ribbon_index"] = json::object();
    j["slices"] = json::array();
    j["stable_runs"] = json::array();
    if (!result.ok) return j;

    const topology_map::algorithms::FrenetTrackBuilder track_builder;
    const auto track_result = track_builder.build(result);
    const topology_map::algorithms::FrenetRibbonBuilder ribbon_builder;
    const auto ribbon_result = ribbon_builder.build(result, track_result);
    for (const auto& track : track_result.tracks) {
        json samples = json::array();
        for (const auto& sample : track.samples) {
            samples.push_back({
                {"s", sample.s},
                {"l", sample.l},
                {"x", sample.x},
                {"y", sample.y},
                {"slice_index", sample.slice_index},
                {"section_index", sample.section_index},
                {"source_line_id", sample.source_line_id},
                {"lane_type", sample.lane_type},
                {"lane_type_value", sample.lane_type_value},
                {"confidence", sample.confidence}
            });
        }
        json item;
        item["id"] = track.id;
        item["label"] = track.label;
        item["source_type"] = track.source_type;
        item["lane_type_summary"] = track.lane_type_summary;
        item["lane_position"] = track.lane_position;
        item["lane_id"] = track.lane_id;
        item["lane_index"] = track.lane_index;
        item["sample_count"] = track.samples.size();
        item["s_range_m"] = {track.s_start, track.s_end};
        item["l_range_m"] = {track.l_min, track.l_max};
        item["mean_l_m"] = track.l_mean;
        item["gap_count"] = track.gap_count;
        item["support_length_m"] = track.support_length_m;
        item["samples"] = std::move(samples);
        j["tracks"].push_back(std::move(item));
    }

    std::string active_key;
    double active_start_s = 0.0;
    double active_end_s = 0.0;
    int active_slice_count = 0;
    json active_width_classes = json::array();
    auto close_run = [&]() {
        if (active_slice_count <= 0) return;
        j["stable_runs"].push_back({
            {"s_range_m", {active_start_s, active_end_s}},
            {"slice_count", active_slice_count},
            {"ribbon_sequence", active_width_classes},
            {"sequence_key", active_key}
        });
    };

    for (std::size_t slice_index = 0; slice_index < result.slices.size(); ++slice_index) {
        const auto& slice = result.slices[slice_index];
        json ribbons = json::array();
        json width_classes = json::array();
        int normal_lane_count = 0;
        int shoulder_count = 0;
        if (slice_index < ribbon_result.ribbons_by_slice.size()) {
            std::vector<topology_map::algorithms::RibbonSampleRef> refs;
            for (const auto& ref : ribbon_result.ribbons_by_slice[slice_index]) {
                if (!ref.self_is_right_boundary) continue;
                refs.push_back(ref);
            }
            std::sort(refs.begin(), refs.end(), [](const auto& a, const auto& b) {
                return a.ribbon_sample_index < b.ribbon_sample_index;
            });
            for (const auto& ref : refs) {
                const auto& ribbon = ribbon_result.ribbons[static_cast<std::size_t>(ref.ribbon_index)];
                const auto& sample = ribbon.samples[static_cast<std::size_t>(ref.ribbon_sample_index)];
                const double width = sample.width_m;
                const double center_l = sample.center_l_m;
                const std::string width_class = ribbonWidthClass(width);
                const std::string kind = topology_map::algorithms::frenetRibbonKindToString(ribbon.kind);
                if (width_class == "lane") ++normal_lane_count;
                if (width_class == "shoulder") ++shoulder_count;
                width_classes.push_back(width_class);
                ribbons.push_back({
                    {"index", ribbons.size()},
                    {"ribbon_id", ribbon.id},
                    {"ribbon_label", ribbon.label},
                    {"kind", kind},
                    {"left_track_id", sample.right_track_id},
                    {"right_track_id", sample.left_track_id},
                    {"left_l_m", sample.right_l},
                    {"right_l_m", sample.left_l},
                    {"center_l_m", center_l},
                    {"width_m", width},
                    {"width_class", width_class},
                    {"left_source_type", ribbon.right_source_type},
                    {"right_source_type", ribbon.left_source_type}
                });
            }
        }
        const std::string key = ribbonSequenceKey(ribbons);
        if (active_slice_count == 0) {
            active_key = key;
            active_start_s = slice.s;
            active_end_s = slice.s;
            active_slice_count = 1;
            active_width_classes = width_classes;
        } else if (key == active_key) {
            active_end_s = slice.s;
            ++active_slice_count;
        } else {
            close_run();
            active_key = key;
            active_start_s = slice.s;
            active_end_s = slice.s;
            active_slice_count = 1;
            active_width_classes = width_classes;
        }

        j["slices"].push_back({
            {"s", slice.s},
            {"origin", {{"x", slice.origin_x}, {"y", slice.origin_y}}},
            {"hit_count", slice.hits.size()},
            {"ribbon_count", ribbons.size()},
            {"normal_lane_count", normal_lane_count},
            {"shoulder_count", shoulder_count},
            {"ribbon_sequence", width_classes},
            {"ribbons", std::move(ribbons)}
        });
    }
    close_run();

    for (const auto& ribbon : ribbon_result.ribbons) {
        const auto stats = ribbon_result.computeStats(ribbon.id);
        std::vector<double> centers;
        centers.reserve(ribbon.samples.size());
        json samples = json::array();
        for (const auto& sample : ribbon.samples) {
            centers.push_back(sample.center_l_m);
            samples.push_back({
                {"s", sample.s},
                {"left_l_m", sample.right_l},
                {"right_l_m", sample.left_l},
                {"center_l_m", sample.center_l_m},
                {"width_m", sample.width_m},
                {"width_class", ribbonWidthClass(sample.width_m)}
            });
        }
        const double center_mean = centers.empty()
            ? 0.0
            : std::accumulate(centers.begin(), centers.end(), 0.0) / static_cast<double>(centers.size());
        const double center_median = medianValue(centers);
        const double center_std = standardDeviation(centers, center_mean);
        const double min_s = ribbon.samples.empty() ? 0.0 : ribbon.samples.front().s;
        const double max_s = ribbon.samples.empty() ? 0.0 : ribbon.samples.back().s;
        const std::string width_class = ribbonWidthClass(stats.width_median);
        std::string ribbon_class = "unstable";
        if (max_s - min_s >= 10.0 && stats.width_std <= 0.45 && width_class == "lane") {
            ribbon_class = "stable_lane";
        } else if (max_s - min_s >= 10.0 && stats.width_std <= 0.35 && width_class == "shoulder") {
            ribbon_class = "stable_shoulder";
        } else if (max_s - min_s >= 6.0 && width_class == "wide") {
            ribbon_class = "wide_transition";
        } else if (max_s - min_s < 6.0 || ribbon.samples.size() < 3) {
            ribbon_class = "short";
        }

        j["ribbon_tracks"].push_back({
            {"id", ribbon.id},
            {"pair_id", ribbon.pair_id},
            {"segment_index", ribbon.segment_index},
            {"label", ribbon.label},
            {"kind", topology_map::algorithms::frenetRibbonKindToString(ribbon.kind)},
            {"left_track_id", ribbon.right_track_id},
            {"right_track_id", ribbon.left_track_id},
            {"left_source_type", ribbon.right_source_type},
            {"right_source_type", ribbon.left_source_type},
            {"sample_count", ribbon.samples.size()},
            {"s_range_m", {min_s, max_s}},
            {"support_length_m", std::max(0.0, max_s - min_s)},
            {"width_mean_m", stats.width_mean},
            {"width_trimmed_mean_m", stats.width_trimmed_mean},
            {"width_median_m", stats.width_median},
            {"width_std_m", stats.width_std},
            {"center_l_mean_m", center_mean},
            {"center_l_median_m", center_median},
            {"center_l_std_m", center_std},
            {"width_class", width_class},
            {"ribbon_class", ribbon_class},
            {"samples", std::move(samples)}
        });
    }

    for (const auto& [track_id, refs] : ribbon_result.ribbons_by_track_id) {
        json left_side = json::array();
        json right_side = json::array();
        auto append_ref = [&](json& out, const topology_map::algorithms::RibbonSampleRef& ref) {
            out.push_back({
                {"ribbon_id", ref.ribbon_id},
                {"slice_index", ref.slice_index},
                {"s", ref.s},
                {"neighbor_track_id", ref.neighbor_track_id},
                {"self_is_right_boundary", ref.self_is_right_boundary},
                {"self_is_left_boundary", ref.self_is_left_boundary}
            });
        };
        for (const auto& ref : refs.left_side) append_ref(left_side, ref);
        for (const auto& ref : refs.right_side) append_ref(right_side, ref);
        j["track_ribbon_index"][track_id] = {
            {"left_side", std::move(left_side)},
            {"right_side", std::move(right_side)}
        };
    }

    j["diagnostics"] = {
        {"track_count", j["tracks"].size()},
        {"ribbon_track_count", j["ribbon_tracks"].size()},
        {"slice_count", j["slices"].size()},
        {"stable_run_count", j["stable_runs"].size()}
    };
    return j;
}

json laneCenterDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result) {
    json j;
    j["schema_version"] = "topology-map.lane-center-debug.v1";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["frame_id"] = result.frame_id;
    j["source"] = "boundary_intersections";
    j["lane_center_tracks"] = json::array();
    j["lane_tracks"] = json::array();
    j["slices"] = json::array();
    if (!result.ok) return j;

    struct LaneCenterAccum {
        std::string left_track_id;
        std::string right_track_id;
        std::string left_source_type;
        std::string right_source_type;
        std::string pair_key;
        int segment_index = 0;
        double last_s = std::numeric_limits<double>::quiet_NaN();
        std::vector<double> s_values;
        std::vector<double> widths;
        std::vector<double> centers;
        json samples = json::array();
    };

    std::map<std::string, std::vector<LaneCenterAccum>> tracks;
    for (const auto& slice : result.slices) {
        if (slice.hits.size() < 2) continue;
        json slice_centers = json::array();
        for (std::size_t i = 1; i < slice.hits.size(); ++i) {
            const auto& left = slice.hits[i - 1];
            const auto& right = slice.hits[i];
            const double width = right.offset_m - left.offset_m;
            const std::string width_class = ribbonWidthClass(width);
            if (width_class != "lane") continue;

            const double center_l = 0.5 * (left.offset_m + right.offset_m);
            const std::string pair_key = left.source_line_id + " -> " + right.source_line_id;
            auto& segments = tracks[pair_key];
            if (segments.empty() || slice.s - segments.back().last_s > 3.2) {
                LaneCenterAccum next;
                next.pair_key = pair_key;
                next.segment_index = static_cast<int>(segments.size());
                segments.push_back(std::move(next));
            }
            auto& track = segments.back();
            track.left_track_id = left.source_line_id;
            track.right_track_id = right.source_line_id;
            track.left_source_type = left.source_type;
            track.right_source_type = right.source_type;
            track.last_s = slice.s;
            track.s_values.push_back(slice.s);
            track.widths.push_back(width);
            track.centers.push_back(center_l);
            track.samples.push_back({
                {"s", slice.s},
                {"center_l_m", center_l},
                {"width_m", width},
                {"left_l_m", left.offset_m},
                {"right_l_m", right.offset_m},
                {"left_track_id", left.source_line_id},
                {"right_track_id", right.source_line_id},
                {"left_source_type", left.source_type},
                {"right_source_type", right.source_type}
            });
            slice_centers.push_back({
                {"index", slice_centers.size()},
                {"center_l_m", center_l},
                {"width_m", width},
                {"left_track_id", left.source_line_id},
                {"right_track_id", right.source_line_id}
            });
        }
        if (!slice_centers.empty()) {
            j["slices"].push_back({
                {"s", slice.s},
                {"lane_center_count", slice_centers.size()},
                {"centers", std::move(slice_centers)}
            });
        }
    }

    int label_index = 0;
    for (auto& [pair_key, segments] : tracks) {
        for (auto& track : segments) {
            const std::size_t n = track.widths.size();
            if (n == 0) continue;
            const double min_s = *std::min_element(track.s_values.begin(), track.s_values.end());
            const double max_s = *std::max_element(track.s_values.begin(), track.s_values.end());
            const double support_length = std::max(0.0, max_s - min_s);
            const double width_sum = std::accumulate(track.widths.begin(), track.widths.end(), 0.0);
            const double center_sum = std::accumulate(track.centers.begin(), track.centers.end(), 0.0);
            const double width_mean = width_sum / static_cast<double>(n);
            const double center_mean = center_sum / static_cast<double>(n);
            const double width_median = medianValue(track.widths);
            const double center_median = medianValue(track.centers);
            const double width_std = standardDeviation(track.widths, width_mean);
            const double center_std = standardDeviation(track.centers, center_mean);
            std::string quality = "unstable";
            if (support_length >= 10.0 && width_std <= 0.45) {
                quality = "stable_lane";
            } else if (support_length >= 6.0 && width_std <= 0.65) {
                quality = "candidate_lane";
            } else if (support_length < 6.0 || n < 3) {
                quality = "short";
            }
            double confidence = 0.25;
            if (quality == "stable_lane") {
                confidence = std::clamp(0.65 + 0.02 * support_length - width_std * 0.35, 0.65, 0.98);
            } else if (quality == "candidate_lane") {
                confidence = std::clamp(0.45 + 0.015 * support_length - width_std * 0.2, 0.35, 0.75);
            }

            j["lane_center_tracks"].push_back({
                {"id", pair_key + "#" + std::to_string(track.segment_index)},
                {"label", "LC" + std::to_string(label_index++)},
                {"pair_id", pair_key},
                {"segment_index", track.segment_index},
                {"left_track_id", track.left_track_id},
                {"right_track_id", track.right_track_id},
                {"left_source_type", track.left_source_type},
                {"right_source_type", track.right_source_type},
                {"sample_count", n},
                {"s_range_m", {min_s, max_s}},
                {"support_length_m", support_length},
                {"width_mean_m", width_mean},
                {"width_median_m", width_median},
                {"width_std_m", width_std},
                {"center_l_mean_m", center_mean},
                {"center_l_median_m", center_median},
                {"center_l_std_m", center_std},
                {"quality", quality},
                {"confidence", confidence},
                {"samples", std::move(track.samples)}
            });
        }
    }

    struct LaneTrackAccum {
        std::vector<int> lane_center_indices;
        json samples = json::array();
    };
    auto sStart = [](const json& track) {
        const auto& range = track.at("s_range_m");
        return range.at(0).get<double>();
    };
    auto sEnd = [](const json& track) {
        const auto& range = track.at("s_range_m");
        return range.at(1).get<double>();
    };
    auto centerMedian = [](const json& track) {
        return track.value("center_l_median_m", 0.0);
    };
    auto firstCenter = [](const json& track) {
        const auto samples = track.value("samples", json::array());
        if (!samples.is_array() || samples.empty()) return track.value("center_l_median_m", 0.0);
        return samples.front().value("center_l_m", track.value("center_l_median_m", 0.0));
    };
    auto lastCenter = [](const json& track) {
        const auto samples = track.value("samples", json::array());
        if (!samples.is_array() || samples.empty()) return track.value("center_l_median_m", 0.0);
        return samples.back().value("center_l_m", track.value("center_l_median_m", 0.0));
    };
    auto isUsableLaneCenter = [](const json& track) {
        const std::string quality = track.value("quality", "");
        return quality == "stable_lane" || quality == "candidate_lane" || quality == "short";
    };
    auto hasConflictInGap = [&](int from_idx, int to_idx) {
        const auto& from = j["lane_center_tracks"].at(from_idx);
        const auto& to = j["lane_center_tracks"].at(to_idx);
        const double gap_start = sEnd(from);
        const double gap_end = sStart(to);
        if (gap_end - gap_start <= 3.2) return false;
        const double from_l = lastCenter(from);
        const double to_l = firstCenter(to);
        for (int i = 0; i < static_cast<int>(j["lane_center_tracks"].size()); ++i) {
            if (i == from_idx || i == to_idx) continue;
            const auto& other = j["lane_center_tracks"].at(i);
            if (!isUsableLaneCenter(other)) continue;
            const double other_start = sStart(other);
            const double other_end = sEnd(other);
            if (other_end <= gap_start || other_start >= gap_end) continue;
            const double anchor_s = std::clamp(0.5 * (other_start + other_end), gap_start, gap_end);
            const double ratio = (anchor_s - gap_start) / std::max(1e-6, gap_end - gap_start);
            const double expected_l = from_l + std::clamp(ratio, 0.0, 1.0) * (to_l - from_l);
            if (std::abs(centerMedian(other) - expected_l) < 1.2) {
                return true;
            }
        }
        return false;
    };
    auto associationScore = [&](int from_idx, int to_idx, json* reasons) {
        const auto& from = j["lane_center_tracks"].at(from_idx);
        const auto& to = j["lane_center_tracks"].at(to_idx);
        const double gap = sStart(to) - sEnd(from);
        if (gap < -1e-3 || gap > 30.0) return -1e9;
        const double center_delta = std::abs(firstCenter(to) - lastCenter(from));
        const double max_center_delta = 0.8 + 0.015 * std::max(0.0, gap);
        if (center_delta > max_center_delta) return -1e9;

        double score = 0.0;
        json local_reasons = json::array();
        if (from.value("pair_id", "") == to.value("pair_id", "")) {
            score += 80.0;
            local_reasons.push_back("same_boundary_pair");
        }
        if (from.value("left_track_id", "") == to.value("left_track_id", "")) {
            score += 45.0;
            local_reasons.push_back("same_left_boundary");
        }
        if (from.value("right_track_id", "") == to.value("right_track_id", "")) {
            score += 45.0;
            local_reasons.push_back("same_right_boundary");
        }
        if (score < 40.0) return -1e9;
        if (hasConflictInGap(from_idx, to_idx)) return -1e9;
        local_reasons.push_back("center_l_continuity");
        if (gap > 3.2) local_reasons.push_back("gap_bridge_without_conflict");
        score += std::max(0.0, 30.0 - gap);
        score += std::max(0.0, 10.0 - 10.0 * center_delta);
        if (reasons) *reasons = std::move(local_reasons);
        return score;
    };

    std::vector<int> order;
    order.reserve(j["lane_center_tracks"].size());
    for (int i = 0; i < static_cast<int>(j["lane_center_tracks"].size()); ++i) {
        if (isUsableLaneCenter(j["lane_center_tracks"].at(i))) {
            order.push_back(i);
        }
    }
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        const double ds = sStart(j["lane_center_tracks"].at(a)) -
                          sStart(j["lane_center_tracks"].at(b));
        if (std::abs(ds) > 1e-6) return ds < 0.0;
        return centerMedian(j["lane_center_tracks"].at(a)) <
               centerMedian(j["lane_center_tracks"].at(b));
    });

    std::vector<LaneTrackAccum> lane_tracks;
    std::vector<json> lane_track_reasons;
    for (const int candidate_idx : order) {
        int best_track = -1;
        double best_score = -1e9;
        json best_reasons = json::array();
        for (int track_idx = 0; track_idx < static_cast<int>(lane_tracks.size()); ++track_idx) {
            const int last_idx = lane_tracks[track_idx].lane_center_indices.back();
            json reasons = json::array();
            const double score = associationScore(last_idx, candidate_idx, &reasons);
            if (score > best_score) {
                best_score = score;
                best_track = track_idx;
                best_reasons = std::move(reasons);
            }
        }
        if (best_track < 0 || best_score < 0.0) {
            LaneTrackAccum next;
            next.lane_center_indices.push_back(candidate_idx);
            lane_tracks.push_back(std::move(next));
            lane_track_reasons.push_back(json::array());
        } else {
            lane_tracks[best_track].lane_center_indices.push_back(candidate_idx);
            lane_track_reasons[best_track].push_back({
                {"from", j["lane_center_tracks"].at(lane_tracks[best_track].lane_center_indices[
                              lane_tracks[best_track].lane_center_indices.size() - 2]).value("label", "")},
                {"to", j["lane_center_tracks"].at(candidate_idx).value("label", "")},
                {"reasons", std::move(best_reasons)}
            });
        }
    }

    for (int track_idx = 0; track_idx < static_cast<int>(lane_tracks.size()); ++track_idx) {
        const auto& track = lane_tracks[track_idx];
        if (track.lane_center_indices.empty()) continue;
        json samples = json::array();
        json segments = json::array();
        json gaps = json::array();
        double confidence_sum = 0.0;
        double support_sum = 0.0;
        std::vector<double> centers;
        centers.reserve(track.lane_center_indices.size());
        for (std::size_t k = 0; k < track.lane_center_indices.size(); ++k) {
            const auto& lc = j["lane_center_tracks"].at(track.lane_center_indices[k]);
            if (k > 0) {
                const auto& prev = j["lane_center_tracks"].at(track.lane_center_indices[k - 1]);
                const double gap_start = sEnd(prev);
                const double gap_end = sStart(lc);
                if (gap_end - gap_start > 3.2) {
                    gaps.push_back({{"s_range_m", {gap_start, gap_end}},
                                    {"length_m", gap_end - gap_start}});
                }
            }
            segments.push_back({
                {"lane_center_id", lc.value("id", "")},
                {"lane_center_label", lc.value("label", "")},
                {"quality", lc.value("quality", "")},
                {"s_range_m", lc.value("s_range_m", json::array())}
            });
            confidence_sum += lc.value("confidence", 0.0);
            support_sum += lc.value("support_length_m", 0.0);
            centers.push_back(centerMedian(lc));
            for (const auto& sample : lc.value("samples", json::array())) {
                samples.push_back(sample);
            }
        }
        std::sort(samples.begin(), samples.end(), [](const json& a, const json& b) {
            return a.value("s", 0.0) < b.value("s", 0.0);
        });
        const auto& first = j["lane_center_tracks"].at(track.lane_center_indices.front());
        const auto& last = j["lane_center_tracks"].at(track.lane_center_indices.back());
        const double center_mean =
            centers.empty() ? 0.0 : std::accumulate(centers.begin(), centers.end(), 0.0) /
                                      static_cast<double>(centers.size());
        const double confidence =
            track.lane_center_indices.empty()
                ? 0.0
                : confidence_sum / static_cast<double>(track.lane_center_indices.size());
        std::string quality = "single_observation";
        if (track.lane_center_indices.size() >= 2 && support_sum >= 20.0) {
            quality = "tracked_lane";
        } else if (track.lane_center_indices.size() >= 2) {
            quality = "weak_tracked_lane";
        }
        j["lane_tracks"].push_back({
            {"id", "LT" + std::to_string(track_idx)},
            {"label", "LT" + std::to_string(track_idx)},
            {"quality", quality},
            {"lane_center_labels", [&]() {
                 json labels = json::array();
                 for (int idx : track.lane_center_indices) {
                     labels.push_back(j["lane_center_tracks"].at(idx).value("label", ""));
                 }
                 return labels;
             }()},
            {"segment_count", track.lane_center_indices.size()},
            {"gap_count", gaps.size()},
            {"s_range_m", {sStart(first), sEnd(last)}},
            {"support_length_m", support_sum},
            {"center_l_mean_m", center_mean},
            {"confidence", confidence},
            {"segments", std::move(segments)},
            {"gaps", std::move(gaps)},
            {"associations", lane_track_reasons[track_idx]},
            {"samples", std::move(samples)}
        });
    }

    j["diagnostics"] = {
        {"lane_center_track_count", j["lane_center_tracks"].size()},
        {"lane_track_count", j["lane_tracks"].size()},
        {"slice_count", j["slices"].size()}
    };
    return j;
}

json boundaryCompletionDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result) {
    json j;
    j["schema_version"] = "topology-map.boundary-completion-debug.v2";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["frame_id"] = result.frame_id;
    j["source"] = "frenet_tracks_topology_ribbons";
    j["observed_nodes"] = json::array();
    j["pair_relations"] = json::array();
    j["ratio_relations"] = json::array();
    j["junction_candidates"] = json::array();
    j["forward_inferred_nodes"] = json::array();
    j["backward_inferred_nodes"] = json::array();
    j["merged_inferred_nodes"] = json::array();
    j["inferred_nodes"] = json::array();
    j["stop_nodes"] = json::array();
    j["links"] = json::array();
    j["completion_tracks"] = json::array();
    if (!result.ok) return j;

    const topology_map::algorithms::FrenetTrackBuilder track_builder;
    const auto track_result = track_builder.build(result);
    const topology_map::algorithms::FrenetRibbonBuilder ribbon_builder;
    const auto ribbon_result = ribbon_builder.build(result, track_result);
    const topology_map::algorithms::FrenetJunctionAnalyzer junction_analyzer;
    const auto junction_result = junction_analyzer.analyze(result, ribbon_result);
    const topology_map::algorithms::FrenetLaneLineCompleter completer;
    const auto completion_result = completer.complete(result, track_result, ribbon_result);

    std::map<std::string, std::map<int, CompletionObservation>> by_line_slice;
    for (std::size_t slice_index = 0; slice_index < result.slices.size(); ++slice_index) {
        const auto& slice = result.slices[slice_index];
        for (const auto& hit : slice.hits) {
            CompletionObservation obs;
            obs.slice_index = static_cast<int>(slice_index);
            obs.s = slice.s;
            obs.l = hit.offset_m;
            obs.x = hit.x;
            obs.y = hit.y;
            obs.normal_x = slice.normal_x;
            obs.normal_y = slice.normal_y;
            obs.line_id = hit.source_line_id;
            obs.source_type = hit.source_type;
            obs.confidence = hit.confidence;
            const std::string obs_track_id = hit.track_line_id.empty() ? hit.source_line_id : hit.track_line_id;
            by_line_slice[obs_track_id][obs.slice_index] = obs;
            j["observed_nodes"].push_back({
                {"s", obs.s},
                {"l", obs.l},
                {"x", obs.x},
                {"y", obs.y},
                {"slice_index", obs.slice_index},
                {"line_id", obs.line_id},
                {"track_line_id", hit.track_line_id},
                {"source_type", obs.source_type},
                {"lane_type", hit.lane_type},
                {"lane_type_value", hit.lane_type_value},
                {"confidence", obs.confidence}
            });
        }
    }

    for (const auto& track : track_result.tracks) {
        if (!topology_map::algorithms::isLaneBoundaryType(track.type)) continue;
        j["completion_tracks"].push_back({
            {"id", track.id},
            {"label", track.label},
            {"source_type", track.source_type},
            {"sample_count", track.samples.size()},
            {"support_length_m", track.support_length_m},
            {"s_range_m", {track.s_start, track.s_end}},
            {"gap_count", track.gap_count}
        });
    }

    for (const auto& ribbon : ribbon_result.ribbons) {
        if (ribbon.kind != topology_map::algorithms::FrenetRibbonKind::kTopologyLane) continue;
        const auto stats = ribbon_result.computeStats(ribbon.id);
        json samples = json::array();
        for (const auto& sample : ribbon.samples) {
            samples.push_back({
                {"s", sample.s},
                {"slice_index", sample.slice_index},
                {"right_l_m", sample.right_l},
                {"left_l_m", sample.left_l},
                {"width_m", sample.width_m}
            });
        }
        j["pair_relations"].push_back({
            {"id", ribbon.id},
            {"label", ribbon.label},
            {"right_line_id", ribbon.right_track_id},
            {"left_line_id", ribbon.left_track_id},
            {"right_source_type", ribbon.right_source_type},
            {"left_source_type", ribbon.left_source_type},
            {"kind", topology_map::algorithms::frenetRibbonKindToString(ribbon.kind)},
            {"sample_count", ribbon.samples.size()},
            {"width_mean_m", stats.width_mean},
            {"width_median_m", stats.width_median},
            {"width_trimmed_mean_m", stats.width_trimmed_mean},
            {"width_std_m", stats.width_std},
            {"samples", std::move(samples)}
        });
    }

    auto inferredNodeJson = [](const topology_map::algorithms::FrenetInferredNode& node) {
        return json{
            {"id", node.id},
            {"label", node.label},
            {"s", node.s},
            {"l", node.l},
            {"x", node.x},
            {"y", node.y},
            {"slice_index", node.slice_index},
            {"line_id", node.track_id},
            {"track_id", node.track_id},
            {"source_type", node.source_type},
            {"lane_type", node.lane_type},
            {"lane_type_value", node.lane_type_value},
            {"state", "inferred_lane_line"},
            {"method", node.method},
            {"confidence", node.confidence},
            {"anchor_track_id", node.anchor_track_id},
            {"left_anchor_track_id", node.left_anchor_track_id},
            {"right_anchor_track_id", node.right_anchor_track_id},
            {"ribbon_id", node.ribbon_id},
            {"left_ribbon_id", node.left_ribbon_id},
            {"right_ribbon_id", node.right_ribbon_id},
            {"estimated_width_m", node.estimated_width_m},
            {"width_support_s", node.width_support_s},
            {"width_support_distance_m", node.width_support_distance_m},
            {"left_width_m", node.left_width_m},
            {"right_width_m", node.right_width_m},
            {"estimated_ratio", node.estimated_ratio}
        };
    };

    for (const auto& node : completion_result.inferred_nodes) {
        auto item = inferredNodeJson(node);
        j["inferred_nodes"].push_back(item);
        j["merged_inferred_nodes"].push_back(std::move(item));
    }
    for (const auto& node : completion_result.stop_nodes) {
        j["stop_nodes"].push_back(inferredNodeJson(node));
    }
    for (const auto& link : completion_result.links) {
        j["links"].push_back({
            {"id", link.id},
            {"label", link.label},
            {"track_id", link.track_id},
            {"from_kind", link.from_kind},
            {"to_kind", link.to_kind},
            {"from_node_id", link.from_node_id},
            {"to_node_id", link.to_node_id},
            {"from_s", link.from_s},
            {"from_l", link.from_l},
            {"from_x", link.from_x},
            {"from_y", link.from_y},
            {"to_s", link.to_s},
            {"to_l", link.to_l},
            {"to_x", link.to_x},
            {"to_y", link.to_y},
            {"method", link.method}
        });
    }

    for (const auto& candidate : junction_result.candidates) {
        j["junction_candidates"].push_back({
            {"id", candidate.id},
            {"label", candidate.label},
            {"s", candidate.s},
            {"l", candidate.l},
            {"x", candidate.x},
            {"y", candidate.y},
            {"distance_m", candidate.distance_m},
            {"event_hint", candidate.event_hint},
            {"right_line_id", candidate.right_line_id},
            {"left_line_id", candidate.left_line_id},
            {"right_track_id", candidate.right_track_id},
            {"left_track_id", candidate.left_track_id},
            {"prev_distance_m", candidate.has_prev_distance ? json(candidate.prev_distance_m) : json(nullptr)},
            {"next_distance_m", candidate.has_next_distance ? json(candidate.next_distance_m) : json(nullptr)},
            {"ribbon_id", candidate.ribbon_id},
            {"ribbon_label", candidate.ribbon_label},
            {"ribbon_start_s", candidate.ribbon_start_s},
            {"ribbon_end_s", candidate.ribbon_end_s},
            {"ribbon_width_start_m", candidate.ribbon_width_start_m},
            {"ribbon_width_end_m", candidate.ribbon_width_end_m},
            {"ribbon_width_slope_m_per_m", candidate.ribbon_width_slope_m_per_m},
            {"ribbon_sample_count", candidate.ribbon_sample_count}
        });
    }

    j["diagnostics"] = {
        {"observed_node_count", j["observed_nodes"].size()},
        {"pair_relation_count", j["pair_relations"].size()},
        {"ratio_relation_count", j["ratio_relations"].size()},
        {"junction_candidate_count", j["junction_candidates"].size()},
        {"forward_inferred_node_count", j["forward_inferred_nodes"].size()},
        {"backward_inferred_node_count", j["backward_inferred_nodes"].size()},
        {"merged_inferred_node_count", j["merged_inferred_nodes"].size()},
        {"inferred_node_count", j["inferred_nodes"].size()},
        {"stop_node_count", j["stop_nodes"].size()},
        {"link_count", j["links"].size()},
        {"completion_track_count", j["completion_tracks"].size()},
        {"junction_ok", junction_result.ok},
        {"junction_error", junction_result.error},
        {"completion_ok", completion_result.ok},
        {"completion_error", completion_result.error}
    };
    return j;
}

json laneRegionDebugToJson(
    const topology_map::algorithms::BoundarySamplingResult& result) {
    json j;
    j["schema_version"] = "topology-map.lane-region-debug.v1";
    j["ok"] = result.ok;
    if (!result.error.empty()) j["error"] = result.error;
    j["frame_id"] = result.frame_id;
    j["source"] = "completed_frenet_tracks";
    j["regions"] = json::array();
    if (!result.ok) return j;

    const topology_map::algorithms::FrenetTrackBuilder track_builder;
    const auto track_result = track_builder.build(result);
    const topology_map::algorithms::FrenetRibbonBuilder ribbon_builder;
    const auto ribbon_result = ribbon_builder.build(result, track_result);
    const topology_map::algorithms::FrenetLaneLineCompleter completer;
    const auto completion_result = completer.complete(result, track_result, ribbon_result);
    const topology_map::algorithms::FrenetCompletedBoundaryBuilder completed_boundary_builder;
    const auto completed_result = completed_boundary_builder.build(result, track_result, completion_result);
    const topology_map::algorithms::FrenetLaneRegionBuilder region_builder;
    const auto region_result = region_builder.build(result, completed_result);

    int candidate_count = 0;
    int inferred_region_count = 0;
    for (const auto& region : region_result.regions) {
        if (region.candidate) ++candidate_count;
        if (region.has_inferred_boundary) ++inferred_region_count;
        json samples = json::array();
        for (const auto& sample : region.samples) {
            samples.push_back({
                {"s", sample.s},
                {"slice_index", sample.slice_index},
                {"right_l_m", sample.right_l},
                {"left_l_m", sample.left_l},
                {"center_l_m", sample.center_l_m},
                {"width_m", sample.width_m},
                {"right_state", topology_map::algorithms::frenetLaneRegionBoundaryStateToString(sample.right_state)},
                {"left_state", topology_map::algorithms::frenetLaneRegionBoundaryStateToString(sample.left_state)}
            });
        }
        j["regions"].push_back({
            {"id", region.id},
            {"label", region.label},
            {"pair_id", region.pair_id},
            {"segment_index", region.segment_index},
            {"right_track_id", region.right_track_id},
            {"left_track_id", region.left_track_id},
            {"right_source_type", region.right_source_type},
            {"left_source_type", region.left_source_type},
            {"right_lane_type", region.right_lane_type},
            {"left_lane_type", region.left_lane_type},
            {"lane_line_pair", region.lane_line_pair},
            {"lane_to_boundary_pair", region.lane_to_boundary_pair},
            {"boundary_pair", region.boundary_pair},
            {"has_inferred_boundary", region.has_inferred_boundary},
            {"candidate", region.candidate},
            {"candidate_reason", region.candidate_reason},
            {"width_class", region.width_class},
            {"sample_count", region.samples.size()},
            {"s_range_m", {region.s_start, region.s_end}},
            {"support_length_m", region.support_length_m},
            {"width_mean_m", region.width_mean_m},
            {"width_median_m", region.width_median_m},
            {"width_min_m", region.width_min_m},
            {"width_max_m", region.width_max_m},
            {"width_std_m", region.width_std_m},
            {"inferred_sample_ratio", region.inferred_sample_ratio},
            {"samples", std::move(samples)}
        });
    }

    j["diagnostics"] = {
        {"track_count", track_result.tracks.size()},
        {"completion_inferred_node_count", completion_result.inferred_nodes.size()},
        {"completion_stop_node_count", completion_result.stop_nodes.size()},
        {"completed_boundary_track_count", completed_result.tracks.size()},
        {"completed_boundary_connector_count", completed_result.connectors.size()},
        {"region_count", region_result.regions.size()},
        {"candidate_region_count", candidate_count},
        {"inferred_region_count", inferred_region_count},
        {"region_ok", region_result.ok},
        {"region_error", region_result.error}
    };
    return j;
}

json visualReferenceToJson(
    const topology_map::algorithms::VisualReferenceResult& result) {
    json j;
    j["schema_version"] = "topology-map.visual-reference.v1";
    j["ok"] = result.ok;
    j["frame_id"] = result.frame_id;
    if (!result.error.empty()) j["error"] = result.error;
    j["selected_source"] = result.selected_source;
    j["method"] = result.method;
    j["left_line_id"] = result.left_line_id;
    j["right_line_id"] = result.right_line_id;
    j["confidence"] = result.confidence;
    j["s_range_m"] = {result.s_range_m.first, result.s_range_m.second};
    j["center_coeffs"] = json::array();
    for (double coeff : result.center_coeffs) {
        j["center_coeffs"].push_back(coeff);
    }
    j["points"] = json::array();
    for (const auto& point : result.points) {
        j["points"].push_back({{"s", point.s}, {"x", point.x}, {"y", point.y}});
    }
    j["debug"] = {
        {"input_line_count", result.input_line_count},
        {"selected_source_line_count", result.selected_source_line_count},
        {"left_lane_position", result.left_lane_position},
        {"right_lane_position", result.right_lane_position},
        {"left_source_type", result.left_source_type},
        {"right_source_type", result.right_source_type},
        {"left_line_x_span_m", result.left_line_x_span_m},
        {"right_line_x_span_m", result.right_line_x_span_m},
        {"left_line_length_m", result.left_line_length_m},
        {"right_line_length_m", result.right_line_length_m},
        {"point_count", result.points.size()}
    };
    return j;
}

json visualReferenceVizLayer(
    const topology_map::algorithms::VisualReferenceResult& result) {
    json layer;
    layer["id"] = "visual_reference";
    layer["name"] = "Visual reference";
    layer["visible"] = true;
    layer["items"] = json::array();
    if (!result.ok || result.points.size() < 2) {
        return layer;
    }
    json points = json::array();
    for (const auto& point : result.points) {
        points.push_back({point.x, point.y, 0.0});
    }
    json item;
    item["type"] = "polyline";
    item["id"] = "visual_reference_center";
    item["name"] = "Visual reference center";
    item["points"] = std::move(points);
    item["style"] = {
        {"color", "#00b894"},
        {"width", 0.18},
        {"dash", false}
    };
    item["properties"] = {
        {"source", "visual_reference_module"},
        {"selected_source", result.selected_source},
        {"method", result.method},
        {"left_line_id", result.left_line_id},
        {"right_line_id", result.right_line_id}
    };
    layer["items"].push_back(std::move(item));
    return layer;
}

}  // namespace offline_replay::debug
