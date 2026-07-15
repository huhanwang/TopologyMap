#include "navigation_route_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace topology_map::algorithms {
namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kMinSegmentVectorLengthM = 1e-3;

std::uint64_t fnv1aAppend(std::uint64_t hash, std::uint64_t value) {
    constexpr std::uint64_t kPrime = 1099511628211ULL;
    for (int i = 0; i < 8; ++i) {
        const auto byte = static_cast<std::uint8_t>((value >> (i * 8)) & 0xff);
        hash ^= byte;
        hash *= kPrime;
    }
    return hash;
}

std::uint64_t quantizeCoord(double value) {
    return static_cast<std::uint64_t>(std::llround(value * 10000000.0));
}

bool containsText(const std::string& text, const char* pattern) {
    return text.find(pattern) != std::string::npos;
}

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double distance2d(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace

const char* navigationEventTypeName(NavigationEventType type) {
    switch (type) {
        case NavigationEventType::kNone:
            return "none";
        case NavigationEventType::kKeepLeft:
            return "keep_left";
        case NavigationEventType::kKeepRight:
            return "keep_right";
        case NavigationEventType::kTurnLeft:
            return "turn_left";
        case NavigationEventType::kTurnRight:
            return "turn_right";
        case NavigationEventType::kRampEnterLeft:
            return "ramp_enter_left";
        case NavigationEventType::kRampEnterRight:
            return "ramp_enter_right";
        case NavigationEventType::kRampEnter:
            return "ramp_enter";
        case NavigationEventType::kRampExit:
            return "ramp_exit";
        case NavigationEventType::kMergeToMainRoad:
            return "merge_to_main_road";
        case NavigationEventType::kContinueMainRoad:
            return "continue_main_road";
        case NavigationEventType::kArriveDestination:
            return "arrive_destination";
    }
    return "unknown";
}

NavigationRouteTracker::NavigationRouteTracker()
    : cfg_(Config{}) {}

NavigationRouteTracker::NavigationRouteTracker(Config config)
    : cfg_(std::move(config)) {}

bool NavigationRouteTracker::updateRoute(const snoah::SDRouteProto& route) {
    const auto signature = computeRouteSignature(route);
    if (route_.valid && route_.route_signature == signature) {
        route_.route_changed = false;
        return false;
    }

    NavigationRouteSnapshot next;
    next.valid = route.navigation_segments_size() > 0;
    next.route_changed = true;
    next.route_id = route.has_route_id() ? route.route_id() : 0;
    next.route_signature = signature;
    next.segment_count = route.navigation_segments_size();

    bool has_origin = false;
    for (int i = 0; i < route.navigation_segments_size() && !has_origin; ++i) {
        const auto& segment = route.navigation_segments(i);
        if (segment.points_size() > 0) {
            next.origin_latitude = segment.points(0).latitude();
            next.origin_longitude = segment.points(0).longitude();
            has_origin = true;
        }
    }
    if (!has_origin) {
        route_ = std::move(next);
        match_ = {};
        match_.error = "route_has_no_navigation_points";
        return true;
    }

    route_ = std::move(next);
    route_.segments.reserve(route.navigation_segments_size());
    for (int i = 0; i < route.navigation_segments_size(); ++i) {
        route_.segments.push_back(buildSegment(i, route.navigation_segments(i)));
    }
    route_.segment_count = static_cast<int>(route_.segments.size());
    rebuildGlobalRoutePoints();
    match_ = {};
    return true;
}

NavigationMatchResult NavigationRouteTracker::updateGnss(
    const snoah::GnssRawReadingProto& gnss) {
    if (!route_.valid || route_.segments.empty()) {
        match_ = {};
        match_.error = "route_not_ready";
        return match_;
    }

    double x_m = 0.0;
    double y_m = 0.0;
    if (!gnssToRouteXY(gnss, &x_m, &y_m)) {
        match_ = {};
        match_.error = "invalid_gnss";
        return match_;
    }

    const auto candidate = findBestCandidate(x_m, y_m);
    fillMatchFromCandidate(candidate);
    return match_;
}

NavigationReferenceResult NavigationRouteTracker::buildReferenceAroundVisual(
    const VisualReferenceResult& visual_reference,
    const snoah::GnssRawReadingProto& gnss) const {
    NavigationReferenceResult result;
    if (!route_.valid || global_route_points_.size() < 2) {
        result.error = "route_not_ready";
        return result;
    }
    if (!visual_reference.ok || visual_reference.points.size() < 2) {
        result.error = "invalid_visual_reference";
        return result;
    }

    const auto& visual_mid = visual_reference.points[visual_reference.points.size() / 2];
    const Vec2 visual_mid_vcs{visual_mid.x, visual_mid.y};
    double mid_route_x = 0.0;
    double mid_route_y = 0.0;
    if (!vcsToRouteXY(gnss, visual_mid_vcs, &mid_route_x, &mid_route_y)) {
        result.error = "invalid_gnss";
        return result;
    }

    const auto projection = projectToGlobalRoute(mid_route_x, mid_route_y);
    if (!projection.valid) {
        result.error = "no_global_route_projection";
        return result;
    }

    const auto& visual_front = visual_reference.points.back();
    const auto& visual_back = visual_reference.points.front();
    const double visual_heading_vcs =
        std::atan2(visual_front.y - visual_back.y, visual_front.x - visual_back.x);
    const double yaw = gnss.has_yaw() ? gnss.yaw() : 0.0;
    const double route_heading_dx = std::cos(projection.heading_rad);
    const double route_heading_dy = std::sin(projection.heading_rad);
    const double route_heading_vcs_x =
        route_heading_dx * std::cos(yaw) + route_heading_dy * std::sin(yaw);
    const double route_heading_vcs_y =
        -route_heading_dx * std::sin(yaw) + route_heading_dy * std::cos(yaw);
    const double route_heading_vcs = std::atan2(route_heading_vcs_y, route_heading_vcs_x);
    result.heading_error_rad = std::abs(normalizeAngle(route_heading_vcs - visual_heading_vcs));
    result.lateral_error_m = projection.lateral_error_m;
    if (result.lateral_error_m > cfg_.navigation_reference_max_lateral_error_m) {
        result.error = "navigation_reference_lateral_mismatch";
        return result;
    }
    if (result.heading_error_rad > cfg_.navigation_reference_max_heading_error_rad) {
        result.error = "navigation_reference_heading_mismatch";
        return result;
    }

    std::vector<NavRoutePoint> window_route_points;
    NavRoutePoint anchor;
    anchor.x_m = projection.x_m;
    anchor.y_m = projection.y_m;
    anchor.s_m = projection.s_m;
    window_route_points.push_back(anchor);
    Vec2 anchor_vcs;
    if (!routeXYToVcs(gnss, projection.x_m, projection.y_m, &anchor_vcs)) {
        result.error = "invalid_gnss";
        return result;
    }

    const double anchor_heading = projection.heading_rad;
    std::size_t forward_idx = projection.segment_start_index + 1;
    double last_heading = anchor_heading;
    double last_x = projection.x_m;
    double last_y = projection.y_m;
    double last_vcs_x = anchor_vcs.x;
    while (forward_idx < global_route_points_.size()) {
        const auto& point = global_route_points_[forward_idx];
        const double search_len = point.s_m - projection.s_m;
        Vec2 point_vcs;
        if (!routeXYToVcs(gnss, point.x_m, point.y_m, &point_vcs)) {
            result.stop_reason_forward = "invalid_gnss";
            break;
        }
        if (point_vcs.x >= cfg_.navigation_reference_front_x_m) {
            const auto& prev_point = window_route_points.back();
            const double denom = std::max(1e-6, point.s_m - prev_point.s_m);
            const double ratio = clamp01(
                (cfg_.navigation_reference_front_x_m - last_vcs_x) /
                std::max(1e-6, point_vcs.x - last_vcs_x));
            NavRoutePoint clipped = point;
            clipped.x_m = prev_point.x_m + (point.x_m - prev_point.x_m) * ratio;
            clipped.y_m = prev_point.y_m + (point.y_m - prev_point.y_m) * ratio;
            clipped.s_m = prev_point.s_m + denom * ratio;
            window_route_points.push_back(clipped);
            result.forward_length_m = cfg_.navigation_reference_front_x_m;
            result.stop_reason_forward = "front_x";
            break;
        }
        if (search_len > cfg_.navigation_reference_max_search_forward_m) {
            result.stop_reason_forward = "max_forward_search";
            break;
        }
        const double heading = std::atan2(point.y_m - last_y, point.x_m - last_x);
        const double local_turn = std::abs(normalizeAngle(heading - last_heading));
        const double total_turn = std::abs(normalizeAngle(heading - anchor_heading));
        if (local_turn > cfg_.navigation_reference_max_local_turn_rad) {
            result.stop_reason_forward = "local_turn";
            break;
        }
        if (total_turn > cfg_.navigation_reference_max_total_heading_change_rad) {
            result.stop_reason_forward = "total_heading_change";
            break;
        }
        window_route_points.push_back(point);
        result.forward_length_m = std::max(result.forward_length_m, point_vcs.x);
        last_heading = heading;
        last_x = point.x_m;
        last_y = point.y_m;
        last_vcs_x = point_vcs.x;
        ++forward_idx;
    }
    if (result.stop_reason_forward.empty()) {
        result.stop_reason_forward = "route_end";
    }

    std::vector<NavRoutePoint> backward_points;
    if (projection.segment_start_index < global_route_points_.size()) {
        std::size_t backward_idx = projection.segment_start_index;
        last_heading = anchor_heading;
        last_x = projection.x_m;
        last_y = projection.y_m;
        last_vcs_x = anchor_vcs.x;
        while (true) {
            const auto& point = global_route_points_[backward_idx];
            const double search_len = projection.s_m - point.s_m;
            Vec2 point_vcs;
            if (!routeXYToVcs(gnss, point.x_m, point.y_m, &point_vcs)) {
                result.stop_reason_backward = "invalid_gnss";
                break;
            }
            if (point_vcs.x <= cfg_.navigation_reference_rear_x_m) {
                const NavRoutePoint& prev_point =
                    backward_points.empty() ? anchor : backward_points.back();
                const double denom = std::max(1e-6, prev_point.s_m - point.s_m);
                const double ratio = clamp01(
                    (last_vcs_x - cfg_.navigation_reference_rear_x_m) /
                    std::max(1e-6, last_vcs_x - point_vcs.x));
                NavRoutePoint clipped = point;
                clipped.x_m = prev_point.x_m + (point.x_m - prev_point.x_m) * ratio;
                clipped.y_m = prev_point.y_m + (point.y_m - prev_point.y_m) * ratio;
                clipped.s_m = prev_point.s_m - denom * ratio;
                backward_points.push_back(clipped);
                result.backward_length_m = std::abs(cfg_.navigation_reference_rear_x_m);
                result.stop_reason_backward = "rear_x";
                break;
            }
            if (search_len > cfg_.navigation_reference_max_search_backward_m) {
                result.stop_reason_backward = "max_backward_search";
                break;
            }
            const double heading = std::atan2(last_y - point.y_m, last_x - point.x_m);
            const double local_turn = std::abs(normalizeAngle(heading - last_heading));
            const double total_turn = std::abs(normalizeAngle(heading - anchor_heading));
            if (local_turn > cfg_.navigation_reference_max_local_turn_rad) {
                result.stop_reason_backward = "local_turn";
                break;
            }
            if (total_turn > cfg_.navigation_reference_max_total_heading_change_rad) {
                result.stop_reason_backward = "total_heading_change";
                break;
            }
            backward_points.push_back(point);
            result.backward_length_m = std::max(result.backward_length_m, std::max(0.0, -point_vcs.x));
            last_heading = heading;
            last_x = point.x_m;
            last_y = point.y_m;
            last_vcs_x = point_vcs.x;
            if (backward_idx == 0) {
                result.stop_reason_backward = "route_start";
                break;
            }
            --backward_idx;
        }
    }
    if (result.stop_reason_backward.empty()) {
        result.stop_reason_backward = "route_start";
    }

    if (result.forward_length_m < cfg_.navigation_reference_min_forward_m) {
        result.error = "navigation_reference_too_short";
        return result;
    }

    for (auto it = backward_points.rbegin(); it != backward_points.rend(); ++it) {
        Vec2 vcs;
        if (routeXYToVcs(gnss, it->x_m, it->y_m, &vcs)) {
            result.vcs_points.push_back(vcs);
        }
    }
    result.vcs_points.push_back(anchor_vcs);
    for (std::size_t i = 1; i < window_route_points.size(); ++i) {
        Vec2 vcs;
        if (routeXYToVcs(gnss, window_route_points[i].x_m, window_route_points[i].y_m, &vcs)) {
            result.vcs_points.push_back(vcs);
        }
    }

    if (result.vcs_points.size() < 2) {
        result.error = "navigation_reference_empty_after_projection";
        return result;
    }
    result.ok = true;
    return result;
}

std::uint64_t NavigationRouteTracker::computeRouteSignature(
    const snoah::SDRouteProto& route) const {
    std::uint64_t hash = 1469598103934665603ULL;
    hash = fnv1aAppend(hash, route.has_route_id() ? route.route_id() : 0);
    hash = fnv1aAppend(hash, static_cast<std::uint64_t>(route.navigation_segments_size()));
    for (int i = 0; i < route.navigation_segments_size(); ++i) {
        const auto& segment = route.navigation_segments(i);
        hash = fnv1aAppend(hash, static_cast<std::uint64_t>(segment.points_size()));
        if (segment.points_size() > 0) {
            const auto& first = segment.points(0);
            const auto& last = segment.points(segment.points_size() - 1);
            hash = fnv1aAppend(hash, quantizeCoord(first.latitude()));
            hash = fnv1aAppend(hash, quantizeCoord(first.longitude()));
            hash = fnv1aAppend(hash, quantizeCoord(last.latitude()));
            hash = fnv1aAppend(hash, quantizeCoord(last.longitude()));
        }
        for (char ch : segment.instruction()) {
            hash = fnv1aAppend(hash, static_cast<std::uint64_t>(static_cast<unsigned char>(ch)));
        }
    }
    return hash;
}

NavigationSegmentInfo NavigationRouteTracker::buildSegment(
    int index,
    const snoah::SDRouteProto_NavigationSegment& segment) const {
    NavigationSegmentInfo info;
    info.index = index;
    if (segment.has_instruction()) info.instruction = segment.instruction();
    if (segment.has_crossing_name()) info.crossing_name = segment.crossing_name();
    if (segment.has_exit_direction_info()) {
        info.exit_direction_info = segment.exit_direction_info();
    }
    if (segment.has_exit_name()) info.exit_name = segment.exit_name();

    info.points.reserve(segment.points_size());
    for (int i = 0; i < segment.points_size(); ++i) {
        auto point = geoToRoutePoint(
            segment.points(i).latitude(),
            segment.points(i).longitude(),
            segment.points(i).altitude());
        if (!info.points.empty()) {
            const auto& prev = info.points.back();
            point.s_m = prev.s_m + distance2d(prev.x_m, prev.y_m, point.x_m, point.y_m);
        }
        info.points.push_back(point);
    }
    if (!info.points.empty()) {
        info.length_m = info.points.back().s_m;
    }
    info.terminal_event = parseTerminalEvent(index, info.instruction, info.length_m);
    return info;
}

NavigationSemanticEvent NavigationRouteTracker::parseTerminalEvent(
    int segment_index,
    const std::string& instruction,
    double segment_length_m) const {
    NavigationSemanticEvent event;
    event.instruction = instruction;
    event.segment_index = segment_index;
    event.distance_to_event_m = segment_length_m;

    if (instruction.empty()) {
        return event;
    }

    if (containsText(instruction, "到达目的地")) {
        event.type = NavigationEventType::kArriveDestination;
    } else if ((containsText(instruction, "向右前方") || containsText(instruction, "靠右")) &&
               containsText(instruction, "进入匝道")) {
        event.type = NavigationEventType::kRampEnterRight;
    } else if ((containsText(instruction, "向左前方") || containsText(instruction, "靠左")) &&
               containsText(instruction, "进入匝道")) {
        event.type = NavigationEventType::kRampEnterLeft;
    } else if (containsText(instruction, "进入匝道")) {
        event.type = NavigationEventType::kRampEnter;
    } else if (containsText(instruction, "出口") || containsText(instruction, "驶出")) {
        event.type = NavigationEventType::kRampExit;
    } else if (containsText(instruction, "右转")) {
        event.type = NavigationEventType::kTurnRight;
    } else if (containsText(instruction, "左转")) {
        event.type = NavigationEventType::kTurnLeft;
    } else if (containsText(instruction, "进入主路")) {
        event.type = NavigationEventType::kMergeToMainRoad;
    } else if (containsText(instruction, "沿主路行驶")) {
        event.type = NavigationEventType::kContinueMainRoad;
    } else if (containsText(instruction, "靠右")) {
        event.type = NavigationEventType::kKeepRight;
    } else if (containsText(instruction, "靠左")) {
        event.type = NavigationEventType::kKeepLeft;
    }

    event.label = navigationEventTypeName(event.type);
    return event;
}

NavRoutePoint NavigationRouteTracker::geoToRoutePoint(
    double lat, double lon, double alt) const {
    NavRoutePoint point;
    point.latitude = lat;
    point.longitude = lon;
    point.altitude = alt;
    point.x_m = (lon - route_.origin_longitude) *
                std::cos(route_.origin_latitude) * kEarthRadiusM;
    point.y_m = (lat - route_.origin_latitude) * kEarthRadiusM;
    return point;
}

bool NavigationRouteTracker::gnssToRouteXY(
    const snoah::GnssRawReadingProto& gnss,
    double* x_m,
    double* y_m) const {
    if (!x_m || !y_m || !gnss.has_latitude() || !gnss.has_longitude()) {
        return false;
    }
    *x_m = (gnss.longitude() - route_.origin_longitude) *
           std::cos(route_.origin_latitude) * kEarthRadiusM;
    *y_m = (gnss.latitude() - route_.origin_latitude) * kEarthRadiusM;
    return true;
}

bool NavigationRouteTracker::vcsToRouteXY(
    const snoah::GnssRawReadingProto& gnss,
    const Vec2& vcs,
    double* x_m,
    double* y_m) const {
    if (!x_m || !y_m || !gnss.has_latitude() || !gnss.has_longitude()) {
        return false;
    }
    double ego_x = 0.0;
    double ego_y = 0.0;
    if (!gnssToRouteXY(gnss, &ego_x, &ego_y)) {
        return false;
    }
    const double yaw = gnss.has_yaw() ? gnss.yaw() : 0.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double east_delta = vcs.x * c - vcs.y * s;
    const double north_delta = vcs.x * s + vcs.y * c;
    *x_m = ego_x + east_delta;
    *y_m = ego_y + north_delta;
    return true;
}

bool NavigationRouteTracker::routeXYToVcs(
    const snoah::GnssRawReadingProto& gnss,
    double x_m,
    double y_m,
    Vec2* vcs) const {
    if (!vcs || !gnss.has_latitude() || !gnss.has_longitude()) {
        return false;
    }
    double ego_x = 0.0;
    double ego_y = 0.0;
    if (!gnssToRouteXY(gnss, &ego_x, &ego_y)) {
        return false;
    }
    const double dx = x_m - ego_x;
    const double dy = y_m - ego_y;
    const double yaw = gnss.has_yaw() ? gnss.yaw() : 0.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    vcs->x = dx * c + dy * s;
    vcs->y = -dx * s + dy * c;
    return true;
}

void NavigationRouteTracker::rebuildGlobalRoutePoints() {
    global_route_points_.clear();
    for (const auto& segment : route_.segments) {
        for (const auto& point : segment.points) {
            if (!global_route_points_.empty()) {
                const auto& prev = global_route_points_.back();
                if (distance2d(prev.x_m, prev.y_m, point.x_m, point.y_m) < 0.05) {
                    continue;
                }
            }
            NavRoutePoint global_point = point;
            if (!global_route_points_.empty()) {
                const auto& prev = global_route_points_.back();
                global_point.s_m =
                    prev.s_m + distance2d(prev.x_m, prev.y_m, global_point.x_m, global_point.y_m);
            } else {
                global_point.s_m = 0.0;
            }
            global_route_points_.push_back(global_point);
        }
    }
}

NavigationRouteTracker::ProjectionCandidate NavigationRouteTracker::projectToSegment(
    int segment_index,
    double x_m,
    double y_m) const {
    ProjectionCandidate best;
    if (segment_index < 0 || segment_index >= static_cast<int>(route_.segments.size())) {
        return best;
    }
    const auto& segment = route_.segments[segment_index];
    if (segment.points.size() < 2) {
        return best;
    }

    for (std::size_t i = 1; i < segment.points.size(); ++i) {
        const auto& a = segment.points[i - 1];
        const auto& b = segment.points[i];
        const double vx = b.x_m - a.x_m;
        const double vy = b.y_m - a.y_m;
        const double len2 = vx * vx + vy * vy;
        if (len2 < kMinSegmentVectorLengthM) continue;

        const double raw_t = ((x_m - a.x_m) * vx + (y_m - a.y_m) * vy) / len2;
        const double segment_len = std::sqrt(len2);
        const double raw_along = a.s_m + segment_len * raw_t;
        if (raw_along < -cfg_.max_match_backward_s_m ||
            raw_along > segment.length_m + cfg_.max_match_forward_s_m) {
            continue;
        }

        const double t = raw_t;
        const double px = a.x_m + t * vx;
        const double py = a.y_m + t * vy;
        const double lateral = distance2d(x_m, y_m, px, py);
        const double along = raw_along;
        const double distance_to_end = std::max(0.0, segment.length_m - along);
        const double score = lateral + std::max(0.0, -along) * 0.05 +
                             std::max(0.0, along - segment.length_m) * 0.2;

        if (!best.valid || score < best.score) {
            best.valid = true;
            best.segment_index = segment_index;
            best.along_s_m = along;
            best.lateral_error_m = lateral;
            best.distance_to_end_m = distance_to_end;
            best.score = score;
        }
    }
    return best;
}

NavigationRouteTracker::ProjectionCandidate NavigationRouteTracker::findBestCandidate(
    double x_m,
    double y_m) const {
    std::vector<int> candidates;
    if (match_.matched && match_.segment_index >= 0) {
        const int last = match_.segment_index;
        candidates.push_back(last);
        if (last + 1 < route_.segment_count) candidates.push_back(last + 1);
        if (last > 0) candidates.push_back(last - 1);
    }

    ProjectionCandidate best;
    for (int index : candidates) {
        const auto candidate = projectToSegment(index, x_m, y_m);
        if (candidate.valid && (!best.valid || candidate.score < best.score)) {
            best = candidate;
        }
    }
    if (best.valid && best.lateral_error_m <= cfg_.max_match_lateral_error_m) {
        return best;
    }

    const int radius = std::max(0, cfg_.fallback_search_radius_segments);
    int begin = 0;
    int end = route_.segment_count;
    if (match_.matched && match_.segment_index >= 0 && radius > 0) {
        begin = std::max(0, match_.segment_index - radius);
        end = std::min(route_.segment_count, match_.segment_index + radius + 2);
    }

    best = {};
    for (int i = begin; i < end; ++i) {
        const auto candidate = projectToSegment(i, x_m, y_m);
        if (!candidate.valid) continue;
        if (!best.valid || candidate.score < best.score) {
            best = candidate;
        }
    }
    if (!best.valid || best.lateral_error_m > cfg_.max_match_lateral_error_m) {
        ProjectionCandidate invalid;
        invalid.score = best.valid ? best.score : 0.0;
        return invalid;
    }
    return best;
}

NavigationRouteTracker::RouteProjectionCandidate NavigationRouteTracker::projectToGlobalRoute(
    double x_m,
    double y_m) const {
    RouteProjectionCandidate best;
    if (global_route_points_.size() < 2) {
        return best;
    }
    for (std::size_t i = 1; i < global_route_points_.size(); ++i) {
        const auto& a = global_route_points_[i - 1];
        const auto& b = global_route_points_[i];
        const double vx = b.x_m - a.x_m;
        const double vy = b.y_m - a.y_m;
        const double len2 = vx * vx + vy * vy;
        if (len2 < kMinSegmentVectorLengthM) continue;

        const double t = clamp01(((x_m - a.x_m) * vx + (y_m - a.y_m) * vy) / len2);
        const double px = a.x_m + t * vx;
        const double py = a.y_m + t * vy;
        const double lateral = distance2d(x_m, y_m, px, py);
        const double segment_len = std::sqrt(len2);
        const double along = a.s_m + segment_len * t;
        const double score = lateral;

        if (!best.valid || score < best.score) {
            best.valid = true;
            best.segment_start_index = i - 1;
            best.s_m = along;
            best.x_m = px;
            best.y_m = py;
            best.heading_rad = std::atan2(vy, vx);
            best.lateral_error_m = lateral;
            best.score = score;
        }
    }
    return best;
}

void NavigationRouteTracker::fillMatchFromCandidate(
    const ProjectionCandidate& candidate) {
    match_ = {};
    if (!candidate.valid) {
        match_.error = "no_navigation_segment_match";
        return;
    }

    match_.matched = true;
    match_.segment_index = candidate.segment_index;
    match_.along_s_m = candidate.along_s_m;
    match_.lateral_error_m = candidate.lateral_error_m;
    match_.distance_to_segment_end_m = candidate.distance_to_end_m;
    match_.near_segment_end =
        candidate.distance_to_end_m <= cfg_.near_segment_end_distance_m;
    match_.can_use_navigation_reference = !match_.near_segment_end;

    const auto& segment = route_.segments[candidate.segment_index];
    match_.current_terminal_event = segment.terminal_event;
    match_.current_terminal_event.distance_to_event_m = candidate.distance_to_end_m;

    if (candidate.segment_index + 1 < route_.segment_count) {
        match_.upcoming_event = route_.segments[candidate.segment_index + 1].terminal_event;
        match_.upcoming_event.distance_to_event_m = candidate.distance_to_end_m;
    } else {
        match_.upcoming_event = match_.current_terminal_event;
    }
}

}  // namespace topology_map::algorithms
