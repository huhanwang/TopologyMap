#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "visual_reference/visual_reference_module.h"

namespace topology_map::algorithms {

struct NavRoutePoint {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double s_m = 0.0;
};

enum class NavigationEventType {
    kNone,
    kKeepLeft,
    kKeepRight,
    kTurnLeft,
    kTurnRight,
    kRampEnterLeft,
    kRampEnterRight,
    kRampEnter,
    kRampExit,
    kMergeToMainRoad,
    kContinueMainRoad,
    kArriveDestination,
};

struct NavigationSemanticEvent {
    NavigationEventType type = NavigationEventType::kNone;
    std::string label;
    std::string instruction;
    int segment_index = -1;
    double distance_to_event_m = 0.0;
};

struct NavigationSegmentInfo {
    int index = -1;
    std::string instruction;
    std::string crossing_name;
    std::string exit_direction_info;
    std::string exit_name;
    double length_m = 0.0;
    std::vector<NavRoutePoint> points;
    NavigationSemanticEvent terminal_event;
};

struct NavigationMatchResult {
    bool matched = false;
    std::string error;
    int segment_index = -1;
    double along_s_m = 0.0;
    double lateral_error_m = 0.0;
    double distance_to_segment_end_m = 0.0;
    bool near_segment_end = false;
    bool can_use_navigation_reference = false;
    NavigationSemanticEvent current_terminal_event;
    NavigationSemanticEvent upcoming_event;
};

struct NavigationRouteSnapshot {
    bool valid = false;
    bool route_changed = false;
    std::uint64_t route_id = 0;
    std::uint64_t route_signature = 0;
    double origin_latitude = 0.0;
    double origin_longitude = 0.0;
    int segment_count = 0;
    std::vector<NavigationSegmentInfo> segments;
};

struct NavigationReferenceResult {
    bool ok = false;
    std::string error;
    std::vector<Vec2> vcs_points;
    double backward_length_m = 0.0;
    double forward_length_m = 0.0;
    double lateral_error_m = 0.0;
    double heading_error_rad = 0.0;
    std::string stop_reason_forward;
    std::string stop_reason_backward;
};

class NavigationRouteTracker {
public:
    struct Config {
        double near_segment_end_distance_m = 50.0;
        double max_match_lateral_error_m = 30.0;
        double max_match_backward_s_m = 80.0;
        double max_match_forward_s_m = 30.0;
        int fallback_search_radius_segments = 2;
        double navigation_reference_front_x_m = 250.0;
        double navigation_reference_rear_x_m = -20.0;
        double navigation_reference_max_search_forward_m = 360.0;
        double navigation_reference_max_search_backward_m = 120.0;
        double navigation_reference_min_forward_m = 80.0;
        double navigation_reference_max_lateral_error_m = 12.0;
        double navigation_reference_max_heading_error_rad = 0.45;
        double navigation_reference_max_local_turn_rad = 0.52;
        double navigation_reference_max_total_heading_change_rad = 1.57;
    };

    NavigationRouteTracker();
    explicit NavigationRouteTracker(Config config);

    bool updateRoute(const snoah::SDRouteProto& route);
    NavigationMatchResult updateGnss(const snoah::GnssRawReadingProto& gnss);
    NavigationReferenceResult buildReferenceAroundVisual(
        const VisualReferenceResult& visual_reference,
        const snoah::GnssRawReadingProto& gnss) const;

    const NavigationRouteSnapshot& routeSnapshot() const { return route_; }
    const NavigationMatchResult& currentMatch() const { return match_; }
    bool canUseNavigationForReference() const {
        return match_.can_use_navigation_reference;
    }

private:
    struct ProjectionCandidate {
        bool valid = false;
        int segment_index = -1;
        double along_s_m = 0.0;
        double lateral_error_m = 0.0;
        double distance_to_end_m = 0.0;
        double score = 0.0;
    };

    struct RouteProjectionCandidate {
        bool valid = false;
        std::size_t segment_start_index = 0;
        double s_m = 0.0;
        double x_m = 0.0;
        double y_m = 0.0;
        double heading_rad = 0.0;
        double lateral_error_m = 0.0;
        double score = 0.0;
    };

    std::uint64_t computeRouteSignature(const snoah::SDRouteProto& route) const;
    NavigationSegmentInfo buildSegment(
        int index,
        const snoah::SDRouteProto_NavigationSegment& segment) const;
    NavigationSemanticEvent parseTerminalEvent(
        int segment_index,
        const std::string& instruction,
        double segment_length_m) const;
    NavRoutePoint geoToRoutePoint(double lat, double lon, double alt) const;
    bool gnssToRouteXY(const snoah::GnssRawReadingProto& gnss,
                       double* x_m,
                       double* y_m) const;
    bool vcsToRouteXY(const snoah::GnssRawReadingProto& gnss,
                      const Vec2& vcs,
                      double* x_m,
                      double* y_m) const;
    bool routeXYToVcs(const snoah::GnssRawReadingProto& gnss,
                      double x_m,
                      double y_m,
                      Vec2* vcs) const;
    void rebuildGlobalRoutePoints();
    ProjectionCandidate projectToSegment(int segment_index, double x_m, double y_m) const;
    ProjectionCandidate findBestCandidate(double x_m, double y_m) const;
    RouteProjectionCandidate projectToGlobalRoute(double x_m, double y_m) const;
    void fillMatchFromCandidate(const ProjectionCandidate& candidate);

    Config cfg_;
    NavigationRouteSnapshot route_;
    NavigationMatchResult match_;
    std::vector<NavRoutePoint> global_route_points_;
};

const char* navigationEventTypeName(NavigationEventType type);

}  // namespace topology_map::algorithms
