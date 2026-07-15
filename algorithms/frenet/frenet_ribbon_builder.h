#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "boundary_sampling/boundary_sampling_module.h"
#include "frenet_track_builder.h"

namespace topology_map::algorithms {

enum class FrenetRibbonKind {
    kTopologyLane,
    kLaneToBoundaryRef,
    kBoundaryOnlyRef,
};

enum class FrenetRibbonEndpointDirection {
    kBeforeTrackHead,
    kAfterTrackTail,
};

std::string frenetRibbonKindToString(FrenetRibbonKind kind);

struct FrenetRibbonSample {
    int slice_index = -1;
    double s = 0.0;
    std::string right_track_id;
    std::string left_track_id;
    double right_l = 0.0;
    double left_l = 0.0;
    double width_m = 0.0;
    double center_l_m = 0.0;
};

struct FrenetRibbonTrack {
    std::string id;
    std::string label;
    std::string pair_id;
    int segment_index = 0;
    std::string right_track_id;
    std::string left_track_id;
    FrenetBoundaryType right_type = FrenetBoundaryType::kUnknown;
    FrenetBoundaryType left_type = FrenetBoundaryType::kUnknown;
    std::string right_source_type;
    std::string left_source_type;
    FrenetRibbonKind kind = FrenetRibbonKind::kBoundaryOnlyRef;
    std::vector<FrenetRibbonSample> samples;
};

struct RibbonSampleRef {
    std::string ribbon_id;
    int ribbon_index = -1;
    int ribbon_sample_index = -1;
    int slice_index = -1;
    double s = 0.0;
    std::string self_track_id;
    std::string neighbor_track_id;
    bool self_is_right_boundary = false;
    bool self_is_left_boundary = false;
};

struct TrackRibbonRefs {
    std::vector<RibbonSampleRef> left_side;
    std::vector<RibbonSampleRef> right_side;
};

struct RibbonStatsOptions {
    double s_min = -1.0e100;
    double s_max = 1.0e100;
    bool reject_outliers = true;
    double mad_scale = 3.0;
    double min_abs_outlier_threshold_m = 0.5;
};

struct RibbonStats {
    int sample_count = 0;
    int used_sample_count = 0;
    double width_mean = 0.0;
    double width_trimmed_mean = 0.0;
    double width_median = 0.0;
    double width_std = 0.0;
    std::vector<int> outlier_sample_indices;
};

struct RibbonWidthEstimate {
    double width_m = 0.0;
    double confidence = 0.0;
    std::string method;
    double support_s = 0.0;
    double support_distance_m = 0.0;
};

struct RibbonTrendOptions {
    double s_min = -1.0e100;
    double s_max = 1.0e100;
    double stable_width_slope_threshold = 0.015;
    double stable_center_slope_threshold = 0.015;
};

struct RibbonTrend {
    int sample_count = 0;
    double width_slope_m_per_m = 0.0;
    double center_slope_m_per_m = 0.0;
    std::string width_trend = "unknown";
    std::string center_trend = "unknown";
};

struct FrenetRibbonResult {
    bool ok = false;
    std::string error;
    std::int64_t frame_id = 0;
    std::vector<FrenetRibbonTrack> ribbons;
    std::unordered_map<std::string, std::size_t> ribbon_index_by_id;
    std::unordered_map<std::string, TrackRibbonRefs> ribbons_by_track_id;
    std::vector<std::vector<RibbonSampleRef>> ribbons_by_slice;

    const FrenetRibbonTrack* ribbonById(const std::string& ribbon_id) const;
    const TrackRibbonRefs* ribbonsOfTrack(const std::string& track_id) const;
    std::vector<RibbonSampleRef> ribbonsOfTrackAtSlice(const std::string& track_id, int slice_index) const;
    std::vector<const FrenetRibbonTrack*> topologyRibbonsAtEndpoint(
        const FrenetTrack& track,
        FrenetRibbonEndpointDirection direction,
        double endpoint_tolerance_m) const;
    RibbonStats computeStats(const std::string& ribbon_id, const RibbonStatsOptions& options = {}) const;
    RibbonTrend computeTrend(const std::string& ribbon_id, const RibbonTrendOptions& options = {}) const;
    std::optional<RibbonWidthEstimate> estimateWidth(const std::string& ribbon_id, double s, const RibbonStatsOptions& options = {}) const;
};

class FrenetRibbonBuilder {
public:
    struct Config {
        double gap_threshold_m = 3.2;
    };

    FrenetRibbonBuilder();
    explicit FrenetRibbonBuilder(Config config);

    FrenetRibbonResult build(const BoundarySamplingResult& samples,
                             const FrenetTrackResult& tracks) const;

private:
    Config cfg_;
};

}  // namespace topology_map::algorithms
