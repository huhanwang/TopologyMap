#include "global_route_debug_exporter.h"

#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/positioning.pb.h"

#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace offline_replay::algorithms {
namespace {
constexpr double kRadToDeg = 180.0 / M_PI;

void writeGeoPoint(std::ofstream& out,
                   const std::string& source,
                   std::int64_t route_rowid,
                   std::uint64_t route_id,
                   int group_index,
                   int point_index,
                   const snoah::mapping::GeoPointProto& point) {
    out << source << ','
        << route_rowid << ','
        << route_id << ','
        << group_index << ','
        << point_index << ','
        << std::setprecision(15) << point.longitude() << ','
        << std::setprecision(15) << point.latitude() << ','
        << std::setprecision(15) << point.longitude() * kRadToDeg << ','
        << std::setprecision(15) << point.latitude() * kRadToDeg << ','
        << std::setprecision(15) << point.altitude() << '\n';
}

}  // namespace

GlobalRouteDebugExporter::GlobalRouteDebugExporter(const std::filesystem::path& output_dir) {
    std::filesystem::create_directories(output_dir);
    gnss_csv_.open(output_dir / "gnss.csv");
    route_csv_.open(output_dir / "sd_route.csv");
    if (!gnss_csv_ || !route_csv_) {
        throw std::runtime_error("failed to open global route debug csv output");
    }

    gnss_csv_ << "rowid,frame_id,raw_timestamp_us,local_timestamp_us,"
              << "longitude_rad,latitude_rad,longitude_deg,latitude_deg,"
              << "altitude,yaw_rad,yaw_deg,heading_deg_from_enu_yaw,"
              << "vel_x,vel_y,vel_z,vel_angle_rad,vel_angle_deg\n";
    route_csv_ << "source,route_rowid,route_id,group_index,point_index,"
               << "longitude_rad,latitude_rad,longitude_deg,latitude_deg,altitude\n";
}

void GlobalRouteDebugExporter::observe(
    const SnapshotFrame& snapshot,
    const PreprocessedSnapshot& preprocessed) {
    const auto gnss_entry_it = snapshot.entries.find("AutoSensorGnss");
    const auto gnss_proto_it = preprocessed.entries.find("AutoSensorGnss");
    if (gnss_entry_it != snapshot.entries.end() &&
        gnss_proto_it != preprocessed.entries.end() &&
        gnss_proto_it->second.ok &&
        gnss_proto_it->second.message &&
        seen_gnss_rows_.insert(gnss_entry_it->second->rowid).second) {
        const auto* gnss = dynamic_cast<const snoah::GnssRawReadingProto*>(
            gnss_proto_it->second.message.get());
        if (gnss && gnss->has_longitude() && gnss->has_latitude()) {
            const double yaw = gnss->has_yaw() ? gnss->yaw() : 0.0;
            const double vel_angle = std::atan2(gnss->vel_y(), gnss->vel_x());
            gnss_csv_ << gnss_entry_it->second->rowid << ','
                      << gnss_entry_it->second->frame_id << ','
                      << gnss_entry_it->second->raw_timestamp_us << ','
                      << gnss_entry_it->second->local_timestamp_us << ','
                      << std::setprecision(15) << gnss->longitude() << ','
                      << std::setprecision(15) << gnss->latitude() << ','
                      << std::setprecision(15) << gnss->longitude() * kRadToDeg << ','
                      << std::setprecision(15) << gnss->latitude() * kRadToDeg << ','
                      << std::setprecision(15) << (gnss->has_altitude() ? gnss->altitude() : 0.0) << ','
                      << std::setprecision(15) << yaw << ','
                      << std::setprecision(15) << yaw * kRadToDeg << ','
                      << std::setprecision(15) << 90.0 - yaw * kRadToDeg << ','
                      << std::setprecision(15) << gnss->vel_x() << ','
                      << std::setprecision(15) << gnss->vel_y() << ','
                      << std::setprecision(15) << gnss->vel_z() << ','
                      << std::setprecision(15) << vel_angle << ','
                      << std::setprecision(15) << vel_angle * kRadToDeg << '\n';
        }
    }

    const auto route_entry_it = snapshot.entries.find("AutoSDRoute");
    const auto route_proto_it = preprocessed.entries.find("AutoSDRoute");
    if (route_entry_it == snapshot.entries.end() ||
        route_proto_it == preprocessed.entries.end() ||
        !route_proto_it->second.ok ||
        !route_proto_it->second.message ||
        !seen_route_rows_.insert(route_entry_it->second->rowid).second) {
        return;
    }

    const auto* route = dynamic_cast<const snoah::SDRouteProto*>(
        route_proto_it->second.message.get());
    if (!route) return;

    int link_group = 0;
    for (const auto& section : route->sd_sections()) {
        for (const auto& link : section.sd_links()) {
            for (int i = 0; i < link.points_size(); ++i) {
                writeGeoPoint(route_csv_, "sd_link", route_entry_it->second->rowid,
                              route->route_id(), link_group, i, link.points(i));
            }
            ++link_group;
        }
    }

    for (int seg_idx = 0; seg_idx < route->navigation_segments_size(); ++seg_idx) {
        const auto& segment = route->navigation_segments(seg_idx);
        for (int i = 0; i < segment.points_size(); ++i) {
            writeGeoPoint(route_csv_, "navigation_segment", route_entry_it->second->rowid,
                          route->route_id(), seg_idx, i, segment.points(i));
        }
    }
}

void GlobalRouteDebugExporter::close() {
    gnss_csv_.close();
    route_csv_.close();
}

}  // namespace offline_replay::algorithms
