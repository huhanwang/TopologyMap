#include <sqlite3.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "boundary_sampling_module.h"
#include "fused_reference_module.h"
#include "navigation_route_tracker.h"
#include "offline_replay/replay_types.h"
#include "proto_preprocess_module.h"
#include "replay_debug_json.h"
#include "visual_reference_module.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using offline_replay::DataEntry;
using offline_replay::SnapshotFrame;
using offline_replay::SyncHit;
using topology_map::algorithms::VisualReferenceResult;

namespace {

enum class TimeType {
    kRaw,
    kLocal,
};

struct RangeConfig {
    std::string type = "all";
    std::int64_t start_frame = 0;
    std::int64_t end_frame = 0;
    double start_time = 0.0;
    double end_time = 0.0;
};

struct ReplayConfig {
    std::string db_path;
    std::string main_topic = "FusedStatic";
    TimeType time_type = TimeType::kRaw;
    RangeConfig range;
    std::vector<std::string> topics;
    int sleep_ms = 0;
    fs::path output_dir;
    int frame_dir_width = 10;
    bool write_raw_blobs = false;
    bool skip_existing_outputs = false;
    std::unordered_set<std::string> write_files;
};

struct TopicIndexItem {
    std::int64_t rowid = 0;
    std::int64_t frame_id = 0;
    std::int64_t raw_timestamp_us = 0;
    std::int64_t local_timestamp_us = 0;

    std::int64_t timeUs(TimeType type) const {
        return type == TimeType::kRaw ? raw_timestamp_us : local_timestamp_us;
    }
};

std::string readTextFile(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open file: " + path.string());
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void writeTextFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("failed to write file: " + path.string());
    }
    out << text;
}

bool shouldWriteFile(const ReplayConfig& cfg, const std::string& key) {
    return cfg.write_files.empty() || cfg.write_files.count(key) > 0;
}

void writeOutputFile(const ReplayConfig& cfg,
                     const std::string& key,
                     const fs::path& path,
                     const std::string& text) {
    if (!shouldWriteFile(cfg, key)) return;
    if (cfg.skip_existing_outputs && fs::exists(path)) return;
    writeTextFile(path, text);
}

std::string sqlQuoteIdent(const std::string& ident) {
    std::string out = "\"";
    for (char ch : ident) {
        if (ch == '"') out += "\"\"";
        else out += ch;
    }
    out += "\"";
    return out;
}

TimeType parseTimeType(const std::string& value) {
    if (value == "raw_timestamp" || value == "raw") return TimeType::kRaw;
    if (value == "local_timestamp" || value == "local") return TimeType::kLocal;
    throw std::runtime_error("unsupported time_type: " + value);
}

const char* timeTypeName(TimeType type) {
    return type == TimeType::kRaw ? "raw_timestamp" : "local_timestamp";
}

ReplayConfig loadConfig(const fs::path& path) {
    const auto j = json::parse(readTextFile(path));
    ReplayConfig cfg;
    cfg.db_path = j.at("db").get<std::string>();
    const auto& main_axis = j.at("main_axis");
    cfg.main_topic = main_axis.value("topic", cfg.main_topic);
    cfg.time_type = parseTimeType(main_axis.value("time_type", "raw_timestamp"));

    if (main_axis.contains("range")) {
        const auto& r = main_axis.at("range");
        cfg.range.type = r.value("type", "all");
        cfg.range.start_frame = r.value("start", r.value("start_frame", std::int64_t{0}));
        cfg.range.end_frame = r.value("end", r.value("end_frame", std::int64_t{0}));
        cfg.range.start_time = r.value("start_time", 0.0);
        cfg.range.end_time = r.value("end_time", 0.0);
    }

    cfg.topics = j.at("topics").get<std::vector<std::string>>();
    if (std::find(cfg.topics.begin(), cfg.topics.end(), cfg.main_topic) == cfg.topics.end()) {
        cfg.topics.insert(cfg.topics.begin(), cfg.main_topic);
    }

    if (j.contains("playback")) {
        cfg.sleep_ms = j.at("playback").value("sleep_ms", 0);
    }
    const auto& out = j.at("output");
    cfg.output_dir = out.at("dir").get<std::string>();
    cfg.frame_dir_width = out.value("frame_dir_width", 10);
    cfg.write_raw_blobs = out.value("write_raw_blobs", false);
    cfg.skip_existing_outputs = out.value("skip_existing", false);
    if (out.contains("write_files")) {
        for (const auto& item : out.at("write_files")) {
            cfg.write_files.insert(item.get<std::string>());
        }
    }
    return cfg;
}

class SqliteDb {
public:
    explicit SqliteDb(const std::string& path) {
        if (sqlite3_open_v2(path.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
            std::string err = db_ ? sqlite3_errmsg(db_) : "unknown sqlite error";
            if (db_) sqlite3_close(db_);
            db_ = nullptr;
            throw std::runtime_error("failed to open db: " + path + " err=" + err);
        }
    }

    ~SqliteDb() {
        if (db_) sqlite3_close(db_);
    }

    sqlite3* get() const { return db_; }

private:
    sqlite3* db_ = nullptr;
};

std::vector<TopicIndexItem> loadTopicIndex(sqlite3* db, const std::string& topic) {
    const std::string sql =
        "SELECT rowid, id, raw_timestamp, local_timestamp FROM " + sqlQuoteIdent(topic) +
        " ORDER BY raw_timestamp ASC, id ASC;";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error("failed to prepare index query for " + topic + ": " +
                                 sqlite3_errmsg(db));
    }

    std::vector<TopicIndexItem> items;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        TopicIndexItem item;
        item.rowid = sqlite3_column_int64(stmt, 0);
        item.frame_id = sqlite3_column_int64(stmt, 1);
        item.raw_timestamp_us = sqlite3_column_int64(stmt, 2);
        item.local_timestamp_us = sqlite3_column_int64(stmt, 3);
        items.push_back(item);
    }
    sqlite3_finalize(stmt);
    return items;
}

std::shared_ptr<DataEntry> loadDataEntry(sqlite3* db, const std::string& topic,
                                         const TopicIndexItem& item,
                                         bool load_blob) {
    auto entry = std::make_shared<DataEntry>();
    entry->topic = topic;
    entry->rowid = item.rowid;
    entry->frame_id = item.frame_id;
    entry->raw_timestamp_us = item.raw_timestamp_us;
    entry->local_timestamp_us = item.local_timestamp_us;
    entry->raw_data = std::make_shared<std::vector<std::uint8_t>>();

    if (!load_blob) {
        return entry;
    }

    const std::string sql =
        "SELECT raw_data FROM " + sqlQuoteIdent(topic) + " WHERE rowid = ?;";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error("failed to prepare blob query for " + topic + ": " +
                                 sqlite3_errmsg(db));
    }
    sqlite3_bind_int64(stmt, 1, item.rowid);
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        const void* blob = sqlite3_column_blob(stmt, 0);
        const int bytes = sqlite3_column_bytes(stmt, 0);
        if (blob && bytes > 0) {
            const auto* first = static_cast<const std::uint8_t*>(blob);
            entry->raw_data->assign(first, first + bytes);
        }
    }
    sqlite3_finalize(stmt);
    return entry;
}

std::vector<TopicIndexItem> selectMainAxisFrames(
    const std::vector<TopicIndexItem>& index, const ReplayConfig& cfg) {
    std::vector<TopicIndexItem> selected;
    if (cfg.range.type == "all") {
        selected = index;
    } else if (cfg.range.type == "frame") {
        for (const auto& item : index) {
            if (item.frame_id >= cfg.range.start_frame && item.frame_id <= cfg.range.end_frame) {
                selected.push_back(item);
            }
        }
    } else if (cfg.range.type == "time") {
        const auto start_us = static_cast<std::int64_t>(cfg.range.start_time * 1000000.0);
        const auto end_us = static_cast<std::int64_t>(cfg.range.end_time * 1000000.0);
        for (const auto& item : index) {
            const auto t = item.timeUs(cfg.time_type);
            if (t >= start_us && t <= end_us) {
                selected.push_back(item);
            }
        }
    } else {
        throw std::runtime_error("unsupported range type: " + cfg.range.type);
    }
    return selected;
}

const TopicIndexItem* findPreviousOrEqual(const std::vector<TopicIndexItem>& index,
                                          TimeType time_type,
                                          std::int64_t target_time_us) {
    auto it = std::upper_bound(
        index.begin(), index.end(), target_time_us,
        [time_type](std::int64_t value, const TopicIndexItem& item) {
            return value < item.timeUs(time_type);
        });
    if (it == index.begin()) return nullptr;
    --it;
    return &(*it);
}

std::string paddedFrameDirName(std::int64_t frame_id, int width) {
    std::ostringstream ss;
    ss << std::setw(width) << std::setfill('0') << frame_id;
    return ss.str();
}

json syncHitToJson(const SyncHit& hit) {
    json j;
    j["hit"] = hit.hit;
    j["topic"] = hit.topic;
    if (hit.hit) {
        j["rowid"] = hit.rowid;
        j["frame_id"] = hit.frame_id;
        j["raw_timestamp_us"] = hit.raw_timestamp_us;
        j["local_timestamp_us"] = hit.local_timestamp_us;
        j["delta_us"] = hit.delta_us;
        j["delta_ms"] = hit.delta_us / 1000.0;
    } else {
        j["reason"] = hit.reason;
    }
    return j;
}

json frameMetaToJson(const SnapshotFrame& frame) {
    json j;
    j["index"] = frame.index;
    j["main_topic"] = frame.main_topic;
    j["main_frame_id"] = frame.main_frame_id;
    j["main_time_us"] = frame.main_time_us;
    j["main_time_sec"] = frame.main_time_us / 1000000.0;
    j["entry_count"] = frame.entries.size();
    return j;
}

void writeSnapshotFiles(const ReplayConfig& cfg, const SnapshotFrame& frame,
                        const fs::path& frame_dir,
                        const json& preprocess_json,
                        const json& viz_json,
                        const json& visual_reference_json,
                        const json& sd_route_debug_json,
                        const json& navigation_tracker_json,
                        const json& navigation_reference_json,
                        const json& fused_reference_json,
                        const json& boundary_intersections_json,
                        const json& frenet_topology_debug_json,
                        const json& lane_center_debug_json,
                        const json& boundary_completion_debug_json,
                        const json& lane_region_debug_json) {
    json frame_meta = frameMetaToJson(frame);
    json sync_json = json::object();
    for (const auto& [topic, hit] : frame.sync) {
        sync_json[topic] = syncHitToJson(hit);
    }

    writeOutputFile(cfg, "frame", frame_dir / "frame.json", frame_meta.dump(2));
    writeOutputFile(cfg, "sync", frame_dir / "sync.json", sync_json.dump(2));
    writeOutputFile(cfg, "preprocess", frame_dir / "preprocess.json", preprocess_json.dump(2));
    writeOutputFile(cfg, "viz", frame_dir / "viz.json", viz_json.dump(2));
    writeOutputFile(cfg, "visual_reference", frame_dir / "visual_reference.json", visual_reference_json.dump(2));
    writeOutputFile(cfg, "sd_route_debug", frame_dir / "sd_route_debug.json", sd_route_debug_json.dump(2));
    writeOutputFile(cfg, "navigation_tracker", frame_dir / "navigation_tracker.json", navigation_tracker_json.dump(2));
    writeOutputFile(cfg, "navigation_reference", frame_dir / "navigation_reference.json", navigation_reference_json.dump(2));
    writeOutputFile(cfg, "fused_reference", frame_dir / "fused_reference.json", fused_reference_json.dump(2));
    writeOutputFile(cfg, "boundary_intersections", frame_dir / "boundary_intersections.json", boundary_intersections_json.dump(2));
    writeOutputFile(cfg, "frenet_topology_debug", frame_dir / "frenet_topology_debug.json", frenet_topology_debug_json.dump(2));
    writeOutputFile(cfg, "lane_center_debug", frame_dir / "lane_center_debug.json", lane_center_debug_json.dump(2));
    writeOutputFile(cfg, "boundary_completion_debug", frame_dir / "boundary_completion_debug.json", boundary_completion_debug_json.dump(2));
    writeOutputFile(cfg, "lane_region_debug", frame_dir / "lane_region_debug.json", lane_region_debug_json.dump(2));
}

std::vector<SnapshotFrame> buildSnapshots(
    sqlite3* db,
    const ReplayConfig& cfg,
    const std::unordered_map<std::string, std::vector<TopicIndexItem>>& indices,
    const std::vector<TopicIndexItem>& main_frames) {
    std::vector<SnapshotFrame> snapshots;
    snapshots.reserve(main_frames.size());

    std::unordered_map<std::string, std::unordered_map<std::int64_t, std::shared_ptr<DataEntry>>> data_cache;

    for (std::size_t i = 0; i < main_frames.size(); ++i) {
        const auto& main = main_frames[i];
        const auto main_time_us = main.timeUs(cfg.time_type);

        SnapshotFrame frame;
        frame.index = i;
        frame.main_topic = cfg.main_topic;
        frame.main_frame_id = main.frame_id;
        frame.main_time_us = main_time_us;

        for (const auto& topic : cfg.topics) {
            const TopicIndexItem* selected = nullptr;
            if (topic == cfg.main_topic) {
                selected = &main;
            } else {
                auto it = indices.find(topic);
                if (it != indices.end()) {
                    selected = findPreviousOrEqual(it->second, cfg.time_type, main_time_us);
                }
            }

            SyncHit hit;
            hit.topic = topic;
            if (!selected) {
                hit.hit = false;
                hit.reason = "no_previous_or_equal_frame";
                frame.sync[topic] = hit;
                continue;
            }

            auto& topic_cache = data_cache[topic];
            auto cached = topic_cache.find(selected->rowid);
            std::shared_ptr<DataEntry> entry;
            if (cached == topic_cache.end()) {
                entry = loadDataEntry(db, topic, *selected, cfg.write_raw_blobs);
                topic_cache[selected->rowid] = entry;
            } else {
                entry = cached->second;
            }

            frame.entries[topic] = entry;
            hit.hit = true;
            hit.rowid = selected->rowid;
            hit.frame_id = selected->frame_id;
            hit.raw_timestamp_us = selected->raw_timestamp_us;
            hit.local_timestamp_us = selected->local_timestamp_us;
            hit.delta_us = selected->timeUs(cfg.time_type) - main_time_us;
            frame.sync[topic] = hit;
        }

        snapshots.push_back(std::move(frame));
    }
    return snapshots;
}

void writeOutputs(const ReplayConfig& cfg, const std::vector<SnapshotFrame>& snapshots) {
    fs::create_directories(cfg.output_dir / "frames");
    offline_replay::algorithms::ProtoPreprocessModule proto_preprocess;
    topology_map::algorithms::NavigationRouteTracker navigation_tracker;
    topology_map::algorithms::VisualReferenceModule visual_reference_module;
    topology_map::algorithms::FusedReferenceModule fused_reference_module;
    topology_map::algorithms::BoundarySamplingModule boundary_sampling_module;
    // Temporary global-route debug exporter is kept in offline_replay/debug/global_route_debug.
    // Re-enable it when GNSS/SDRoute global alignment needs investigation again.
    // offline_replay::algorithms::GlobalRouteDebugExporter global_debug(
    //     cfg.output_dir / "global_debug");

    json index = json::array();
    for (const auto& frame : snapshots) {
        const auto dir_name = paddedFrameDirName(frame.main_frame_id, cfg.frame_dir_width);
        const auto rel_dir = fs::path("frames") / dir_name;
        const auto frame_dir = cfg.output_dir / rel_dir;
        const auto preprocessed = proto_preprocess.process(frame);
        const auto preprocess_json = offline_replay::debug::preprocessedSnapshotToJson(preprocessed);
        auto viz_json = offline_replay::debug::buildReplayVizJson(frame, preprocessed);
        const auto sd_route_debug_json = offline_replay::debug::sdRouteDebugToJson(frame, preprocessed);

        const auto route_it = preprocessed.entries.find("AutoSDRoute");
        if (route_it != preprocessed.entries.end() && route_it->second.ok && route_it->second.message) {
            const auto* route = dynamic_cast<const snoah::SDRouteProto*>(
                route_it->second.message.get());
            if (route) {
                navigation_tracker.updateRoute(*route);
            }
        }
        const auto gnss_it = preprocessed.entries.find("AutoSensorGnss");
        if (gnss_it != preprocessed.entries.end() && gnss_it->second.ok && gnss_it->second.message) {
            const auto* gnss = dynamic_cast<const snoah::GnssRawReadingProto*>(
                gnss_it->second.message.get());
            if (gnss) {
                navigation_tracker.updateGnss(*gnss);
            }
        }
        VisualReferenceResult visual_reference;
        const auto fused_it = preprocessed.entries.find("FusedStatic");
        if (fused_it == preprocessed.entries.end() || !fused_it->second.ok || !fused_it->second.message) {
            visual_reference.frame_id = frame.main_frame_id;
            visual_reference.error = "missing_or_unparsed_fused_static";
        } else {
            const auto* fused_static = dynamic_cast<const idrive::workflow::proto::FusedStaticMsg*>(
                fused_it->second.message.get());
            if (!fused_static) {
                visual_reference.frame_id = frame.main_frame_id;
                visual_reference.error = "fused_static_type_mismatch";
            } else {
                visual_reference = visual_reference_module.process(frame.main_frame_id, *fused_static);
            }
        }
        const auto visual_reference_json = offline_replay::debug::visualReferenceToJson(visual_reference);
        topology_map::algorithms::NavigationReferenceResult navigation_reference;
        if (gnss_it != preprocessed.entries.end() && gnss_it->second.ok && gnss_it->second.message) {
            const auto* gnss = dynamic_cast<const snoah::GnssRawReadingProto*>(
                gnss_it->second.message.get());
            if (gnss) {
                navigation_reference =
                    navigation_tracker.buildReferenceAroundVisual(visual_reference, *gnss);
            }
        }
        const auto navigation_tracker_json =
            offline_replay::debug::navigationTrackerDebugToJson(
                navigation_tracker.routeSnapshot(), navigation_tracker.currentMatch());
        const auto navigation_reference_json =
            offline_replay::debug::navigationReferenceToJson(navigation_reference);
        const auto fused_reference =
            fused_reference_module.process(visual_reference, navigation_reference);
        const auto fused_reference_json =
            offline_replay::debug::fusedReferenceToJson(fused_reference);
        topology_map::algorithms::BoundarySamplingResult boundary_intersections;
        if (fused_it != preprocessed.entries.end() && fused_it->second.ok && fused_it->second.message) {
            const auto* fused_static = dynamic_cast<const idrive::workflow::proto::FusedStaticMsg*>(
                fused_it->second.message.get());
            if (fused_static) {
                boundary_intersections =
                    boundary_sampling_module.process(frame.main_frame_id, fused_reference, *fused_static);
            }
        }
        const auto boundary_intersections_json =
            offline_replay::debug::boundaryIntersectionsToJson(boundary_intersections);
        const auto frenet_topology_debug_json =
            offline_replay::debug::frenetTopologyDebugToJson(boundary_intersections);
        const auto lane_center_debug_json =
            offline_replay::debug::laneCenterDebugToJson(boundary_intersections);
        const auto boundary_completion_debug_json =
            offline_replay::debug::boundaryCompletionDebugToJson(boundary_intersections);
        const auto lane_region_debug_json =
            offline_replay::debug::laneRegionDebugToJson(boundary_intersections);
        if (viz_json.contains("layers") && viz_json["layers"].is_array()) {
            const auto layer = offline_replay::debug::visualReferenceVizLayer(visual_reference);
            if (!layer.value("items", json::array()).empty()) {
                viz_json["layers"].push_back(layer);
            }
            const auto navigation_layer =
                offline_replay::debug::navigationReferenceVizLayer(navigation_reference);
            if (!navigation_layer.value("items", json::array()).empty()) {
                viz_json["layers"].push_back(navigation_layer);
            }
            const auto fused_layer =
                offline_replay::debug::fusedReferenceVizLayer(fused_reference);
            if (!fused_layer.value("items", json::array()).empty()) {
                viz_json["layers"].push_back(fused_layer);
            }
            const auto boundary_layer =
                offline_replay::debug::boundaryIntersectionsVizLayer(boundary_intersections);
            if (!boundary_layer.value("items", json::array()).empty()) {
                viz_json["layers"].push_back(boundary_layer);
            }
        }
        // global_debug.observe(frame, preprocessed);
        writeSnapshotFiles(cfg, frame, frame_dir, preprocess_json, viz_json,
                           visual_reference_json, sd_route_debug_json,
                           navigation_tracker_json, navigation_reference_json,
                           fused_reference_json, boundary_intersections_json,
                           frenet_topology_debug_json, lane_center_debug_json,
                           boundary_completion_debug_json, lane_region_debug_json);

        json item = frameMetaToJson(frame);
        item["dir"] = rel_dir.string();
        item["files"] = {
            {"frame", (rel_dir / "frame.json").string()},
            {"sync", (rel_dir / "sync.json").string()},
            {"preprocess", (rel_dir / "preprocess.json").string()},
            {"viz", (rel_dir / "viz.json").string()},
            {"visual_reference", (rel_dir / "visual_reference.json").string()},
            {"sd_route_debug", (rel_dir / "sd_route_debug.json").string()},
            {"navigation_tracker", (rel_dir / "navigation_tracker.json").string()},
            {"navigation_reference", (rel_dir / "navigation_reference.json").string()},
            {"fused_reference", (rel_dir / "fused_reference.json").string()},
            {"boundary_intersections", (rel_dir / "boundary_intersections.json").string()},
            {"frenet_topology_debug", (rel_dir / "frenet_topology_debug.json").string()},
            {"lane_center_debug", (rel_dir / "lane_center_debug.json").string()},
            {"boundary_completion_debug", (rel_dir / "boundary_completion_debug.json").string()},
            {"lane_region_debug", (rel_dir / "lane_region_debug.json").string()},
        };
        index.push_back(item);
    }

    json manifest;
    manifest["db"] = cfg.db_path;
    manifest["main_topic"] = cfg.main_topic;
    manifest["time_type"] = timeTypeName(cfg.time_type);
    manifest["sync_mode"] = "previous_or_equal";
    manifest["topics"] = cfg.topics;
    manifest["frame_count"] = snapshots.size();
    manifest["frames_index"] = "frames_index.json";

    writeTextFile(cfg.output_dir / "manifest.json", manifest.dump(2));
    writeTextFile(cfg.output_dir / "frames_index.json", index.dump(2));

    json dataset;
    dataset["manifest"] = manifest;
    dataset["frames"] = index;
    writeTextFile(cfg.output_dir / "dataset.js",
                  "window.REPLAY_DATASET = " + dataset.dump(2) + ";\n");
    // global_debug.close();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        fs::path config_path = "offline_replay/configs/fusedstatic_gnss_sdroute.json";
        if (argc >= 2) {
            config_path = argv[1];
        }

        const ReplayConfig cfg = loadConfig(config_path);
        std::cout << "[offline_replay] config=" << config_path << "\n";
        std::cout << "[offline_replay] db=" << cfg.db_path << "\n";
        std::cout << "[offline_replay] main_axis=" << cfg.main_topic
                  << " time_type=" << timeTypeName(cfg.time_type) << "\n";

        SqliteDb db(cfg.db_path);

        std::unordered_map<std::string, std::vector<TopicIndexItem>> indices;
        for (const auto& topic : cfg.topics) {
            auto index = loadTopicIndex(db.get(), topic);
            std::cout << "[offline_replay] indexed topic=" << topic
                      << " count=" << index.size() << "\n";
            indices.emplace(topic, std::move(index));
        }

        const auto main_it = indices.find(cfg.main_topic);
        if (main_it == indices.end() || main_it->second.empty()) {
            throw std::runtime_error("main topic has no data: " + cfg.main_topic);
        }
        const auto main_frames = selectMainAxisFrames(main_it->second, cfg);
        std::cout << "[offline_replay] selected main frames=" << main_frames.size() << "\n";

        auto snapshots = buildSnapshots(db.get(), cfg, indices, main_frames);
        std::cout << "[offline_replay] built snapshots=" << snapshots.size() << "\n";

        if (cfg.sleep_ms > 0) {
            for (const auto& frame : snapshots) {
                (void)frame;
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg.sleep_ms));
            }
        }

        writeOutputs(cfg, snapshots);
        std::cout << "[offline_replay] wrote output=" << cfg.output_dir << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[offline_replay][error] " << e.what() << "\n";
        return 1;
    }
}
