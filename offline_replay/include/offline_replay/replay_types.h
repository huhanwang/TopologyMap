#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace offline_replay {

struct DataEntry {
    std::string topic;
    std::int64_t rowid = 0;
    std::int64_t frame_id = 0;
    std::int64_t raw_timestamp_us = 0;
    std::int64_t local_timestamp_us = 0;
    std::shared_ptr<std::vector<std::uint8_t>> raw_data;
};

struct SyncHit {
    bool hit = false;
    std::string topic;
    std::int64_t rowid = 0;
    std::int64_t frame_id = 0;
    std::int64_t raw_timestamp_us = 0;
    std::int64_t local_timestamp_us = 0;
    std::int64_t delta_us = 0;
    std::string reason;
};

struct SnapshotFrame {
    std::size_t index = 0;
    std::string main_topic;
    std::int64_t main_frame_id = 0;
    std::int64_t main_time_us = 0;
    std::unordered_map<std::string, std::shared_ptr<const DataEntry>> entries;
    std::map<std::string, SyncHit> sync;
};

}  // namespace offline_replay
