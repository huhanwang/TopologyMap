#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_set>

#include "offline_replay/replay_types.h"
#include "proto_preprocess/proto_preprocess_module.h"

namespace offline_replay::algorithms {

class GlobalRouteDebugExporter {
public:
    explicit GlobalRouteDebugExporter(const std::filesystem::path& output_dir);

    void observe(const SnapshotFrame& snapshot, const PreprocessedSnapshot& preprocessed);
    void close();

private:
    std::ofstream gnss_csv_;
    std::ofstream route_csv_;
    std::unordered_set<std::int64_t> seen_gnss_rows_;
    std::unordered_set<std::int64_t> seen_route_rows_;
};

}  // namespace offline_replay::algorithms
