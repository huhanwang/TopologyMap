#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <google/protobuf/message.h>

#include "offline_replay/replay_types.h"

namespace offline_replay::algorithms {

struct ParsedProtoEntry {
    bool ok = false;
    std::string topic;
    std::string proto_type;
    std::string error;
    std::shared_ptr<google::protobuf::Message> message;
};

struct PreprocessedSnapshot {
    std::unordered_map<std::string, ParsedProtoEntry> entries;
};

class ProtoPreprocessModule {
public:
    ProtoPreprocessModule();

    PreprocessedSnapshot process(const SnapshotFrame& snapshot) const;

private:
    using Parser = std::function<std::shared_ptr<google::protobuf::Message>(
        const std::vector<std::uint8_t>&, std::string*)>;

    template <typename ProtoT>
    void registerParser(const std::string& topic);

    std::unordered_map<std::string, Parser> parsers_;
};

}  // namespace offline_replay::algorithms
