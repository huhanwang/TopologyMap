#include "proto_preprocess_module.h"

#include "interface/pilot/fused_static.pb.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace offline_replay::algorithms {

template <typename ProtoT>
void ProtoPreprocessModule::registerParser(const std::string& topic) {
    parsers_[topic] = [](const std::vector<std::uint8_t>& bytes,
                         std::string* error) -> std::shared_ptr<google::protobuf::Message> {
        auto msg = std::make_shared<ProtoT>();
        if (!msg->ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
            if (error) *error = "ParseFromArray failed";
            return nullptr;
        }
        return msg;
    };
}

ProtoPreprocessModule::ProtoPreprocessModule() {
    registerParser<idrive::workflow::proto::FusedStaticMsg>("FusedStatic");
    registerParser<snoah::GnssRawReadingProto>("AutoSensorGnss");
    registerParser<snoah::SDRouteProto>("AutoSDRoute");
}

PreprocessedSnapshot ProtoPreprocessModule::process(const SnapshotFrame& snapshot) const {
    PreprocessedSnapshot out;
    for (const auto& [topic, entry] : snapshot.entries) {
        ParsedProtoEntry parsed;
        parsed.topic = topic;

        const auto parser_it = parsers_.find(topic);
        if (parser_it == parsers_.end()) {
            parsed.error = "no parser registered";
            out.entries.emplace(topic, std::move(parsed));
            continue;
        }
        if (!entry || !entry->raw_data || entry->raw_data->empty()) {
            parsed.error = "empty raw_data";
            out.entries.emplace(topic, std::move(parsed));
            continue;
        }

        std::string error;
        parsed.message = parser_it->second(*entry->raw_data, &error);
        parsed.ok = static_cast<bool>(parsed.message);
        parsed.error = error;
        if (parsed.message) {
            parsed.proto_type = parsed.message->GetDescriptor()
                ? parsed.message->GetDescriptor()->full_name()
                : parsed.message->GetTypeName();
        }
        out.entries.emplace(topic, std::move(parsed));
    }
    return out;
}

}  // namespace offline_replay::algorithms
