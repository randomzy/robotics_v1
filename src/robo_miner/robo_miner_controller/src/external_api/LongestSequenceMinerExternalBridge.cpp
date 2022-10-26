#include "robo_miner_controller/external_api/LongestSequenceMinerExternalBridge.h"

#include "robo_miner_common/defines/RoboMinerTopics.h"
#include "utils/Log.h"

namespace {
    constexpr auto NODE_NAME = "LongestVeinMinerExternalBridge";

    template<typename T>
    void waitForService(const T &client) {
        using namespace std::literals;
        while (!client->wait_for_service(1s)) {
            LOG("Service: [%s] not available. Waiting for 1s ...", client->get_service_name());
        }
    }
}

LongestSequenceMinerExternalBridge::LongestSequenceMinerExternalBridge()
    : rclcpp::Node(NODE_NAME)
{
}

ErrorCode LongestSequenceMinerExternalBridge::init()
{
    m_longestSequenceValidateClient = create_client<LongestSequenceValidate>(LONGEST_SEQUENCE_VALIDATE_SERVICE);
    m_activateMiningValidate = create_client<ActivateMiningValidate>(ACTIVATE_MINING_VALIDATE_SERVICE);

    waitForService(m_longestSequenceValidateClient);
    return ErrorCode::SUCCESS;
}

bool LongestSequenceMinerExternalBridge::queryLongestSequenceValidate(std::vector<FieldPos> const & longesSequence)
{
    auto request = std::make_shared<LongestSequenceValidate::Request>();
    std::vector<FieldPoint> longestSequenceTr;
    std::transform(longesSequence.begin(), longesSequence.end(), std::back_inserter(longestSequenceTr), [](FieldPos const & in){
        FieldPoint out;
        out.set__row(in.row);
        out.set__col(in.col);
        LOG("Pos: %d %d", in.row, in.col);
        return out;
    });
    request->set__sequence_points(longestSequenceTr);
    auto result = m_longestSequenceValidateClient->async_send_request(request);
    auto response = result.get();
    return response->success;
}

bool LongestSequenceMinerExternalBridge::queryActivateMiningValidate()
{
    const auto request = std::make_shared<ActivateMiningValidate::Request>();
    const auto result = m_activateMiningValidate->async_send_request(request);
    const auto response = result.get();
    return response->success;
}