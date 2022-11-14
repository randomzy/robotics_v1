#include "robo_arm_controller/external_api/RoboArmExternalBridge.h"
#include "urscript_common/defines/UrScriptTopics.h"
#include "utils/Log.h"

namespace {
    constexpr auto NODE_NAME = "RoboArmExternalBridge";

    template <typename T>
    void waitForService(const T &client) {
        using namespace std::literals;
        while (!client->wait_for_service(1s)) {
            LOG("Service: [%s] not available. Waiting for 1s ...", client->get_service_name());
        }
    }
}

RoboArmExternalBridge::RoboArmExternalBridge()
    : Node(NODE_NAME)
{
}

ErrorCode RoboArmExternalBridge::init(RoboArmExternalBridgeOutInterface const & outInterface)
{
    using namespace std::literals;
    if (!outInterface.onExternalBridgeReadyCb) {
        return ErrorCode::FAILURE;
    }

    m_outInterface = outInterface;

    constexpr size_t queueSize = 10;
    const rclcpp::QoS qos(queueSize);

    m_sendScriptClient = create_client<UrScript>(URSCRIPT_SERVICE);
    m_eefAngleAxisClient = create_client<GetEefAngleAxis>(GET_EEF_ANGLE_AXIS_SERVICE);

    m_nodeSpinningTimer = create_wall_timer(1us, std::bind(&RoboArmExternalBridge::onNodeSpinning, this));

    waitForService(m_sendScriptClient);
    waitForService(m_eefAngleAxisClient);

    return ErrorCode::SUCCESS;
}

bool RoboArmExternalBridge::sendUrScript(std::string const & script)
{
    auto request = std::make_shared<UrScript::Request>();
    request->set__data(script);
    auto result =  m_sendScriptClient->async_send_request(request);
    auto response = result.get();

    bool success = response->success;
    if (!success) {
        LOG("%s", response->error_reason.c_str());
    }
    return success;
}

bool RoboArmExternalBridge::queryToolOrientation(tf2::Vector3 & angleAxis)
{
    auto request = std::make_shared<GetEefAngleAxis::Request>();
    auto result = m_eefAngleAxisClient->async_send_request(request);
    auto response = result.get();

    bool success = response->success;
    if (!success) {
        LOG("%s", response->error_reason.c_str());
    }
    angleAxis.setX(response->angle_axis.x);
    angleAxis.setY(response->angle_axis.y);
    angleAxis.setZ(response->angle_axis.z);
    return success;
}

void RoboArmExternalBridge::onNodeSpinning()
{
    LOG("Node is spinning");
    m_nodeSpinningTimer->cancel();
    m_outInterface.onExternalBridgeReadyCb();
}
