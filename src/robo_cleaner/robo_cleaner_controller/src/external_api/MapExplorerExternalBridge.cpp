#include "robo_cleaner_controller/external_api/MapExplorerExternalBridge.h"

#include <future>

#include <robo_common/Misc.h>

namespace {
    constexpr auto NODE_NAME = "MapExplorerExternalBridge";

    template <typename T>
    void waitForService(const T &client) {
        using namespace std::literals;
        while (!client->wait_for_service(1s)) {
            LOG("Service: [%s] not available. Waiting for 1s ...", client->get_service_name());
        }
    }

    template <typename T, typename ActionName>
    void waitForAction(const T &action, const ActionName &actionName) {
        using namespace std::literals;
        while (!action->wait_for_action_server(1s)) {
            LOG("Action: [%s] not available. Waiting for 1s ...)", actionName);
        }
    }
}

MapExplorerExternalBridge::MapExplorerExternalBridge()
    : Node(NODE_NAME)
{
};

ErrorCode MapExplorerExternalBridge::init(MapExplorerExternalBridgeOutInterface const & outInterface)
{
    using namespace std::literals;
    if(!outInterface.onExternalBridgeReadyCb) {
        return ErrorCode::FAILURE;
    }

    m_outInterface = outInterface;

    constexpr size_t queueSize = 10;
    const rclcpp::QoS qos(queueSize);

    m_actionCallbackGroup = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    m_robotMoveActionClient = rclcpp_action::create_client<RobotMove>(this, ROBOT_MOVE_ACTION, m_actionCallbackGroup);

    m_userAuthenticatePublisher = create_publisher<UserAuthenticate>(USER_AUTHENTICATE_TOPIC, qos);
    m_toggleHelpPagePublisher = create_publisher<Empty>(TOGGLE_HELP_PAGE_TOPIC, qos);
    m_toggleDebugInfoPublisher = create_publisher<Empty>(TOGGLE_DEBUG_INFO_TOPIC, qos);
    m_setDebugMsgPublisher = create_publisher<String>(DEBUG_MSG_TOPIC, qos);

    m_initialRobotStateClient = create_client<QueryInitialRobotState>(QUERY_INITIAL_ROBOT_STATE_SERVICE);
    m_batteryStatusClient = create_client<QueryBatteryStatus>(QUERY_BATTERY_STATUS_SERVICE);
    m_chargeBatteryClient = create_client<ChargeBattery>(CHARGE_BATTERY_SERVICE);


    m_nodeSpinningTimer = create_wall_timer(1us, std::bind(&MapExplorerExternalBridge::onNodeSpinning, this));

    waitForService(m_initialRobotStateClient);
    waitForService(m_batteryStatusClient);
    waitForService(m_chargeBatteryClient);

    waitForAction(m_robotMoveActionClient, ROBOT_MOVE_ACTION);

    return ErrorCode::SUCCESS;
}

void MapExplorerExternalBridge::publishUserData(UserData const & userData) const
{
    UserAuthenticate msg;
    msg.user = userData.user;
    msg.repository = userData.repository;
    msg.commit_sha = userData.commitSha;
    m_userAuthenticatePublisher->publish(msg);
}

void MapExplorerExternalBridge::publishToggleHelpPage() const
{
    m_toggleHelpPagePublisher->publish(Empty());
}

void MapExplorerExternalBridge::publishToggleDebugInfo() const
{
    m_toggleDebugInfoPublisher->publish(Empty());
}

void MapExplorerExternalBridge::publishDebugMsg(std::string const & msg) const
{
    String debugMessage;
    debugMessage.set__data(msg);
    m_setDebugMsgPublisher->publish(debugMessage);
}

bool MapExplorerExternalBridge::queryInitialRobotState(Battery & battery, uint8_t & tile, Direction & direction)
{
    const auto request = std::make_shared<QueryInitialRobotState::Request>();
    const auto result = m_initialRobotStateClient->async_send_request(request);
    const auto response = result.get();

    const auto & initial_robot_state = response->initial_robot_state;
    bool success = response->success;
    direction = getRobotDirection(initial_robot_state.robot_dir);
    tile = initial_robot_state.robot_tile;
    battery.max_moves = initial_robot_state.battery_status.max_moves_on_full_energy;
    battery.moves_left = initial_robot_state.battery_status.moves_left;
    if (!success) {
        m_errorMessage = response->error_reason;
    }
    return success;
}

void MapExplorerExternalBridge::queryBatteryStatus(Battery & battery)
{
    const auto request = std::make_shared<QueryBatteryStatus::Request>();
    const auto result = m_batteryStatusClient->async_send_request(request);
    const auto response = result.get();

    battery.max_moves = response->battery_status.max_moves_on_full_energy;
    battery.moves_left = response->battery_status.moves_left;
}

bool MapExplorerExternalBridge::queryChargeBattery(int32_t turns, Battery & battery, int32_t & turns_charged)
{
    const auto request = std::make_shared<ChargeBattery::Request>();
    request->set__charge_turns(turns);
    const auto result = m_chargeBatteryClient->async_send_request(request);
    const auto response = result.get();

    battery.max_moves = response->battery_status.max_moves_on_full_energy;
    battery.moves_left = response->battery_status.moves_left;
    turns_charged = response->turns_spend_charging;

    bool success = response->success;
    if (!success) {
        m_errorMessage = response->error_reason;
    }
    return success;
}

bool MapExplorerExternalBridge::robotMove(MoveType moveType, uint8_t & marker)
{
    using namespace std::placeholders;

    auto goalMsg = RobotMove::Goal();
    RobotMoveType moveMsg;
    moveMsg.set__move_type(getMoveTypeField(moveType)); 
    goalMsg.robot_move_type = moveMsg;

    auto sendGoalOptions = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    sendGoalOptions.feedback_callback = std::bind(&MapExplorerExternalBridge::onRobotMoveFeedBack, this, _1, _2);
    std::packaged_task<void(const GoalHandleRobotMove::WrappedResult & result)> moveResultTask(std::bind(&MapExplorerExternalBridge::onRobotMoveResult, this, _1));
    auto future_result = moveResultTask.get_future();
    sendGoalOptions.result_callback = [&moveResultTask](const GoalHandleRobotMove::WrappedResult & result){ moveResultTask(result); };

    auto result = m_robotMoveActionClient->async_send_goal(goalMsg, sendGoalOptions);
    auto response = result.get();

    if (!response) {
        LOG("Server rejected goal");
        return false;
    } else {
        LOG("Server accepted goal");
    }

    future_result.get();

    if (!m_success) {
        return false;
    }
    marker = m_last_field_marker;
    return true;
}

void MapExplorerExternalBridge::onRobotMoveFeedBack([[maybe_unused]]const GoalHandleRobotMove::SharedPtr goalHandle, const std::shared_ptr<const RobotMove::Feedback> feedback)
{
    LOG("Approaching marker: '%c'", feedback->approaching_field_marker);
    if (!canStepOn(feedback->approaching_field_marker))  {
        // TODO: async_cancel_goal causes deadlock 
        m_robotMoveActionClient->async_cancel_all_goals();
    }
}

void MapExplorerExternalBridge::onRobotMoveResult(const GoalHandleRobotMove::WrappedResult & result)
{
    m_success = result.result->success;
    m_last_field_marker = result.result->processed_field_marker;
}

void MapExplorerExternalBridge::onNodeSpinning()
{
    LOG("Node is spinning");
    m_nodeSpinningTimer->cancel();
    m_outInterface.onExternalBridgeReadyCb();
}
