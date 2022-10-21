#include "robo_miner_controller/external_api/MapExplorerExternalBridge.h"

namespace {
    constexpr auto NODE_NAME = "MapExplorerExternalBridge";

    template <typename T>
    void waitForService(const T &client) {
        using namespace std::literals;
        while (!client->wait_for_service(1s)) {
            LOG("Service: [%s] not available. Waiting for 1s ...", client->get_service_name());
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

    m_userAuthenticatePublisher = create_publisher<UserAuthenticate>(USER_AUTHENTICATE_TOPIC, qos);
    m_toggleHelpPagePublisher = create_publisher<Empty>(TOGGLE_HELP_PAGE_TOPIC, qos);
    m_toggleDebugInfoPublisher = create_publisher<Empty>(TOGGLE_DEBUG_INFO_TOPIC, qos);
    m_setDebugMsgPublisher = create_publisher<String>(DEBUG_MSG_TOPIC, qos);

    m_initialRobotPosClient = create_client<QueryInitialRobotPosition>(QUERY_INITIAL_ROBOT_POSITION_SERVICE);
    m_robotMoveClient = create_client<RobotMove>(ROBOT_MOVE_SERVICE);
    m_fieldMapValidateClient = create_client<FieldMapValidate>(FIELD_MAP_VALIDATE_SERVICE);

    m_nodeSpinningTimer = create_wall_timer(1us, std::bind(&MapExplorerExternalBridge::onNodeSpinning, this));

    waitForService(m_initialRobotPosClient);
    waitForService(m_robotMoveClient);
    waitForService(m_fieldMapValidateClient);

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

bool MapExplorerExternalBridge::queryFieldMapValidate(FieldMap const & fieldMap) const
{
    const auto request = std::make_shared<FieldMapValidate::Request>();
    request->field_map.set__cols(fieldMap.cols());
    request->field_map.set__rows(fieldMap.rows());
    request->field_map.set__data(fieldMap.asVector());
    auto result = m_fieldMapValidateClient->async_send_request(request);
    auto response = result.get();

    bool success = response->success;
    if (!success) {
        publishDebugMsg(response->error_reason);
        LOGY("%s", response->error_reason.c_str());
    }
    return success;
}

bool MapExplorerExternalBridge::queryInitialPosition(SurroundingTiles & surroundingTiles, Direction & direction, uint8_t & initial) const
{
    const auto request = std::make_shared<QueryInitialRobotPosition::Request>();
    const auto result = m_initialRobotPosClient->async_send_request(request);
    const auto response = result.get();

    const auto & position_response = response->robot_position_response;
    bool success = position_response.success;
    direction = getRobotDirection(position_response.robot_dir);
    surroundingTiles = position_response.surrounding_tiles;
    initial = response->robot_initial_tile;
    if (!success) {
        publishDebugMsg(position_response.error_reason);
        LOGY("%s", position_response.error_reason.c_str());
    }
    return success;
}
bool MapExplorerExternalBridge::queryRobotMove(MoveType moveType, SurroundingTiles & surroundingTiles, Direction & direction) const
{
    auto request = std::make_shared<RobotMove::Request>();
    request->robot_move_type.move_type = getMoveTypeField(moveType);
    auto result = m_robotMoveClient->async_send_request(request);
    auto response = result.get();
    
    const auto & position_response = response->robot_position_response;
    bool success = position_response.success;
    direction = getRobotDirection(position_response.robot_dir);
    surroundingTiles = position_response.surrounding_tiles;

    return success;
}

void MapExplorerExternalBridge::onNodeSpinning()
{
    LOG("Node is spinning");
    m_nodeSpinningTimer->cancel();
    m_outInterface.onExternalBridgeReadyCb();
}