#ifndef MAPEXPLOREREXTERNALBRIDGE_H_
#define MAPEXPLOREREXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>

#include <robo_miner_interfaces/srv/detail/query_initial_robot_position__struct.hpp>
#include <robo_miner_interfaces/srv/robot_move.hpp>
#include <robo_miner_interfaces/srv/field_map_validate.hpp>

#include <robo_miner_interfaces/msg/user_authenticate.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/detail/empty__struct.hpp>

#include <robo_miner_common/message_helpers/RoboMinerMessageHelpers.h>
#include <robo_miner_common/defines/RoboMinerTopics.h>
#include <robo_common/defines/RoboCommonDefines.h>

#include <utils/ErrorCode.h>
#include <utils/Log.h>
#include <functional>

struct MapExplorerExternalBridgeOutInterface
{
    std::function<void()> onExternalBridgeReadyCb;
};

class MapExplorerExternalBridge : public rclcpp::Node
{
public:
    MapExplorerExternalBridge();

    ErrorCode init(MapExplorerExternalBridgeOutInterface const & outInterface);

    void publishUserData(UserData const & userData) const;
    void publishToggleHelpPage() const;
    void publishToggleDebugInfo() const;
    void publishDebugMsg(std::string const & msg) const;
    bool queryFieldMapValidate(FieldMap const & fieldMap) const;
    bool queryInitialPosition(SurroundingTiles & surroundingTiles, Direction & direction, uint8_t & initial) const;
    bool queryRobotMove(MoveType moveType, SurroundingTiles & surroundingTiles, Direction & direction) const;
    
private:
    using RobotMove = robo_miner_interfaces::srv::RobotMove;
    using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
    using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;

    using UserAuthenticate = robo_miner_interfaces::msg::UserAuthenticate;
    using Empty = std_msgs::msg::Empty;
    using String = std_msgs::msg::String;

    rclcpp::Publisher<UserAuthenticate>::SharedPtr m_userAuthenticatePublisher;
    rclcpp::Publisher<Empty>::SharedPtr m_toggleHelpPagePublisher;
    rclcpp::Publisher<Empty>::SharedPtr m_toggleDebugInfoPublisher;
    rclcpp::Publisher<String>::SharedPtr m_setDebugMsgPublisher;

    rclcpp::Client<QueryInitialRobotPosition>::SharedPtr m_initialRobotPosClient;
    rclcpp::Client<RobotMove>::SharedPtr m_robotMoveClient;
    rclcpp::Client<FieldMapValidate>::SharedPtr m_fieldMapValidateClient;

    rclcpp::TimerBase::SharedPtr m_nodeSpinningTimer;

    void onNodeSpinning();

    MapExplorerExternalBridgeOutInterface m_outInterface;
};

#endif // MAPEXPLOREREXTERNALBRIDGE_H_
