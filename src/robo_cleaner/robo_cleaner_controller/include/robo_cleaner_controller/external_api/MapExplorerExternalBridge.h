#ifndef MAPEXPLOREREXTERNALBRIDGE_H_
#define MAPEXPLOREREXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robo_cleaner_interfaces/action/robot_move.hpp>

#include <robo_cleaner_interfaces/srv/query_initial_robot_state.hpp>
#include <robo_cleaner_interfaces/srv/charge_battery.hpp>
#include <robo_cleaner_interfaces/srv/query_battery_status.hpp>

#include <robo_cleaner_interfaces/msg/user_authenticate.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/detail/empty__struct.hpp>

#include <robo_cleaner_common/message_helpers/RoboCleanerMessageHelpers.h>
#include <robo_cleaner_common/defines/RoboCleanerTopics.h>
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

    bool queryInitialRobotState(Battery & battery, uint8_t & tile, Direction & direction);
    void queryBatteryStatus(Battery & battery);
    bool queryChargeBattery(int32_t turns, Battery & battery, int32_t & turns_charged);

    bool robotMove(MoveType moveType, uint8_t & marker);

    // TODO: writing and getting error message is not thread safe
    std::string_view getErrorMessage() const;
private:
    std::string m_errorMessage;

    using RobotMove = robo_cleaner_interfaces::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

    using QueryInitialRobotState = robo_cleaner_interfaces::srv::QueryInitialRobotState;
    using QueryBatteryStatus = robo_cleaner_interfaces::srv::QueryBatteryStatus;
    using ChargeBattery = robo_cleaner_interfaces::srv::ChargeBattery;

    using UserAuthenticate = robo_cleaner_interfaces::msg::UserAuthenticate;
    using RobotMoveType = robo_cleaner_interfaces::msg::RobotMoveType;
    using Empty = std_msgs::msg::Empty;
    using String = std_msgs::msg::String;

    void onRobotMoveGoalResponse(std::shared_future<GoalHandleRobotMove::SharedPtr> future);
    void onRobotMoveFeedBack(const GoalHandleRobotMove::SharedPtr goalHandle, const std::shared_ptr<const RobotMove::Feedback> feedback);
    void onRobotMoveResult(const GoalHandleRobotMove::WrappedResult & result);

    void onNodeSpinning();

    rclcpp::CallbackGroup::SharedPtr m_actionCallbackGroup;

    rclcpp_action::Client<RobotMove>::SharedPtr m_robotMoveActionClient;

    rclcpp::Client<QueryInitialRobotState>::SharedPtr m_initialRobotStateClient;
    rclcpp::Client<QueryBatteryStatus>::SharedPtr m_batteryStatusClient;
    rclcpp::Client<ChargeBattery>::SharedPtr m_chargeBatteryClient;

    rclcpp::Publisher<UserAuthenticate>::SharedPtr m_userAuthenticatePublisher;
    rclcpp::Publisher<Empty>::SharedPtr m_toggleHelpPagePublisher;
    rclcpp::Publisher<Empty>::SharedPtr m_toggleDebugInfoPublisher;
    rclcpp::Publisher<String>::SharedPtr m_setDebugMsgPublisher;

    rclcpp::TimerBase::SharedPtr m_nodeSpinningTimer;

    uint8_t m_last_field_marker;
    uint8_t m_success = false;

    MapExplorerExternalBridgeOutInterface m_outInterface;
};

#endif // MAPEXPLOREREXTERNALBRIDGE_H_
