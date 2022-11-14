#ifndef ROBOARMEXTERNALBRIDGE_H_
#define ROBOARMEXTERNALBRIDGE_H_

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>

#include "utils/ErrorCode.h"

#include "urscript_interfaces/srv/ur_script.hpp"
#include "urscript_interfaces/srv/get_eef_angle_axis.hpp"

struct RoboArmExternalBridgeOutInterface
{
    std::function<void()> onExternalBridgeReadyCb;
};

class RoboArmExternalBridge : public rclcpp::Node
{
public:
    RoboArmExternalBridge();
    ErrorCode init(RoboArmExternalBridgeOutInterface const & outInterface);
    bool sendUrScript(std::string const & script);
    bool queryToolOrientation(tf2::Vector3 & angleAxis);
private:
    using UrScript = urscript_interfaces::srv::UrScript;
    using GetEefAngleAxis = urscript_interfaces::srv::GetEefAngleAxis;

    rclcpp::CallbackGroup::SharedPtr m_actionCallbackGroup;

    rclcpp::Client<UrScript>::SharedPtr m_sendScriptClient;
    rclcpp::Client<GetEefAngleAxis>::SharedPtr m_eefAngleAxisClient;

    rclcpp::TimerBase::SharedPtr m_nodeSpinningTimer;

    RoboArmExternalBridgeOutInterface m_outInterface;

    void onNodeSpinning();
};

#endif // ROBOARMEXTERNALBRIDGE_H_
