#ifndef LONGESTVEINMINEREXTERNALBRIDGE_H_
#define LONGESTVEINMINEREXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>

#include "utils/ErrorCode.h"
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/srv/activate_mining_validate.hpp"

class LongestSequenceMinerExternalBridge : public rclcpp::Node
{
public:
    LongestSequenceMinerExternalBridge();
    ErrorCode init();
    
    bool queryLongestSequenceValidate(std::vector<FieldPos> const & longesSequence);
    bool queryActivateMiningValidate();
private:
    using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
    using ActivateMiningValidate = robo_miner_interfaces::srv::ActivateMiningValidate;

    using FieldPoint = robo_miner_interfaces::msg::FieldPoint;

    rclcpp::Client<LongestSequenceValidate>::SharedPtr m_longestSequenceValidateClient;
    rclcpp::Client<ActivateMiningValidate>::SharedPtr m_activateMiningValidate;
};

#endif // LONGESTVINEMINEREXTERNALBRIDGE_H_
