#include "robo_common/defines/RoboCommonDefines.h"

std::string RobotState::toString() const
{
    return "robotId = " + std::to_string(robotId) + 
        "; fieldPos.r = " + std::to_string(fieldPos.row) + 
        "; fieldPos.c = " + std::to_string(fieldPos.col) +
        "; dir = " + std::to_string(std::underlying_type<Direction>::type(dir));  
}