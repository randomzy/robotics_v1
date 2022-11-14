#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "robo_arm_controller/external_api/RoboArmExternalBridge.h"
#include "utils/ErrorCode.h"
#include "ros2_game_engine/communicator/Ros2Communicator.h"

struct ApplicationConfig
{
    Ros2CommunicatorConfig ros2CommunicatorConfig;
};

class Application
{
public:
    ErrorCode init(ApplicationConfig const & config);
    void deinit();
    void run();
private:
    ErrorCode initRoboArmExternalBridge();
    
    void waitExternalBridgeReady();
    void signalExternalBridgeReady();

    std::shared_ptr<Ros2Communicator> m_ros2Communicator;
    std::shared_ptr<RoboArmExternalBridge> m_roboArmExternalBridge;

    std::mutex m_nodeSpinReadyMtx;
    std::condition_variable m_nodeSpinCond;
    bool m_nodeSpinning = false;

};

#endif // APPLICATION_H_
