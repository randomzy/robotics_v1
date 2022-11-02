#ifndef APPLICATION_H_
#define APPLICATION_H_

#include <memory>
#include <mutex>

#include "robo_cleaner_controller/external_api/MapExplorerExternalBridge.h"
#include "ros2_game_engine/communicator/Ros2Communicator.h"
#include "utils/ErrorCode.h"

struct ApplicationConfig
{
    UserData userdata;
    Ros2CommunicatorConfig ros2CommunicatorConfig;
};

class Application
{
public:
    ErrorCode init(ApplicationConfig const & config);
    void deinit();
    void run();
private:
    void exploreMap();

    ErrorCode initMapExplorerExternalBridge();
    
    void waitExternalBridgeReady();
    void signalExternalBridgeReady();

    std::shared_ptr<MapExplorerExternalBridge> m_mapExplorerExternalBridge;
    std::shared_ptr<Ros2Communicator> m_ros2Communicator;

    std::mutex m_nodeSpinReadyMtx;
    std::condition_variable m_nodeSpinCond;
    bool m_nodeSpinning = false;

    static constexpr auto invalid = std::numeric_limits<decltype(FieldPos::col)>::min();
    ApplicationConfig m_config;
    RobotState m_robotState;
    DynamicFieldMap m_dynamicMap;
};

#endif // APPLICATION_H_
