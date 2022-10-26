#ifndef APPLICATION_H_
#define APPLICATION_H_

#include <memory>
#include <mutex>

#include "robo_miner_controller/external_api/MapExplorerExternalBridge.h"
#include "robo_miner_controller/external_api/LongestSequenceMinerExternalBridge.h"
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
    void longestChainOfMinerals();
    void mineLongestChainOfMinerals();

    ErrorCode initMapExplorerExternalBridge();
    ErrorCode initLongestSequenceMinerExternalBridge();
    
    void waitExternalBridgeReady();
    void signalExternalBridgeReady();

    std::shared_ptr<MapExplorerExternalBridge> m_mapExplorerExternalBridge;
    std::shared_ptr<LongestSequenceMinerExternalBridge> m_longestSequenceMinerExternalBridge;
    std::shared_ptr<Ros2Communicator> m_ros2Communicator;

    std::mutex m_nodeSpinReadyMtx;
    std::condition_variable m_nodeSpinCond;
    bool m_nodeSpinning = false;

    static constexpr auto invalid = std::numeric_limits<decltype(FieldPos::col)>::min();
    FieldPos m_topLeft;
    DynamicGrid<FieldPos> m_predecessors;
    ApplicationConfig m_config;
    RobotState m_robotState;
    FieldMap m_map;
    DynamicFieldMap m_dynamicMap;
};

#endif // APPLICATION_H_
