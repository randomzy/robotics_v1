#include "robo_cleaner_controller/Application.h"

#include <set>
#include <stack>
#include <functional>

#include "robo_common/Misc.h"

ErrorCode Application::init(ApplicationConfig const & config)
{
    m_config = config;
    m_ros2Communicator = std::make_shared<Ros2Communicator>();
    if (ErrorCode::FAILURE == m_ros2Communicator->init(m_config.ros2CommunicatorConfig)) {
        LOGERR("Could not initialize ros2Communcator");
        return ErrorCode::FAILURE;
    }
    if (ErrorCode::FAILURE == initMapExplorerExternalBridge()) {
        LOGERR("Could not initialize MapExplorerExternalBridge");
    }
    return ErrorCode::SUCCESS;
}

ErrorCode Application::initMapExplorerExternalBridge()
{
    m_mapExplorerExternalBridge = std::make_shared<MapExplorerExternalBridge>();
    MapExplorerExternalBridgeOutInterface outInterface;
    outInterface.onExternalBridgeReadyCb = std::bind(&Application::signalExternalBridgeReady, this);
    if (ErrorCode::FAILURE == m_mapExplorerExternalBridge->init(outInterface)) {
        return ErrorCode::FAILURE;
    }
    m_ros2Communicator->registerNode(m_mapExplorerExternalBridge);
    return ErrorCode::SUCCESS;
}

void Application::waitExternalBridgeReady()
{
    std::unique_lock<std::mutex> lock(m_nodeSpinReadyMtx);
    m_nodeSpinCond.wait(lock, [this](){return m_nodeSpinning;});
}

void Application::signalExternalBridgeReady()
{
    std::lock_guard<std::mutex> lock(m_nodeSpinReadyMtx);
    m_nodeSpinning = true;
    m_nodeSpinCond.notify_all();
}

void Application::deinit()
{
    m_ros2Communicator->unregisterNode(m_mapExplorerExternalBridge);
    m_mapExplorerExternalBridge.reset();
    m_ros2Communicator->deinit();
    m_ros2Communicator.reset();
}

void Application::run()
{
    m_ros2Communicator->start();
    waitExternalBridgeReady();
    m_mapExplorerExternalBridge->publishUserData(m_config.userdata);
    exploreMap();
}

void Application::exploreMap()
{
    DynamicGrid<uint8_t> dynamicFieldMap;

    m_robotState.fieldPos = {0,0};

    {
        uint8_t marker;
        m_mapExplorerExternalBridge->queryInitialRobotState(m_robotState.battery, marker, m_robotState.dir);
        dynamicFieldMap.at(m_robotState.fieldPos) = marker;
    }

    auto robotMove = [&](MoveType moveType) -> bool{
        uint8_t marker;
        m_mapExplorerExternalBridge->robotMove(moveType, marker);
        auto posInFront = FieldUtils::getAdjacentPos(m_robotState.dir, m_robotState.fieldPos);
        switch (moveType) {
            case MoveType::FORWARD:
                if (!dynamicFieldMap.contains(posInFront)) {
                    dynamicFieldMap.at(posInFront) = marker;
                    if (canStepOn(marker)) {
                        m_robotState.fieldPos = posInFront;
                    }
                }
                break;
            case MoveType::ROTATE_LEFT:
                m_robotState.dir = rotateCC(m_robotState.dir);
                break;
            case MoveType::ROTATE_RIGHT:
                m_robotState.dir = rotateCW(m_robotState.dir);
                break;
            default:
                LOGERR("Unknown move type");
        }
        m_mapExplorerExternalBridge->queryBatteryStatus(m_robotState.battery);
        return true;
    };

    

    robotMove(MoveType::ROTATE_LEFT);
    robotMove(MoveType::FORWARD);
    robotMove(MoveType::ROTATE_RIGHT);
    robotMove(MoveType::FORWARD);

    auto map = convertDynamicFieldMap<false>(dynamicFieldMap);
    std::cout << toString(map) << std::endl;
}

std::vector<FieldPos> bfs(FieldPos const & start, Grid<uint8_t> const & map, Grid<int> & visited)
{
    std::vector<FieldPos> sequence;
    std::queue<FieldPos> q;
    q.push(start);
    visited.at(start) = 1;
    sequence.emplace_back(start);
    uint8_t type = map.at(start);
    const std::array<FieldPos, 4> neighbours = {{{0,-1},{1,0},{0,1},{-1,0}}};
    while(!q.empty()) {
        auto curr = q.front();
        q.pop();
        for (auto const & neighbour : neighbours) {
            FieldPos v(curr.row + neighbour.row, curr.col + neighbour.col);
            if (map.contains(v) && !visited.at(v) && canStepOn(map.at(v)) && map.at(v) == type) {
                visited.at(v) = 1;
                q.push(v);
                sequence.emplace_back(v);
            }
        }
    }
    return sequence;
}
