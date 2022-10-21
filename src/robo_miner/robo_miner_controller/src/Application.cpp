#include "robo_miner_controller/Application.h"

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
    using SurroundingPositions = std::array<FieldPos, RoboCommonDefines::SURROUNDING_TILES_CTN>;

    SurroundingTiles surroundingMarkers;
    SurroundingPositions surroundingPositions;

    DynamicGrid<uint8_t> dynamicFieldMap;

    m_robotState.fieldPos = {0,0};

    {
        uint8_t initialValue;
        m_mapExplorerExternalBridge->queryInitialPosition(surroundingMarkers, m_robotState.dir, initialValue);
        dynamicFieldMap.insert(m_robotState.fieldPos, initialValue);
        surroundingPositions = getSurroundingPos(m_robotState);
        for (int i = 0; i < RoboCommonDefines::SURROUNDING_TILES_CTN; i++) {
            dynamicFieldMap.insert(surroundingPositions[i], surroundingMarkers[i]);
        }
    }

    auto robotMove = [&](MoveType moveType) -> bool{
        LOG("RobotMove: %d", moveType);
        bool success = m_mapExplorerExternalBridge->queryRobotMove(moveType, surroundingMarkers, m_robotState.dir);
        m_robotState.fieldPos = updatePosition(moveType, m_robotState.fieldPos, m_robotState.dir);
        LOG("RobotState: %s", m_robotState.toString().c_str());
        surroundingPositions = getSurroundingPos(m_robotState);
        for (int i = 0; i < RoboCommonDefines::SURROUNDING_TILES_CTN; i++) {
            if (!dynamicFieldMap.contains(surroundingPositions[i])) {
                dynamicFieldMap.insert(surroundingPositions[i], surroundingMarkers[i]);
            }
        }
        return success;
    };

    // one rotation at the beginning to explore all neighbouring fields
    // this is needed because position behind robot will be unexplored in dynamic map
    // and result in an error when trying to reach it
    robotMove(MoveType::ROTATE_LEFT);

    // DFS
    // integer in the pair shows index of neighbour to be processed
    std::stack<std::pair<FieldPos, size_t>> s;
    // White := unvisited
    // Gray := visited, but not all neighbours are processed
    // Black := all neighbours are processed
    enum class NodeColor {WHITE, GRAY, BLACK};
    DynamicGrid<NodeColor> color;

    color.insert(m_robotState.fieldPos, NodeColor::GRAY);
    s.push(std::make_pair(m_robotState.fieldPos, 0));

    const std::array<FieldPos, 4> neighbours = {{{0,-1},{1,0},{0,1},{-1,0}}};
    
    while (!s.empty()) {
        auto u = s.top();
        s.pop();
        moveToNeighbour(m_robotState, u.first, robotMove);
        if (u.second < neighbours.size()) {
            s.push(std::make_pair(u.first, u.second + 1));
            auto const & n = neighbours[u.second];
            auto const v = FieldPos{u.first.row + n.row, u.first.col + n.col};
            if (!color.contains(v)) {
                color.insert(v, NodeColor::WHITE);
            }
            if (color.at(v) == NodeColor::WHITE && canStepOn(dynamicFieldMap.at(v))) {
                color.at(v) = NodeColor::GRAY;
                s.push(std::make_pair(v,0));
            }

        } else {
            color.at(u.first) = NodeColor::BLACK;
        }
    }
    
    FieldMap fieldMap = convertDynamicFieldMap(dynamicFieldMap);
    m_mapExplorerExternalBridge->queryFieldMapValidate(fieldMap);
}