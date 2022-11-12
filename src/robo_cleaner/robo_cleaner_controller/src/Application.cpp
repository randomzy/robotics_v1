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

inline std::vector<FieldPos> shortestPath(DynamicFieldMap const & map, FieldPos const & src, FieldPos const & dest)
{
    std::queue<FieldPos> queue;
    DynamicGrid<bool> visited;
    DynamicGrid<FieldPos> predecessor;

    const auto invalid = std::numeric_limits<decltype(FieldPos::col)>::min();
    const FieldPos nil{invalid, invalid};
    const std::array<FieldPos, 4> neighbours = {{{0,-1},{1,0},{0,1},{-1,0}}};

    visited.at(src) = true;
    predecessor.at(src) = nil;
    queue.push(src);

    bool found = false;

    while(!queue.empty() && !found) {
        auto curr = queue.front();
        queue.pop();
        for (auto const & neighbour : neighbours) {
            FieldPos next{neighbour.row + curr.row, neighbour.col + curr.col};
            if (!visited.contains(next) && 
                map.contains(next) &&
                canStepOn(map.at(next))
            ) {
                visited.at(next) = true;
                queue.push(next);
                predecessor.at(next) = curr;
                if (next == dest) {
                    found = true;
                    break;
                }
            }
        }
    }

    // reconstruct path
    std::vector<FieldPos> path;
    FieldPos crawl = dest; 
    path.push_back(crawl);
    predecessor.at(crawl);
    while(predecessor.at(crawl) != nil) {
        crawl = predecessor.at(crawl);
        path.push_back(crawl);
    }

    std::reverse(path.begin(), path.end());

    return path;
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
        while (!m_mapExplorerExternalBridge->robotMove(moveType, marker)) {
            LOG("robot move was canceled or could not be complete, trying again...");
        }
        LOG("Current robot state: %s", m_robotState.toString().c_str());
        LOG("Marker infront: '%c'", marker);
        switch (moveType) {
            case MoveType::FORWARD:
            {
                auto posInFront = FieldUtils::getAdjacentPos(m_robotState.dir, m_robotState.fieldPos);

                if (!dynamicFieldMap.contains(posInFront)) {
                    dynamicFieldMap.at(posInFront) = marker;
                }
                if (canStepOn(marker)) {
                    m_robotState.fieldPos = posInFront;
                }
                break;
            }
            case MoveType::ROTATE_LEFT:
                m_robotState.dir = rotateCC(m_robotState.dir);
                break;
            case MoveType::ROTATE_RIGHT:
                m_robotState.dir = rotateCW(m_robotState.dir);
                break;
            default:
                LOGERR("Unknown move type");
        }
        LOG("New robot state: %s", m_robotState.toString().c_str());
        m_mapExplorerExternalBridge->queryBatteryStatus(m_robotState.battery);
        return true;
    };


    robotMove(MoveType::ROTATE_LEFT);
    robotMove(MoveType::FORWARD);
    robotMove(MoveType::FORWARD);
    return;
    

    std::stack<FieldPos> frontier;
    const std::array<FieldPos, 4> neighbours = {{{0,-1},{1,0},{0,1},{-1,0}}};
    for (auto const & n : neighbours) {
        frontier.push(n);
    }
    while (!frontier.empty()) {
        // auto next = std::min_element(frontier.begin(), frontier.end(), [this](FieldPos const & a, FieldPos const b){
        //     return stZDistance(a, m_robotState.fieldPos) < stZDistance(b, m_robotState.fieldPos);
        // });
        auto u = frontier.top();
        LOG("Upcoming pos: %s", toString(u).c_str());
        frontier.pop();
        if (!canStepOn(dynamicFieldMap.at(u))) {
            LOG("Dropping current pos");
            continue;
        }
        LOG("Move from %s to %s", toString(m_robotState.fieldPos).c_str(), toString(u).c_str());
        auto path = shortestPath(dynamicFieldMap, m_robotState.fieldPos, u);
        for (auto const & step : path) {
            moveToNeighbour(m_robotState, step, [&](MoveType mt){robotMove(mt);});
            if (canStepOn(m_dynamicMap.at(step))) {
                for (const auto & neighbour : neighbours) {
                    FieldPos v = m_robotState.fieldPos + neighbour;
                    if (!dynamicFieldMap.contains(v)) {
                        frontier.push(v);
                    }
                }
            }
        }
    }

    auto map = convertDynamicFieldMap<false>(dynamicFieldMap);
    std::cout << toString(map) << std::endl;
}
