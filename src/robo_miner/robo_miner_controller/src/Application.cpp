#include "robo_miner_controller/Application.h"

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
    if (ErrorCode::FAILURE == initLongestSequenceMinerExternalBridge()) {
        LOGERR("Could not initialize LongestSequenceMinerExternalBridge");
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

ErrorCode Application::initLongestSequenceMinerExternalBridge()
{
    m_longestSequenceMinerExternalBridge = std::make_shared<LongestSequenceMinerExternalBridge>();
    if (ErrorCode::FAILURE == m_longestSequenceMinerExternalBridge->init()) {
        return ErrorCode::FAILURE;
    }
    m_ros2Communicator->registerNode(m_longestSequenceMinerExternalBridge);
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
    longestChainOfMinerals();
    mineLongestChainOfMinerals();
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
        dynamicFieldMap.at(m_robotState.fieldPos) = initialValue;
        surroundingPositions = getSurroundingPos(m_robotState);
        for (int i = 0; i < RoboCommonDefines::SURROUNDING_TILES_CTN; i++) {
            dynamicFieldMap.at(surroundingPositions[i]) = surroundingMarkers[i];
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
                dynamicFieldMap.at(surroundingPositions[i]) = surroundingMarkers[i];
            }
        }
        return success;
    };

    // one rotation at the beginning to explore all neighbouring fields
    // this is needed because position behind robot will be unexplored in dynamic map
    // and result in an error when trying to reach it
    robotMove(MoveType::ROTATE_LEFT);

    // TODO: refactor BFS to be a function
    // DFS
    // integer in the pair shows index of neighbour to be processed
    std::stack<std::pair<FieldPos, size_t>> s;
    // White := unvisited
    // Gray := visited, but not all neighbours are processed
    // Black := all neighbours are processed
    enum class NodeColor {WHITE, GRAY, BLACK};
    DynamicGrid<NodeColor> color;
    const FieldPos nil{invalid, invalid};
    m_predecessors.at(m_robotState.fieldPos) = nil;

    color.at(m_robotState.fieldPos) = NodeColor::GRAY;
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
                color.at(v) = NodeColor::WHITE;
            }
            if (color.at(v) == NodeColor::WHITE && canStepOn(dynamicFieldMap.at(v))) {
                color.at(v) = NodeColor::GRAY;
                s.push(std::make_pair(v,0));
                m_predecessors.at(v) = u.first;
            }

        } else {
            color.at(u.first) = NodeColor::BLACK;
        }
    }

    m_dynamicMap = dynamicFieldMap;
    m_map = convertDynamicFieldMap(dynamicFieldMap);
    auto min_col = std::min_element(dynamicFieldMap.cbegin(), dynamicFieldMap.cend(), [](const auto & a, const auto & b){
        return a.first.col < b.first.col;
    });
    auto min_row = std::min_element(dynamicFieldMap.cbegin(), dynamicFieldMap.cend(), [](const auto & a, const auto & b){
        return a.first.row < b.first.row;
    });

    m_topLeft = FieldPos{min_row->first.row + 1, min_col->first.col + 1};
    m_mapExplorerExternalBridge->queryFieldMapValidate(m_map);
}

// TODO: use refactored BFS function instead of DFS // it was quicker to write
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

void Application::longestChainOfMinerals()
{
    std::vector<FieldPos> longestSequence;
    Grid<int> visited{m_map.rows(), m_map.cols(), 0};
    FieldPos longest_pos;
    for (int32_t r = 0; r < m_map.rows(); r++) {
    for (int32_t c = 0; c < m_map.cols(); c++) {
        FieldPos current{r,c};
        LOG("Checkng pos %d %d", current.row, current.col);
        if (!visited.at(current) && canStepOn(m_map.at(current))) {
            auto sequence = bfs(current, m_map, visited);
            if (sequence.size() > longestSequence.size()) {
                longest_pos = current;
                longestSequence = std::move(sequence);
                LOG("New longest pos at %d %d", r, c);
            }
        }
    }
    }
    m_longestSequenceMinerExternalBridge->queryLongestSequenceValidate(longestSequence);

    // go to longest pos
    // Convert to local coordinates
    longest_pos = {m_topLeft.row + longest_pos.row, m_topLeft.col + longest_pos.col};
    LOG("Longest pos: %d %d", longest_pos.row, longest_pos.col);

    // reconstruct path
    std::vector<FieldPos> path;
    FieldPos crawl = longest_pos;
    LOG("Path: %d %d", crawl.row, crawl.col);
    path.push_back(crawl);
    const FieldPos nil{invalid, invalid};
    while(m_predecessors.at(crawl) != nil) {
        crawl = m_predecessors.at(crawl);
        path.push_back(crawl);
        LOG("Path: %d %d", crawl.row, crawl.col);
    }

    for (auto rit = path.rbegin(); rit != path.rend(); rit++) {
        moveToNeighbour(m_robotState, *rit, [this](MoveType moveType){
            SurroundingTiles surroundingMarkers;
            m_mapExplorerExternalBridge->queryRobotMove(moveType, surroundingMarkers, m_robotState.dir);
            m_robotState.fieldPos = updatePosition(moveType, m_robotState.fieldPos, m_robotState.dir);
        });
    }
    m_longestSequenceMinerExternalBridge->queryActivateMiningValidate();
}

void Application::mineLongestChainOfMinerals()
{
    // DFS
    // integer in the pair shows index of neighbour to be processed
    std::stack<std::pair<FieldPos, size_t>> s;
    // White := unvisited
    // Gray := visited, but not all neighbours are processed
    // Black := all neighbours are processed
    enum class NodeColor {WHITE, GRAY, BLACK};
    DynamicGrid<NodeColor> color;
    const FieldPos nil{invalid, invalid};
    m_predecessors.at(m_robotState.fieldPos) = nil;

    color.at(m_robotState.fieldPos) = NodeColor::GRAY;
    s.push(std::make_pair(m_robotState.fieldPos, 0));

    const std::array<FieldPos, 4> neighbours = {{{0,-1},{1,0},{0,1},{-1,0}}};

    uint8_t crystal = m_dynamicMap.at(m_robotState.fieldPos);
    
    while (!s.empty()) {
        auto u = s.top();
        s.pop();
        // TODO: fix bug wehere assert in moveToNeighbour fails at level 3
        moveToNeighbour(m_robotState, u.first, [this](MoveType moveType){
            SurroundingTiles surroundingMarkers;
            m_mapExplorerExternalBridge->queryRobotMove(moveType, surroundingMarkers, m_robotState.dir);
            m_robotState.fieldPos = updatePosition(moveType, m_robotState.fieldPos, m_robotState.dir);
        });
        if (u.second < neighbours.size()) {
            s.push(std::make_pair(u.first, u.second + 1));
            auto const & n = neighbours[u.second];
            auto const v = FieldPos{u.first.row + n.row, u.first.col + n.col};
            if (!color.contains(v)) {
                color.at(v) = NodeColor::WHITE;
            }
            if (color.at(v) == NodeColor::WHITE && canStepOn(m_dynamicMap.at(v) && m_dynamicMap.at(v) == crystal)) {
                color.at(v) = NodeColor::GRAY;
                s.push(std::make_pair(v,0));
                m_predecessors.at(v) = u.first;
            }

        } else {
            color.at(u.first) = NodeColor::BLACK;
        }
    }
}