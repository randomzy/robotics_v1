#include "robo_arm_controller/Application.h"
#include "utils/Log.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <deque>
#include <tf2/LinearMath/Quaternion.h>
#include "robo_arm_controller/Math.h"
#include "robo_arm_controller/CommandWrappers.h"

namespace {
    constexpr auto PROJECT_NAME = "robo_arm_controller";
    const auto projectInstallPrefix =
    ament_index_cpp::get_package_share_directory(PROJECT_NAME);
}

inline std::string load_file(const std::string & fname)
{
    std::ifstream t(fname);
    if (!t) {
        throw std::runtime_error("Could not open file " + fname);
    }
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

ErrorCode Application::init(RoboArmConfig const & config)
{
    m_ros2Communicator = std::make_shared<Ros2Communicator>();

    m_cfg = config;

    if (ErrorCode::FAILURE == m_ros2Communicator->init(config.ros2CommunicatorConfig)) {
        LOGERR("Could not initialize ros2Communcator");
        return ErrorCode::FAILURE;
    }

    if (ErrorCode::FAILURE == initRoboArmExternalBridge()) {
        LOGERR("Could not initialize MapExplorerExternalBridge");
    }

    try {
        m_URScriptHardwareHeader = load_file(projectInstallPrefix + "/resources/scripts/header.script");
    } catch (std::exception & e) {
        LOGERR("%s", e.what());
        return ErrorCode::FAILURE; 
    }

    return ErrorCode::SUCCESS;
}

ErrorCode Application::initRoboArmExternalBridge()
{
    m_roboArmExternalBridge = std::make_shared<RoboArmExternalBridge>();
    RoboArmExternalBridgeOutInterface outInterface;
    outInterface.onExternalBridgeReadyCb = std::bind(&Application::signalExternalBridgeReady, this);
    if (ErrorCode::FAILURE == m_roboArmExternalBridge->init(outInterface)) {
        return ErrorCode::FAILURE;
    }
    m_ros2Communicator->registerNode(m_roboArmExternalBridge);
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
    m_ros2Communicator->unregisterNode(m_roboArmExternalBridge);
    m_roboArmExternalBridge.reset();
    m_ros2Communicator->deinit();
    m_ros2Communicator.reset();
}

void Application::run()
{
    // generate staircase, the center of the first "step" is situated at (0,0,0)
    // and the steps goes up in the positive Y direction
    std::deque<tf2::Vector3> staircase;
    for (int c = 0; c < 4; c++) {
        for (int z = 0; z <= c; z++) {
            staircase.emplace_back(0, (m_cfg.boxDim + m_cfg.boxDelta)*c, m_cfg.boxDim * z);
        }
    }
    std::deque<tf2::Vector3> tower;
    for (int z = 0; z < 4; z++) {
        tower.emplace_back(0, m_cfg.boxDim*3, m_cfg.boxDim * (4 + z));
    }
    auto modelTransform = [this](auto & pts) {
        std::transform(pts.begin(), pts.end(), pts.begin(),
            [this](tf2::Vector3 const & in) {
                // Mind the order of transformations
                // rotation and translation are not commutative
                // rotate first, to give the right orientation
                // then translate to place on table
                tf2::Quaternion rotation;
                rotation.setRotation(tf2::Vector3{0,0,1}, tf2Radians(m_cfg.staircaseRotation));
                auto result = tf2::quatRotate(rotation, in);
                result = result + m_cfg.positionOfFirstStep;
                return result;
            }
        );
    };
    // place staircase and tower on the table with the right orientation
    modelTransform(staircase);
    modelTransform(tower);

    std::vector<MoveCommand> commands;
    if (!m_cfg.isSimulation) {
        commands.push_back(InitGrippersCommand{});
    }

    LOG("Starting application");
    m_ros2Communicator->start();
    LOG("Waiting external bridge");
    waitExternalBridgeReady();
    LOG("External bridge ready");


    tf2::Vector3 angleAxis;
    m_roboArmExternalBridge->queryToolOrientation(angleAxis);
    auto [angle, axis] = decomposeAngleAxis(angleAxis);
    LOG("TCP angle: %f deg, axis: %f %f %f", tf2Degrees(angle), axis.x(), axis.y(), axis.z());

    std::deque<tf2::Vector3> boxes{m_cfg.boxes.begin(), m_cfg.boxes.end()};

    MovelCommand homePosition{m_cfg.homePoint, m_cfg.homeAngleAxis, m_cfg.acceleration, m_cfg.velocity, 0};
    commands.push_back(homePosition);


    // TODO: calculate maximum radius of each two segments
    // float radius = m_cfg.boxDim/2 - m_cfg.placeDelta.length();
    auto radius = m_cfg.radius;

    auto avoidBase = [&](tf2::Vector3 const & a, tf2::Vector3 const & b)
    {
        auto [dist, p] = distance(Line{a, b}, Line{tf2::Vector3(0,0,0), tf2::Vector3(0,0,1)});
        if (dist < m_cfg.safeOzRadius) {
            // if TCP will move too close the Oz, avoid it
            auto proj = tf2::Vector3{p.x(), p.y(), 0.0};
            auto t = m_cfg.safeOzRadius/proj.length();
            auto avoidOrigin = tf2::Vector3{p.x()*t, p.y()*t , p.z()};
            commands.push_back(MovelCommand{avoidOrigin, angleAxis, m_cfg.acceleration, m_cfg.velocity, radius});
        }
    };

    auto lastPoint = m_cfg.homePoint;
    while(!staircase.empty()) {
        auto targetPick = boxes.front();
        boxes.pop_front();
        auto approachPick = targetPick + m_cfg.approachPick;
        auto retreatPick = approachPick;

        avoidBase(lastPoint, approachPick);

        commands.push_back(MovelCommand{approachPick, m_cfg.boxesOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        commands.push_back(MovelCommand{targetPick, m_cfg.boxesOrientation, m_cfg.acceleration, m_cfg.velocity, 0});

        if (!m_cfg.isSimulation) {
            commands.push_back(CloseGrippersCommand{});
        }
        commands.push_back(MovelCommand{retreatPick, m_cfg.boxesOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        auto targetPlace = staircase.front();
        staircase.pop_front();
        auto approachPlace2 = targetPlace + m_cfg.approachPlace2;
        auto approachPlace = targetPlace + m_cfg.approachPlace;
        auto retreatPlace = targetPlace + m_cfg.approachPick;
        targetPlace += tf2::Vector3{0,0,m_cfg.boxDelta};

        avoidBase(approachPick, approachPlace);

        commands.push_back(MovelCommand{approachPlace, m_cfg.placeOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        commands.push_back(MovelCommand{approachPlace2, m_cfg.placeOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        commands.push_back(MovelCommand{targetPlace, m_cfg.placeOrientation, m_cfg.acceleration, m_cfg.velocity, 0});
        if (!m_cfg.isSimulation) {
            commands.push_back(OpenGrippersCommand{});
        }
        commands.push_back(MovelCommand{retreatPlace, m_cfg.placeOrientation, m_cfg.acceleration, m_cfg.velocity, radius});

        lastPoint = approachPlace;
    }

    // TODO: refactor duplication of code
    while(!tower.empty())
    {
        auto targetPick = boxes.front();
        boxes.pop_front();
        auto approachPick = targetPick + m_cfg.approachPick;
        auto retreatPick = approachPick;

        avoidBase(lastPoint, approachPick);

        commands.push_back(MovelCommand{approachPick, m_cfg.boxesOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        commands.push_back(MovelCommand{targetPick, m_cfg.boxesOrientation, m_cfg.acceleration, m_cfg.velocity, 0});

        if (!m_cfg.isSimulation) {
            commands.push_back(CloseGrippersCommand{});
        }
        commands.push_back(MovelCommand{retreatPick, m_cfg.boxesOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        auto targetPlace = tower.front();
        tower.pop_front();

        targetPlace += tf2::Vector3{0,0,m_cfg.boxDelta};
        auto approachPlace = targetPlace + m_cfg.approachPlace3;
        auto retreatPlace = approachPlace;

        avoidBase(approachPick, approachPlace);

        commands.push_back(MovelCommand{approachPlace, m_cfg.towerPlaceOrientation, m_cfg.acceleration, m_cfg.velocity, radius});
        commands.push_back(MovelCommand{targetPlace, m_cfg.towerPlaceOrientation, m_cfg.acceleration, m_cfg.velocity, 0});
        if (!m_cfg.isSimulation) {
            commands.push_back(OpenGrippersCommand{});
        }
        if (tower.empty()) {
            radius = 0;
        }
        commands.push_back(MovelCommand{retreatPlace, m_cfg.towerPlaceOrientation, m_cfg.acceleration, m_cfg.velocity, radius});

        lastPoint = approachPlace;
    }

    auto script = robotDo(commands, m_cfg.isSimulation ? m_URScriptSimulationHeader : m_URScriptHardwareHeader);
    writeCSV(commands, m_cfg.csvFilename);
    std::cout << "Executing script:" << std::endl << script << std::endl;
    m_roboArmExternalBridge->sendUrScript(script);
}
