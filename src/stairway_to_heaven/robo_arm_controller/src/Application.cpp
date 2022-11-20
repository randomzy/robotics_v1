#include "robo_arm_controller/Application.h"
#include "utils/Log.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <deque>
#include <sstream>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include "robo_arm_controller/Math.h"
#include <variant>

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
        m_URScriptHeader = load_file(projectInstallPrefix + "/resources/scripts/header.script");
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

// TODO: cleanup this mess

constexpr size_t joint_count = 6;
using JointDescription = std::array<float, joint_count>;

struct MovejCommand
{
    JointDescription pose{};
    float acceleration{0.f};
    float velocity{0.f};
};
struct MovelCommand
{
    tf2::Vector3 point{};
    tf2::Vector3 angleAxis{};
    float acceleration {0.f};
    float velocity {0.f};
    float radius {0.f};
};

struct InitGrippersCommand{};
struct OpenGrippersCommand{};
struct CloseGrippersCommand{};

std::ostream & operator << (std::ostream & os, MovejCommand const & mjc)
{
    os << "\tmovej([";
    const char * delim = "";
    for (const auto & j : mjc.pose) {
        os << delim << j; 
        delim = ",";
    }
    os << "],a=" << mjc.acceleration;
    os << ",v=" << mjc.velocity;
    os << ")\n";
    return os;
}

std::ostream & operator << (std::ostream & os, MovelCommand const & mlc)
{
    os << "\tmovel(";
    os << "p[" << mlc.point.x() << "," << mlc.point.y() << "," << mlc.point.z();
    os << "," << mlc.angleAxis.x() << "," << mlc.angleAxis.y() << "," << mlc.angleAxis.z() << "]";
    os << "," << "a=" << mlc.acceleration << ",v=" << mlc.velocity << ",t=0,r=" << mlc.radius;
    os << ")\n";
    return os;
}

std::ostream & operator << (std::ostream & os, [[maybe_unused]]InitGrippersCommand const & igc)
{
    os << "\trq_activate_and_wait()";
    return os;
}

std::ostream & operator << (std::ostream & os, [[maybe_unused]]OpenGrippersCommand const & igc)
{
    os << "\trq_open_and_wait()";
    return os;
}

std::ostream & operator << (std::ostream & os, [[maybe_unused]]CloseGrippersCommand const & igc)
{
    os << "\trq_close_and_wait()";
    return os;
}

using MoveCommand = std::variant<MovejCommand, MovelCommand, InitGrippersCommand, OpenGrippersCommand, CloseGrippersCommand>;

std::string robotDo(std::vector<MoveCommand> const & commands)
{
    std::stringstream ss;
    ss << "def move():\n";
    for (const auto & command : commands) {
        std::visit([&ss](auto && cmd){
            ss << cmd;
        }, command);
    }
    ss << "end" << std::endl;
    return ss.str();
}

std::string to_string(tf2::Vector3 const & v)
{
    std::stringstream ss;
    ss << v.x() << "," << v.y() << "," << v.z();
    return ss.str();
}

std::string toCSV(MovelCommand const & ml)
{
    return to_string(ml.point*100);
}

void writeCSV(std::vector<MoveCommand> const & commands, std::string const & csvFilename)
{
    std::ofstream ofs(csvFilename);
    for (const auto & command : commands) {
        std::visit([&ofs](auto const & cmd){
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, MovelCommand>) {
                ofs << toCSV(cmd) << "\n";
            }
        }, command);
    }
}

void Application::run()
{
    // generate staircase, the center of the first "step" is situated at (0,0,0)
    // and the step goes up in the positive Y direction
    std::deque<tf2::Vector3> staircase;
    for (int c = 0; c < 4; c++) {
        for (int z = 0; z <= c; z++) {
            staircase.emplace_back(0, m_cfg.boxDim*c, m_cfg.boxDim * z);
        }
    }
    // place staircase on the table with the right orientation
    std::transform(staircase.begin(), staircase.end(), staircase.begin(),
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

    auto boxes = m_cfg.boxes;
    std::reverse(boxes.begin(), boxes.end());

    MovelCommand homePosition{m_cfg.homePoint, m_cfg.homeAngleAxis, m_cfg.acceleration, m_cfg.velocity, 0};
    commands.push_back(homePosition);

    auto avoidBase = [&](tf2::Vector3 const & a, tf2::Vector3 const & b)
    {
        auto [dist, p] = distance(Line{a, b}, Line{tf2::Vector3(0,0,0), tf2::Vector3(0,0,1)});
        if (dist < m_cfg.safeOzRadius) {
            // if TCP will move too close the Oz, avoid it
            auto proj = tf2::Vector3{p.x(), p.y(), 0.0};
            auto t = m_cfg.safeOzRadius/proj.length();
            auto avoidOrigin = tf2::Vector3{p.x()*t, p.y()*t , p.z()};
            commands.push_back(MovelCommand{avoidOrigin, angleAxis, m_cfg.acceleration, m_cfg.velocity, 0});
        }
    };

    auto lastPoint = m_cfg.homePoint;
    while(!staircase.empty()) {
        auto targetPick = boxes.back();
        boxes.pop_back();
        auto approachPick = targetPick + m_cfg.approachOffset;

        avoidBase(lastPoint, approachPick);

        commands.push_back(MovelCommand{approachPick,angleAxis,m_cfg.acceleration, m_cfg.velocity, 0});
        commands.push_back(MovelCommand{targetPick,angleAxis,m_cfg.acceleration,m_cfg.velocity, 0});

        if (!m_cfg.isSimulation) {
            commands.push_back(CloseGrippersCommand{});
        }
        commands.push_back(MovelCommand{approachPick,angleAxis,m_cfg.acceleration,m_cfg.velocity, 0});
        auto targetPlace = staircase.front();
        staircase.pop_front();
        auto approachPlace = targetPlace + m_cfg.approachOffset;
        targetPlace += m_cfg.placeDelta;

        avoidBase(approachPick, approachPlace);

        commands.push_back(MovelCommand{approachPlace, angleAxis,m_cfg.acceleration,m_cfg.velocity,0});
        commands.push_back(MovelCommand{targetPlace, angleAxis,m_cfg.acceleration,m_cfg.velocity,0});
        if (!m_cfg.isSimulation) {
            commands.push_back(OpenGrippersCommand{});
        }
        commands.push_back(MovelCommand{approachPlace, angleAxis,m_cfg.acceleration,m_cfg.velocity,0});

        lastPoint = approachPlace;
    }
    auto script = robotDo(commands);
    writeCSV(commands, m_cfg.csvFilename);
    std::cout << "Executing script:" << std::endl << script << std::endl;
    // m_roboArmExternalBridge->sendUrScript(script);
}
