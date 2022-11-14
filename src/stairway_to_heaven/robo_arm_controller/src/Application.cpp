#include "robo_arm_controller/Application.h"
#include "utils/Log.h"

ErrorCode Application::init(ApplicationConfig const & config)
{
    m_ros2Communicator = std::make_shared<Ros2Communicator>();

    if (ErrorCode::FAILURE == m_ros2Communicator->init(config.ros2CommunicatorConfig)) {
        LOGERR("Could not initialize ros2Communcator");
        return ErrorCode::FAILURE;
    }

    if (ErrorCode::FAILURE == initRoboArmExternalBridge()) {
        LOGERR("Could not initialize MapExplorerExternalBridge");
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
    LOG("Starting application");
    m_ros2Communicator->start();
    waitExternalBridgeReady();
    tf2::Vector3 angleAxis;
    m_roboArmExternalBridge->queryToolOrientation(angleAxis);
    m_roboArmExternalBridge->sendUrScript("");
}
