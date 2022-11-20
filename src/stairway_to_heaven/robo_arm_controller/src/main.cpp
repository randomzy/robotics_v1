#include <rclcpp/rclcpp.hpp>

#include "utils/Log.h"
#include "robo_arm_controller/Application.h"

RoboArmConfig generateConfig()
{
    RoboArmConfig config;

    // execution policy must be RUN_IN_DEDICATED_THREAD:
    config.ros2CommunicatorConfig.executionPolicy = ExecutionPolicy::RUN_IN_DEDICATED_THREAD;
    config.ros2CommunicatorConfig.numberOfThreads = 0;
    config.ros2CommunicatorConfig.executorType = ExecutorType::MULTI_THREADED;
    
    return config;
}

int32_t main(int32_t argc, char ** argv)
{
    rclcpp::InitOptions initOptions;
    initOptions.shutdown_on_sigint = false;
    rclcpp::init(argc, argv);

    Application app;
    ErrorCode errorCode = app.init(generateConfig());
    if (ErrorCode::FAILURE == errorCode) {
        LOGERR("Could not initialize app");
        return EXIT_FAILURE;
    }
    app.run();

    app.deinit();
    
    rclcpp::shutdown();
    
    return EXIT_SUCCESS;
}