#include <cstdint>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include "utils/ErrorCode.h"

#include "robo_miner_controller/Application.h"

ApplicationConfig generateConfig()
{
    ApplicationConfig config;

    config.userdata.user = "Martin Tanchev";
    config.userdata.repository = "https://github.com/randomzy/robotics_v1.git";
    config.userdata.commitSha = "8751ac2ece8cfc9e703279617a79712b51923dab";

    // execution policy must be RUN_IN_DEDICATED_THREAD:
    config.ros2CommunicatorConfig.executionPolicy = ExecutionPolicy::RUN_IN_DEDICATED_THREAD;
    config.ros2CommunicatorConfig.numberOfThreads = 0;
    config.ros2CommunicatorConfig.executorType = ExecutorType::SINGLE_THREADED;
    
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