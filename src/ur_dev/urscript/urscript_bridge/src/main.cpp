#include "urscript_bridge/external_api/UrBridgeExternalInterface.h"

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Allow undeclared node parameters.
  rclcpp::NodeOptions nodeOptions;
  nodeOptions.allow_undeclared_parameters(true);
  nodeOptions.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<UrBridgeExternalInterface>(nodeOptions);
  rclcpp::executors::MultiThreadedExecutor e;
  e.add_node(node);
  e.spin();

  return EXIT_SUCCESS;
}