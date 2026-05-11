#include <memory>

#include "kimera_vio_ros/components/mono_vio.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<kimera_vio_ros::components::MonoVio>(options);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
