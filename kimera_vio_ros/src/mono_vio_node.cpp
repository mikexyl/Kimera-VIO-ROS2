#include "gflags/gflags.h"
#include "glog/logging.h"
#include "kimera_vio_ros/components/mono_vio.hpp"

#include <vector>

int main(int argc, char * argv[])
{
  auto g_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::vector<char *> g_argv;
  g_argv.reserve(g_args.size());
  for (auto & arg : g_args) {
    g_argv.push_back(arg.data());
  }
  int g_argc = static_cast<int>(g_argv.size());
  char ** g_argv_data = g_argv.data();

  google::ParseCommandLineFlags(&g_argc, &g_argv_data, true);
  google::InitGoogleLogging(argv[0]);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<kimera_vio_ros::components::MonoVio>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
